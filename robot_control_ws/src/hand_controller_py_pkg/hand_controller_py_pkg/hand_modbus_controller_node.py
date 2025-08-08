# import numpy as np
import json
import copy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pxdex.dh13 import DexH13Control
from pxdex.dh13 import ControlMode,FingerAngle

# nonemode = ControlMode.NONE_CONTROL_MODE
torquemode = ControlMode.TORQUE_CONTROL_MODE
speedmode = ControlMode.SPEED_CONTROL_MODE
positionmode = ControlMode.POSITION_CONTROL_MODE


class HandControllerNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        handy_port_num="/dev/ttyUSB0"
        camera_port_num="none"

        self.simple_subscriber = self.create_subscription(
            msg_type=String, 
            topic="/hand_ctrl", 
            callback=self.callback,
            qos_profile=10, 
            )

        self.create_timer(0.3, self.timer_callback)
        self.latest_msg_data = None
        self.msg_data_record = None

        # 创建连接手的控制器
        self.control = DexH13Control()

        # 连接手
        dev = self.control.activeHandy(handy_port_num, camera_port_num)
        self.get_logger().info(f"设备: {dev}")

    def callback(self, msg):
        self.latest_msg_data = msg.data
    
    def timer_callback(self):
        if self.latest_msg_data:
            msg_data = json.loads(self.latest_msg_data)
            if msg_data != self.msg_data_record:
                self.msg_data_record = copy.deepcopy(msg_data)
                self.get_logger().info(f"订阅数据: {msg_data}, 类型: {type(msg_data)}")
                self.set_joint_positions_angle(msg_data)

    def check_health(self):
        # 连接验证
        sigConnected = self.control.isConnectHandy()
        self.get_logger().info(f"连接状态: {sigConnected}")

        # 固件版本验证
        firmwareVersion = self.control.getFirmwareVersion()
        self.get_logger().info(f"固件版本: {firmwareVersion}")

        # SDK版本验证
        SDKVersion = self.control.getSDKVersion()
        self.get_logger().info(f"SDK版本: {SDKVersion}")

    def init_motor_zero(self):
        # 复位
        initMotorPosition = self.control.initMotorPosition()
        self.get_logger().info(f"复位: {initMotorPosition}")

    def set_control_mode(self, control_mode):
        # 禁用电机
        en = self.control.disableMotor()
        self.get_logger().info(f"禁用电机: {en}")
        sig_enabled = self.control.isMotorEnabled()
        self.get_logger().info(f"电机是否禁用: {sig_enabled}")

        # 设置控制模式(需在电机禁用状态下进行设置)
        self.control.setMotorControlMode(control_mode)
        self.get_logger().info(f"设置控制模式: {control_mode}")

        # 启用电机
        en = self.control.enableMotor()
        self.get_logger().info(f"启用电机: {en}")
        sigEnabled = self.control.isMotorEnabled()
        self.get_logger().info(f"电机是否已启用: {sigEnabled}")

        # 查询控制模式(需在电机启用后进行查询)
        getMode = self.control.getMotorControlMode()
        self.get_logger().info(f"获取控制模式: {getMode}")

    def set_joint_positions_angle(self, target_list):
        """ 
        target_list = [拇指, 食指, 中指, 无名指] 
        *指 = [偏角, 仰角, 近端关节角, 远端关节角]
        """
        if len(target_list) != 16:
            self.get_logger().info("列表长度不是 16!")
            return 

        for i in range(1, 4):
            target_list[3 + i * 4] = 0
        target = target_list[4:] + target_list[:4]

        finger_angles = [FingerAngle() for _ in range(4)]
        for i, finger_angle in enumerate(finger_angles):
            (
                finger_angle.joint1, 
                finger_angle.joint2, 
                finger_angle.joint3, 
                finger_angle.joint4, 
            ) = target[i * 4: (i + 1) * 4]
        return self.control.setJointPositionsAngle(finger_angles)


def main():
    rclpy.init()
    node = HandControllerNode("hand_modbus_control_subscriber")
    node.check_health()
    node.init_motor_zero()
    node.set_control_mode(positionmode)
    rclpy.spin(node)
    rclpy.shutdown()
