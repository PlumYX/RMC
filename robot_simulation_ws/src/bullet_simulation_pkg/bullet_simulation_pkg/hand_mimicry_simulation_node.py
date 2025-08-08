import os
import json
import copy

import pybullet
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class HandMimicrySimulationNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.pkg_name = "bullet_simulation_pkg"

        self.simple_subscriber = self.create_subscription(
            msg_type = String, 
            topic = "/hand_ctrl", 
            callback = self.callback,
            qos_profile = 10, 
            )

        # 延时回调
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.latest_msg_data = None
        self.msg_data_record = None
        self._init_env()

    def _init_env(self):
        """
        仿真环境初始化
        """
        pybullet.connect(pybullet.GUI)
        pybullet.setGravity(0, 0, -9.8)
        pybullet.resetDebugVisualizerCamera(
            cameraDistance = 1, 
            cameraYaw = 0, 
            cameraPitch = -40, 
            cameraTargetPosition = [0.1, -0.3, 0.1], 
            )

        # 加载地面
        self.sceneUid = pybullet.loadURDF(
            os.path.join(
                get_package_share_directory(self.pkg_name), 
                "urdf", 
                "plane.urdf", 
                ), 
            basePosition = [0, 0, -0.65],
            )
        if self.sceneUid == -1:
            self.get_logger().error("地面加载失败!")

        # 加载桌子
        self.tableUid = pybullet.loadURDF(
            os.path.join(
                get_package_share_directory(self.pkg_name), 
                "urdf", 
                "table.urdf", 
                ), 
            basePosition=[0.5, 0, -0.65], 
            )
        if self.tableUid == -1:
            self.get_logger().error("桌子加载失败!")

        # 加载杯子
        self.tableUid = pybullet.loadURDF(
            os.path.join(
                get_package_share_directory(self.pkg_name), 
                "urdf", 
                "cup.urdf", 
                ), 
            basePosition = [0.8, 0, 0], 
            useFixedBase=True,
            )
        if self.tableUid == -1:
            self.get_logger().error("杯子加载失败!")

        # 加载灵巧手(右)
        self.dexh13RightUid = pybullet.loadURDF(
            os.path.join(
                get_package_share_directory(self.pkg_name), 
                "urdf", 
                "dexh13_right.urdf", 
                ), 
            basePosition=[0, 0, 0], 
            baseOrientation=[0, -1, 0, 1], 
            useFixedBase=True,
            )
        if self.dexh13RightUid == -1:
            self.get_logger().error("灵巧手(右)加载失败!")

    def callback(self, msg):
        """
        消息记录
        """
        self.latest_msg_data = msg.data

    def timer_callback(self):
        """
        延时回调
        """
        if self.latest_msg_data:
            msg_data = json.loads(self.latest_msg_data)

            if msg_data != self.msg_data_record:
                self.msg_data_record = copy.deepcopy(msg_data)
                self.get_logger().info(f"订阅数据: {msg_data}, 类型: {type(msg_data)}")
                self.set_joint_positions_angle(msg_data)

    def set_joint_positions_angle(self, msg_data):
        """
        角度控制
        """
        for i, angle in enumerate(msg_data):
            pybullet.setJointMotorControl2(
                self.dexh13RightUid,
                jointIndex = i,
                controlMode = pybullet.POSITION_CONTROL,
                targetPosition = (angle / 180) * 3.1415,
                force = 100,
            )
        pybullet.stepSimulation()


def main():
    rclpy.init()

    node = HandMimicrySimulationNode("hand_mimicry_simulation_subscriber")

    rclpy.spin(node)
    rclpy.shutdown()
