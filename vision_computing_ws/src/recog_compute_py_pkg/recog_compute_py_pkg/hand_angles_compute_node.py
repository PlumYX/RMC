import mediapipe as mp
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def vectors_angle(v1, v2):
    """
    计算两个向量之间的夹角

    Args:
        v1, v2: np.ndarray 输入向量

    Return:
        angle_deg: float 向量夹角
    """
    # 计算点积
    dot_product = np.dot(v1, v2)

    # 计算向量的L2范数(模长)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)

    # 计算夹角余弦(防止浮点误差超出[-1,1]范围)
    cos_theta = np.clip(dot_product / (norm_v1 * norm_v2), -1.0, 1.0)

    # 计算反余弦得到弧度值
    angle_rad = np.arccos(cos_theta)

    # 弧度转角度
    angle_deg = np.degrees(angle_rad)
    return angle_deg


def pip_joint_angle(mcp_joint, proximal_joint, distal_joint):
    """
    计算手指近端关节角

    Args:
        mcp_joint: list 手指根关节坐标
        proximal_joint: list 手指近端关节坐标
        distal_joint: list 手指远端关节坐标

    Returns:
        proximal_joint_angle: float 近端指关节角
    """
    # 计算指根指向近端关节向量
    metacarpophalangeal_to_proximal_vector = np.array(mcp_joint) - np.array(proximal_joint)

    # 计算近端关节指向远端关节向量
    proximal_to_distal_vector = np.array(proximal_joint) - np.array(distal_joint)

    # 计算近端指关节角
    proximal_joint_angle = vectors_angle(
        metacarpophalangeal_to_proximal_vector, 
        proximal_to_distal_vector, 
        )
    return proximal_joint_angle


def projection_vector(v, n):
    """
    通过法向量 n 计算 v 的投影向量

    Arge: 
        v: np.array 向量
        n: np.array 法向量
    
    Return:
        v_projection: np.array 投影向量
    """
    # 计算法向量模长平方
    n_norm_squared = np.dot(n, n)

    # 计算投影长度
    projection_length = np.abs(np.dot(v, n) / n_norm_squared)
    normal_component = projection_length * n

    # 计算投影向量
    projected_vector = v - normal_component
    return projected_vector


def mcp_joint_angle(wrist_joint, mcp_joint, proximal_joint, adjacent_mcp_joint):
    """
    计算手指根关节角

    Args:
        wrist_joint: list 手腕关节坐标
        mcp_joint: list 手指根关节坐标
        proximal_joint: list 手指近端关节坐标
        adjacent_mcp_joint: list 相邻手指根关节坐标

    Return:
        mcp_joint_up_angle: float 手指根关节仰角
        mcp_joint_yaw_angle: float 手指根关节偏角
    """
    # 计算手掌平米两交叉向量
    wrist_to_mcp_vector = np.array(wrist_joint) - np.array(mcp_joint)
    mcp_to_adjacent_vector = np.array(mcp_joint) - np.array(adjacent_mcp_joint)

    # 计算手掌平面法向量
    normal_vector = np.cross(wrist_to_mcp_vector, mcp_to_adjacent_vector)

    # 计算指根关节仰角
    mcp_to_proximal_vector = np.array(proximal_joint) - np.array(mcp_joint)
    mcp_joint_up_angle = vectors_angle(mcp_to_proximal_vector, normal_vector)
    mcp_joint_up_angle = 180 - mcp_joint_up_angle if mcp_joint_up_angle > 90 \
        else mcp_joint_up_angle
    mcp_joint_up_angle = 90 - mcp_joint_up_angle

    # 计算指根关节偏角
    proj_vector = projection_vector(mcp_to_proximal_vector, normal_vector)
    mcp_joint_yaw_angle = 180 - vectors_angle(proj_vector, wrist_to_mcp_vector)

    return mcp_joint_up_angle, 20-mcp_joint_yaw_angle


def get_joint_coordinates(hand_landmarks, joint_index):
    """
    获取关节坐标

    Args:
        hand_landmarks: class 手指关节识别类
        joint_index: int 关节索引

    Return:
        joint_coordinates: list 关节坐标
    """
    joint_coordinates = [
        hand_landmarks.landmark[joint_index].x, 
        hand_landmarks.landmark[joint_index].y, 
        hand_landmarks.landmark[joint_index].z, 
        ]
    return joint_coordinates


def get_finger_coordinates(hand_landmarks, finger):
    """
    获取手指所有关节坐标

    Args:
        hand_landmarks: class 手指关节识别类
        finger: int 0: 拇指, 1: 食指, 2: 中指, 3: 无名指, 4: 小拇指

    Return:
        mcp_joint: 指根关节坐标
        proximal_joint: 近端关节坐标
        distal_joint: 远端关节坐标
        tip_joint: 指尖关节坐标
    """
    index = 4 * finger

    mcp_joint = get_joint_coordinates(hand_landmarks, 1+index)
    proximal_joint = get_joint_coordinates(hand_landmarks, 2+index)
    distal_joint = get_joint_coordinates(hand_landmarks, 3+index)
    tip_joint = get_joint_coordinates(hand_landmarks, 4+index)

    return mcp_joint, proximal_joint, distal_joint, tip_joint


def clip(value, min_val, max_val):
    """
    整数截断

    Arga:
        value: int 整数
        min_val: int 最小值
        max_val: int 最大值
    
    Return:
        : int 截断后的值
    """
    return max(min_val, min(value, max_val))


class JointAnglesComputeNode(Node):
    """
    摄像头检查手指关节 --> 计算角度 --> 发布
    """
    def __init__(self, node_name):
        super().__init__(node_name)

        self.simple_publisher_ = self.create_publisher(
            msg_type=String, 
            topic="/hand_ctrl", 
            qos_profile=10,
            )

        # 初始化MediaPipe Hands模块
        mp_hands = mp.solutions.hands
        self.hands = mp_hands.Hands(
            static_image_mode=False,       # 是否静态图像(False表示实时视频)
            max_num_hands=1,               # 最多检测的手部数量
            min_detection_confidence=0.5,  # 检测置信度阈值
            min_tracking_confidence=0.5,   # 追踪置信度阈值
        )

        # 初始化绘图工具（用于可视化关键点）
        mp_drawing = mp.solutions.drawing_utils

        # 打开摄像头
        self.cap = cv2.VideoCapture(0)

        # 实例化消息
        msg = String()

        while self.cap.isOpened():
            success, image = self.cap.read()
            if not success:
                continue

            # 将BGR图像转为RGB（MediaPipe需要RGB输入）
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
            # 处理图像并获取手部关键点
            results = self.hands.process(image_rgb)

            # 如果有检测到手部，绘制关键点和连接线
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        image, 
                        hand_landmarks, 
                        mp_hands.HAND_CONNECTIONS, 
                        )

                    # 获取手腕关节坐标
                    wrist_joint = get_joint_coordinates(hand_landmarks, 0)

                    # 获取母指关节坐标
                    (
                        thumb_mcp_joint, 
                        thumb_proximal_joint, 
                        thumb_distal_joint, 
                        thumb_tip_joint,
                    ) = get_finger_coordinates(hand_landmarks, 0)

                    # 获取食指关节坐标
                    (
                        index_finger_mcp_joint, 
                        index_finger_proximal_joint, 
                        index_finger_distal_joint, 
                        index_finger_tip_joint,
                    ) = get_finger_coordinates(hand_landmarks, 1)

                    # 获取中指关节坐标
                    (
                        middle_finger_mcp_joint, 
                        middle_finger_proximal_joint, 
                        middle_finger_distal_joint, 
                        middle_finger_tip_joint, 
                    ) = get_finger_coordinates(hand_landmarks, 2)

                    # 获取无名指关节坐标
                    (
                        ring_finger_mcp_joint, 
                        ring_finger_proximal_joint, 
                        ring_finger_distal_joint, 
                        ring_finger_tip_joint, 
                    ) = get_finger_coordinates(hand_landmarks, 3)

                    # 计算拇指角度
                    thumb_mcp_up_angle, thumb_mcp_yaw_angle = mcp_joint_angle(
                        wrist_joint,
                        thumb_mcp_joint, 
                        thumb_distal_joint, 
                        index_finger_mcp_joint,
                        )
                    # self.get_logger().info(f"拇指根关节仰角: {thumb_mcp_up_angle}, 偏角: {thumb_mcp_yaw_angle}")

                    thumb_proximal_angle = pip_joint_angle(
                        thumb_mcp_joint, 
                        thumb_proximal_joint, 
                        thumb_distal_joint, 
                        )
                    thumb_distal_angle = pip_joint_angle(
                        thumb_proximal_joint, 
                        thumb_distal_joint, 
                        thumb_tip_joint,
                        )
                    # self.get_logger().info(f"拇指近端关节角: {thumb_proximal_angle}, 远端关节角: {thumb_distal_angle}")

                    # 计算食指角度
                    index_finger_mcp_up_angle, index_finger_mcp_yaw_angle = mcp_joint_angle(
                        wrist_joint,
                        index_finger_mcp_joint, 
                        index_finger_distal_joint, 
                        middle_finger_mcp_joint, 
                        )
                    # self.get_logger().info(f"食指根关节仰角: {index_finger_mcp_up_angle}, 偏角: {index_finger_mcp_yaw_angle}")

                    index_finger_proximal_angle = pip_joint_angle(
                        index_finger_mcp_joint, 
                        index_finger_proximal_joint, 
                        index_finger_distal_joint, 
                        )
                    index_finger_distal_angle = pip_joint_angle(
                        index_finger_proximal_joint, 
                        index_finger_distal_joint, 
                        index_finger_tip_joint, 
                        )
                    # self.get_logger().info(f"食指近端关节角: {index_finger_proximal_angle}")

                    # 计算中指角度
                    middle_finger_mcp_up_angle, middle_finger_mcp_yaw_angle = mcp_joint_angle(
                        wrist_joint,
                        middle_finger_mcp_joint, 
                        middle_finger_distal_joint, 
                        index_finger_mcp_joint, 
                        )
                    # self.get_logger().info(f"中指根关节仰角: {middle_finger_mcp_up_angle}, 偏角: {middle_finger_mcp_yaw_angle}")

                    middle_finger_proximal_angle = pip_joint_angle(
                        middle_finger_mcp_joint, 
                        middle_finger_proximal_joint, 
                        middle_finger_distal_joint, 
                        )
                    middle_finger_distal_angle = pip_joint_angle(
                        middle_finger_proximal_joint, 
                        middle_finger_distal_joint, 
                        middle_finger_tip_joint, 
                        )
                    # self.get_logger().info(f"中指近端关节角: {middle_finger_proximal_angle}")

                    # 无名指角度计算
                    ring_finger_mcp_up_angle, ring_finger_mcp_yaw_angle = mcp_joint_angle(
                        wrist_joint,
                        ring_finger_mcp_joint, 
                        ring_finger_distal_joint, 
                        middle_finger_mcp_joint, 
                        )
                    # self.get_logger().info(f"无名指根关节仰角: {ring_finger_mcp_up_angle}, 偏角: {ring_finger_mcp_yaw_angle}")

                    ring_finger_proximal_angle = pip_joint_angle(
                        ring_finger_mcp_joint, 
                        ring_finger_proximal_joint, 
                        ring_finger_distal_joint, 
                        )
                    ring_finger_distal_angle = pip_joint_angle(
                        ring_finger_proximal_joint, 
                        ring_finger_distal_joint, 
                        ring_finger_tip_joint, 
                        )
                    # self.get_logger().info(f"无名指近端关节角: {ring_finger_proximal_angle}")

                    target_list = [
                        clip(int(thumb_mcp_yaw_angle * 3.0), -20, 20),
                        clip(int(thumb_mcp_up_angle * 3.0), 0, 90),
                        clip(int(thumb_proximal_angle * 2.0), 0, 90),
                        clip(int(thumb_distal_angle * 2.0), 0, 90),

                        clip(int((index_finger_mcp_yaw_angle + 10) * 1.5), -20, 20),
                        clip(int(index_finger_mcp_up_angle * 1.5), 0, 90),
                        clip(int(index_finger_proximal_angle * 2.0), 0, 90),
                        clip(int(index_finger_distal_angle * 2.0), 0, 90),

                        clip(int(middle_finger_mcp_yaw_angle * 1.5), -20, 20),
                        clip(int(middle_finger_mcp_up_angle * 1.5), 0, 90),
                        clip(int(middle_finger_proximal_angle * 2.0), 0, 90),
                        clip(int(middle_finger_distal_angle * 2.0), 0, 90),

                        clip(int((ring_finger_mcp_yaw_angle - 10) * 1.5), -20, 20),
                        clip(int(ring_finger_mcp_up_angle * 1.5), 0, 90),
                        clip(int(ring_finger_proximal_angle * 2.0), 0, 90),
                        clip(int(ring_finger_distal_angle * 2.0), 0, 90),
                        ]

                    # 发布数据
                    msg.data = str(target_list)
                    self.simple_publisher_.publish(msg)
                    self.get_logger().info(f"发布数据: {msg.data}")

            # 显示结果
            cv2.imshow('Hand Tracking', image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # 释放资源
        self.cap.release()
        cv2.destroyAllWindows()


def main():
    rclpy.init()

    node = JointAnglesComputeNode("hand_angles_compute_publisher")

    rclpy.spin(node)
    rclpy.shutdown()
