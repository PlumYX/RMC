import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisherNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._publisher = self.create_publisher(
            msg_type = String, 
            topic = "/simple_topic", 
            qos_profile = 10,
            )
        self._timer = self.create_timer(1.5, self._timer_callback)
        self._count = 0
        self.get_logger().info(f"发布节点 {node_name} 已启动!")

    def _timer_callback(self):
        msg = String()
        self._count += 1
        msg.data = f"id({self._count}) 这是一个简单的 python 发布节点!"
        self._publisher.publish(msg)
        self.get_logger().info(f"发布消息: {msg.data}")


def main():
    rclpy.init()
    node = SimplePublisherNode("python_simple_publisher")
    rclpy.spin(node)
    rclpy.shutdown()
