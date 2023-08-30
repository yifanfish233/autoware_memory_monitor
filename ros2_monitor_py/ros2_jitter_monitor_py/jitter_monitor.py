from rclpy.node import Node
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy

class JitterMonitor(Node):
    def __init__(self, topic_name, msg_module):
        super().__init__('jitter_monitor')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            msg_module,
            topic_name,
            self.listener_callback,
            qos_profile
        )
        self.prev_time = None

    def listener_callback(self, msg):
        current_time = time.time()
        if self.prev_time:
            jitter = current_time - self.prev_time
            self.get_logger().info(f'Topic: {self.subscription.topic_name}, Jitter: {jitter * 1000:.2f} ms')
        self.prev_time = current_time


