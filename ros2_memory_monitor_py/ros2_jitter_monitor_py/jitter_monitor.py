import sys
import rclpy
from rclpy.node import Node
import importlib
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

def get_msg_module(topic_type):
    parts = topic_type.split('/')
    msg_name = parts[-1]
    package_name = parts[0]

    try:
        module = importlib.import_module(f'{package_name}.msg')
        return getattr(module, msg_name)
    except ModuleNotFoundError:
        print(f"Failed to import module for topic type '{topic_type}'")
        return None

def get_topic_info(topic_name):
    topics = list_topics()
    for t_name, t_types in topics:
        if t_name == topic_name:
            return t_types[0] if t_types else None
    return None

def list_topics():
    temp_node = rclpy.create_node('temp_node_for_interaction')
    time.sleep(2)  # Give some time for discovery
    topics = temp_node.get_topic_names_and_types()
    temp_node.destroy_node()
    return topics

