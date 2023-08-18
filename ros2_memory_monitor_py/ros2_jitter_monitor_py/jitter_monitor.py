import sys
import rclpy
from rclpy.node import Node
import importlib
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy

class JitterMonitor(Node):
    def __init__(self, topic_name):
        super().__init__('jitter_monitor')

        # Get topic type dynamically
        topic_type = self.get_topic_type(topic_name)
        msg_module = self.get_msg_module(topic_type)

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

    def get_msg_module(self, topic_type):
        parts = topic_type.split('/')
        # The last part is the message name and everything before is the package name
        msg_name = parts[-1]
        package_name = parts[0]

        try:
            module = importlib.import_module(f'{package_name}.msg')
            msg_module = getattr(module, msg_name)
            return msg_module
        except ModuleNotFoundError:
            print(f"Failed to import module for topic type '{topic_type}'")
            sys.exit(1)

    def get_topic_type(self, topic_name):
        temp_node = rclpy.create_node('temp_node_for_getting_topic_type')
        time.sleep(2)  # Give some time for discovery
        topic_list = temp_node.get_topic_names_and_types()
        temp_node.destroy_node()
        for t_name, t_types in topic_list:
            if t_name == topic_name:
                return t_types[0] if t_types else None
        return None

    def listener_callback(self, msg):
        current_time = time.time()
        if self.prev_time:
            jitter = current_time - self.prev_time
            self.get_logger().info(f'Topic: {self.subscription.topic_name}, Jitter: {jitter * 1000:.2f} ms')
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    topic_name = input("Enter the name of the topic you want to monitor: ")
    jitter_monitor = JitterMonitor(topic_name)
    rclpy.spin(jitter_monitor)

    jitter_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
