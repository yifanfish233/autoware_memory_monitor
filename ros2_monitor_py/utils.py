import rclpy
import importlib
import time

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
    temp_node_name = "temp_node_for_interaction_" + str(time.time()).replace('.', '_')
    temp_node = rclpy.create_node(temp_node_name)
    time.sleep(2)  # Give some time for discovery
    topics = temp_node.get_topic_names_and_types()
    temp_node.destroy_node()
    return topics

def get_total_topics():
    return len(list_topics())
