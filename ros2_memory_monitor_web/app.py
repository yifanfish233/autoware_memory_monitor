# Organized imports
import sys
import time
from flask import Flask, jsonify, render_template
import threading
import rclpy
import os  # For environment variables

# Local imports
sys.path.append('../ros2_memory_monitor_py/ros2_memory_monitor_py')
from ros2_memory_monitor_py.ros2_memory_monitor_py.memory_monitor import NodeMonitor
from ros2_memory_monitor_py.ros2_jitter_monitor_py.jitter_monitor import (
    JitterMonitor,
    get_topic_info,
    get_msg_module,
    list_topics
)

app = Flask(__name__)

# Use configurations
app.config['LOG_DIRECTORY'] = os.environ.get('LOG_DIRECTORY', '/home/robotlab/.ros/log/')

@app.route('/')
def home():
    return render_template('index.html')
@app.route('/api/memory_usage/<node>')
def get_memory_usage(node):
    memory = node_monitor.get_memory_usage(node)
    if memory is not None:
        return jsonify({'node': node, 'memory': memory})
    else:
        return jsonify({'error': 'No such node'}), 404

@app.route('/api/memory_usage_history/<node>')
def get_memory_usage_history(node):
    history = node_monitor.get_memory_usage_history(node)
    if history is not None:
        return jsonify(history)
    else:
        return jsonify({'error': 'No such node'}), 404

@app.route('/api/memory_usage_stats/<node>')
def get_memory_usage_stats(node):
    stats = node_monitor.get_memory_usage_stats(node)
    if stats is not None:
        return jsonify(stats)
    else:
        return jsonify({'error': 'No such node'}), 404

@app.route('/api/memory_current_usage_top10')
def get_current_memory_usage_top10():
    memory_usage_current = [(node, memory) for node, memory in node_monitor.memory_usage_current.items()]
    memory_usage_current.sort(key=lambda x: x[1], reverse=True)
    top10 = memory_usage_current[:10]
    return jsonify(top10)
@app.route('/api/nodes')
def get_nodes():
    nodes = list(node_monitor.pid_node_dict.keys())
    return jsonify(nodes)

@app.route('/api/manual_update', methods=['POST'])
def manual_update():
    node_monitor.manual_update_stats()
    return '', 204


@app.errorhandler(500)
def internal_error(error):
    return jsonify({'error': 'Internal server error'}), 500
def start_memory_monitoring(node_monitor):
    while True:
        node_monitor.monitor()
        time.sleep(1)  # 等待1秒再次监控


# New route to start jitter monitoring for a specific topic
@app.route('/api/topics')
def api_list_topics():
    return jsonify(list_topics())

@app.route('/api/start_jitter_monitor/<topic>')
def start_jitter_monitor(topic):
    topic_type = get_topic_info(topic)
    if not topic_type:
        return jsonify({'error': f"Failed to get topic type for '{topic}'"}), 404

    msg_module = get_msg_module(topic_type)
    if not msg_module:
        return jsonify({'error': f"Failed to get message module for topic type '{topic_type}'"}), 404

    # Ensure rclpy is initialized before creating a node
    if not rclpy.ok():
        rclpy.init()

    jitter_monitor = JitterMonitor(topic, msg_module)
    threading.Thread(target=lambda: rclpy.spin(jitter_monitor)).start()
    return jsonify({'status': 'Jitter monitoring started'}), 200


if __name__ == "__main__":
    print("Starting monitors...")

    # Memory Monitor Initialization
    node_monitor = NodeMonitor(app.config['LOG_DIRECTORY'], "WARNING")
    memory_thread = threading.Thread(target=start_memory_monitoring, args=(node_monitor,))
    memory_thread.daemon = True
    memory_thread.start()

    # Initialize ROS for potential jitter monitoring
    rclpy.init()

    # Start Flask server
    app.run()

    # Shutdown ROS at the end
    rclpy.shutdown()

