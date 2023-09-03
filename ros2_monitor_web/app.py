# Organized imports
import sys
import time
from datetime import datetime

from flask import Flask, jsonify, render_template
import threading
import rclpy
import os  # For environment variables
from flask_socketio import SocketIO, emit

# Local imports
sys.path.append('/home/robotlab/Desktop/autoware_memory_monitor/ros2_monitor_py')
from ros2_memory_monitor_py.memory_monitor import NodeMonitor
from ros2_jitter_monitor_py.jitter_monitor import JitterMonitor
from utils import (
    get_topic_info,
    get_msg_module,
    list_topics,
    get_total_topics
)
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")


# Use configurations
app.config['LOG_DIRECTORY'] = os.environ.get('LOG_DIRECTORY', '/home/robotlab/.ros/log/')

@app.route('/')
def home():
    return render_template('index.html')

@app.errorhandler(500)
def internal_error(error):
    return jsonify({'error': 'Internal server error'}), 500

def top_info_task(node_monitor):
    rclpy.init()
    while True:
        time.sleep(1)
        current_time = int(time.time() * 1000)  # current time in ms
        total_nodes = node_monitor.get_total_nodes()
        total_topics = get_total_topics()
        total_memory_usage = node_monitor.get_total_memory_usage()
        data = {
            'total_nodes': total_nodes,
            'total_topics': total_topics,
            'total_memory_usage': total_memory_usage,
            'current_time': current_time
        }
        socketio.emit('test_response', data)
    rclpy.shutdown()

@app.route('/get_modules', methods=['GET'])
def get_modules():
    modules = node_monitor.get_modules()
    return jsonify({"modules": modules})

if __name__ == "__main__":
    print("Starting monitors...")
    # Memory Monitor Initialization
    node_monitor = NodeMonitor(app.config['LOG_DIRECTORY'], "WARNING")

    print("Client connected")
    socketio.start_background_task(top_info_task, node_monitor)
    # Start Flask server
    socketio.run(app, host='0.0.0.0', port=5000)