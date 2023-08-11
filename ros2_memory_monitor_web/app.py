import sys
import time

sys.path.append('../ros2_memory_monitor_py/ros2_memory_monitor_py')
from ros2_memory_monitor_py.ros2_memory_monitor_py.monitor import NodeMonitor
from flask import Flask, jsonify, render_template
import threading

app = Flask(__name__)
log_directory = '/home/robotlab/.ros/log/'  # 替换为实际的日志目录

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
def start_monitoring(node_monitor):
    while True:
        node_monitor.monitor()
        time.sleep(1)  # 等待1秒再次监控

if __name__ == "__main__":
    print("start monitor")
    node_monitor = NodeMonitor(log_directory, "WARNING")
    monitor_thread = threading.Thread(target=start_monitoring, args=(node_monitor,)) # 注意在node_monitor后面添加一个逗号
    monitor_thread.daemon = True
    monitor_thread.start()
    # 启动Flask服务器
    app.run()
