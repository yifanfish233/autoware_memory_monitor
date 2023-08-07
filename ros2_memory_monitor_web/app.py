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

@app.route('/api/memory_usage_top10')
def get_memory_usage_top10():
    memory_usage_current = [(node, memory) for node, memory in node_monitor.memory_usage_current.items()]
    memory_usage_current.sort(key=lambda x: x[1], reverse=True)  # 按照内存使用量降序排列
    top10 = memory_usage_current[:10]  # 获取内存使用量最高的10个节点
    return jsonify(top10)

@app.route('/api/ros2_status')
def get_ros2_status():
    status = node_monitor.get_ros2_status()
    return jsonify({'status': status})

@app.route('/api/nodes')
def get_nodes():
    nodes = list(node_monitor.pid_node_dict.keys())
    return jsonify(nodes)

@app.route('/api/refresh', methods=['POST'])
def refresh():
    node_monitor.update_stats()
    return '', 204

if __name__ == "__main__":
    print("start monitor")
    node_monitor = NodeMonitor(log_directory, "Info")

    # 在新的线程中启动监控程序，以便主线程可以执行其他任务
    monitor_thread = threading.Thread(target=node_monitor.monitor)
    monitor_thread.start()

    # 启动Flask服务器
    app.run()
