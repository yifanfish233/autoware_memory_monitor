import os
import psutil
import re
import statistics
import threading
import time
import logging

# Node内存使用情况监控类
class NodeMonitor:
    def __init__(self, log_directory, log_level):
        self.log_directory = log_directory
        self.pid_node_dict = {}
        self.memory_usage_history = {}  # 存储内存使用历史记录
        self.memory_usage_current = {}  # 记录当前内存使用情况
        self.is_updated = threading.Event()  # 添加了一个Event对象，用于表示状态是否已更新
        self.update_stats()

        # 设置日志级别
        numeric_level = getattr(logging, log_level.upper(), None)
        if not isinstance(numeric_level, int):
            raise ValueError('Invalid log level: %s' % log_level)
        logging.basicConfig(level=numeric_level)
        self.update_stats()

    # 更新统计数据
    def update_stats(self):
        log_file = self.get_latest_log_file()
        self.pid_node_dict = self.extract_pid_from_log(log_file)
        self.is_updated.set()  # 设置状态已更新
        self.monitor()

    # 获取最新的日志文件
    def get_latest_log_file(self):
        log_folders = [f.path for f in os.scandir(self.log_directory) if f.is_dir()]
        latest_folder = max(log_folders, key=os.path.getmtime)
        log_file = os.path.join(latest_folder, 'launch.log')
        return log_file

    # 从日志文件中提取进程ID
    def extract_pid_from_log(self, log_file, update=False):
        pid_node_dict = {}
        with open(log_file, 'r') as file:
            lines = file.readlines()
            for line in lines:
                # 修改正则表达式以匹配整个节点名称和数字，并从行中捕获它们
                match = re.search(r'\[INFO\] \[(.+)\]: process started with pid \[(\d+)\]', line)
                if match:
                    # 匹配整个节点名称，包括数字
                    node_name_with_number = match.group(1)
                    pid = int(match.group(2))  # PID现在是第二组
                    if update and node_name_with_number in pid_node_dict and pid_node_dict[
                        node_name_with_number] != pid:
                        logging.warning(
                            f"Updated PID for node {node_name_with_number}: {pid_node_dict[node_name_with_number]} -> {pid}")
                    pid_node_dict[node_name_with_number] = pid
                    if node_name_with_number not in self.memory_usage_history:
                        self.memory_usage_history[node_name_with_number] = {}
                    if pid not in self.memory_usage_history[node_name_with_number]:
                        self.memory_usage_history[node_name_with_number][pid] = []
        return pid_node_dict

    # 获取ROS2的状态
    def get_ros2_status(self):
        log_file_path = self.get_latest_log_file()
        with open(log_file_path, 'r') as file:
            lines = file.readlines()
            last_lines = lines[-100:]
            if any('[ERROR]' in line and 'process has died' in line for line in last_lines):
                return 'waiting'
            else:
                return 'running'

    # 监控内存使用
    def monitor(self):
        new_pid_node_dict = {}
        for node, pid in self.pid_node_dict.items():
            try:
                process = psutil.Process(pid)
                mem_info = process.memory_info()
                mem_usage = mem_info.rss / 1024 / 1024  # convert bytes to MB
                new_pid_node_dict[node] = pid
                self.memory_usage_history[node][pid].append(mem_usage)

                # 记录当前内存使用量
                self.memory_usage_current[node] = mem_usage  # 添加此行

            except psutil.NoSuchProcess:
                logging.info("Process with PID: {} doesn't exist for node {}. Updating PID...".format(pid, node))
                self.is_updated.clear()  # 设置状态未更新
                log_file = self.get_latest_log_file()
                new_pid_node_dict = self.extract_pid_from_log(log_file, update=True)
                if node not in new_pid_node_dict:
                    logging.warning("Node {} is not working. Keeping its statistical data.".format(node))

        self.pid_node_dict = new_pid_node_dict  # 更新pid-node映射


    # 获取某个节点的内存使用情况
    def get_memory_usage(self, node):
        pid = self.pid_node_dict.get(node)
        if pid is None:
            return None
        try:
            process = psutil.Process(pid)
            mem_info = process.memory_info()
            return mem_info.rss / 1024 / 1024
        except psutil.NoSuchProcess:
            logging.warning(f"Process with PID: {pid} doesn't exist.")
            return None

    # 获取内存使用统计数据
    def get_memory_usage_stats(self, node):
        if node not in self.memory_usage_history:
            return None

        usage_history = [usage for pid_hist in self.memory_usage_history[node].values() for usage in pid_hist]
        avg = statistics.mean(usage_history)
        stdev = statistics.stdev(usage_history) if len(usage_history) > 1 else 0
        return {
            'average': avg,
            'stdev': stdev
        }

def main():
    log_directory = '/home/robotlab/.ros/log/'  # 替换为实际的日志目录
    node_monitor = NodeMonitor(log_directory, "WARNING")

    # 在新的线程中启动监控程序，以便主线程可以执行其他任务
    monitor_thread = threading.Thread(target=node_monitor.monitor)
    monitor_thread.start()

    # A dictionary to keep track of the average memory usage for each node
    average_memory_usage = {}

    # Keep track of the previous top 10 nodes
    previous_top_10_nodes = []

    while True:
        for node in node_monitor.pid_node_dict.keys():
            memory_usage = node_monitor.get_memory_usage(node)
            if memory_usage is not None:
                # Update the average memory usage
                if node in average_memory_usage:
                    average_memory_usage[node] = (average_memory_usage[node] + memory_usage) / 2
                else:
                    average_memory_usage[node] = memory_usage

        # Get the top 10 nodes by average memory usage
        top_10_nodes = sorted(average_memory_usage.items(), key=lambda x: x[1], reverse=True)[:10]

        print("Top 10 nodes by average memory usage:")
        for rank, (node, avg_usage) in enumerate(top_10_nodes, 1):
            print(f"Rank #{rank}: Node: {node}, Average Memory Usage: {avg_usage:.2f} MB")
            # Check if the node changed its position
            if previous_top_10_nodes and rank <= len(previous_top_10_nodes) and node != previous_top_10_nodes[rank - 1][0]:
                print(f"Position changed at rank #{rank}: {previous_top_10_nodes[rank - 1][0]} -> {node}")

        # Save the current top 10 nodes for the next iteration
        previous_top_10_nodes = top_10_nodes.copy()

        # Wait for 1 second before updating the information
        time.sleep(1)

if __name__ == "__main__":
    main()


