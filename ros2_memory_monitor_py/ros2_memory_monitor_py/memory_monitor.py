import json
import os
import psutil
import re
import statistics
import threading
import time
import logging

class NodeMonitor:
    def __init__(self, log_directory, log_level):
        self.log_directory = log_directory
        self.pid_node_dict = {}
        self.node_status = {}
        self.memory_usage_history = {}
        self.memory_usage_current = {}
        self.lock = threading.Lock()
        self.log_filename = self.create_log_file()

        numeric_level = getattr(logging, log_level.upper(), None)
        if not isinstance(numeric_level, int):
            raise ValueError('Invalid log level: %s' % log_level)
        logging.basicConfig(level=numeric_level)
        self.update_stats()

    def create_log_file(self):
        log_folders = [f.path for f in os.scandir(self.log_directory) if f.is_dir()]
        latest_folder_name = os.path.basename(max(log_folders, key=os.path.getmtime))

        filename = latest_folder_name + '.json'  # 这将只使用时间戳作为文件名
        counter = 1
        while os.path.exists(filename):
            filename = f"{latest_folder_name}_{counter}.json"
            counter += 1

        print(f"Created JSON file: {filename}")
        return filename


    def update_stats(self):
        log_file = self.get_latest_log_file()
        with self.lock:
            self.pid_node_dict = self.extract_pid_from_log(log_file)
            for node in self.pid_node_dict:
                self.node_status[node] = {'update': False, 'down': False}

    def get_latest_log_file(self):
        log_folders = [f.path for f in os.scandir(self.log_directory) if f.is_dir()]
        latest_folder = max(log_folders, key=os.path.getmtime)
        log_file = os.path.join(latest_folder, 'launch.log')
        return log_file

    def extract_pid_from_log(self, log_file):
        pid_node_dict = {}
        with open(log_file, 'r') as file:
            lines = file.readlines()
            for line in lines:
                match = re.search(r'\[INFO\] \[(.+)\]: process started with pid \[(\d+)\]', line)
                if match:
                    node_name_with_number = match.group(1)
                    pid = int(match.group(2))
                    pid_node_dict[node_name_with_number] = pid
                    if node_name_with_number not in self.memory_usage_history:
                        self.memory_usage_history[node_name_with_number] = {}
                    if pid not in self.memory_usage_history[node_name_with_number]:
                        self.memory_usage_history[node_name_with_number][pid] = []
        return pid_node_dict

    def monitor(self, MAX_HISTORY_PER_NODE=10):
        new_pid_node_dict = {}
        timestamp = time.time_ns()
        for node, pid in self.pid_node_dict.items():
            try:
                # 确保 node 和 pid 的键存在
                if node not in self.memory_usage_history:
                    self.memory_usage_history[node] = {}
                if pid not in self.memory_usage_history[node]:
                    self.memory_usage_history[node][pid] = []
                process = psutil.Process(pid)
                mem_info = process.memory_info()
                mem_usage = mem_info.rss / 1024 / 1024
                new_pid_node_dict[node] = pid

                # 只添加一次，包含时间戳和内存使用量的元组
                self.memory_usage_history[node][pid].append((timestamp, mem_usage))

                self.memory_usage_current[node] = mem_usage

            except psutil.NoSuchProcess:
                logging.info(f"Process with PID: {pid} doesn't exist for node {node}. Updating PID...")
                log_file = self.get_latest_log_file()
                updated_pid_node_dict = self.extract_pid_from_log(log_file)
                if node in updated_pid_node_dict and updated_pid_node_dict[node] == pid:
                    self.node_status[node]['down'] = True
                else:
                    self.node_status[node]['update'] = True

        with self.lock:
            self.pid_node_dict = new_pid_node_dict

        # 检查是否需要保存历史记录
        if any(len(pid_hist) >= MAX_HISTORY_PER_NODE for node_hist in self.memory_usage_history.values() for pid_hist in
               node_hist.values()):
            self.save_history_usage()



    def save_history_usage(self):
        # 创建一个缓存列表
        data_list = []

        for node, pid_history in self.memory_usage_history.items():
            for pid, history in pid_history.items():
                for timestamp, mem_usage in history:
                    data = {
                        "timestamp": timestamp,
                        "node": node,
                        "pid": pid,
                        "memory_usage": mem_usage
                    }
                    data_list.append(data)

        # 如果数据量达到了一定的大小（例如10），则写入文件
        if len(data_list) >= 10:
            with open(self.log_filename, 'a') as file:
                for data in data_list:
                    json.dump(data, file)
                    file.write('\n')

            # 清空缓存列表
            data_list.clear()

        # 清空历史记录
        self.memory_usage_history.clear()

    def get_memory_usage(self, node):
        with self.lock:
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

    def get_memory_usage_history(self, node):
        if node in self.memory_usage_history:
            return self.memory_usage_history[node]
        return None

    # 获取内存使用统计数据
    def get_memory_usage_stats(self, node):
        usage_history = self.get_memory_usage_history(node)
        if usage_history is None:
            return None

        # 将历史记录展平，以便计算统计数据
        usage_history_flat = [mem_usage for pid_hist in usage_history.values() for timestamp, mem_usage in pid_hist]

        if not usage_history_flat:  # 判断列表是否为空
            return {
                'average': -1,
                'stdev': -1
            }

        avg = statistics.mean(usage_history_flat)
        stdev = statistics.stdev(usage_history_flat) if len(usage_history_flat) > 1 else 0
        return {
            'average': avg,
            'stdev': stdev
        }

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
    def manual_update_stats(self):
        log_file = self.get_latest_log_file()
        updated_pid_node_dict = self.extract_pid_from_log(log_file)

        with self.lock:  # 使用锁来保护共享数据的修改
            for node, new_pid in updated_pid_node_dict.items():
                old_pid = self.pid_node_dict.get(node)
                if old_pid != new_pid:
                    # 更新PID和状态
                    self.pid_node_dict[node] = new_pid
                    self.node_status[node]['update'] = True
                    self.node_status[node]['down'] = False
                    logging.info(f"Updated PID for node {node}: {old_pid} -> {new_pid}")
                else:
                    # PID相同，保持状态不变
                    logging.info(f"Node {node} is working with the same PID: {old_pid}")

# def main():
#     log_directory = '/home/robotlab/.ros/log/'  # 替换为实际的日志目录
#     node_monitor = NodeMonitor(log_directory, "WARNING")
#
#     # Keep track of the previous top 10 nodes by average memory usage
#     previous_top_10_nodes = []
#
#     while True:
#         node_monitor.monitor(MAX_HISTORY_PER_NODE=10)  # 直接在主线程中调用monitor
#
#         average_memory_usage = {}
#         for node in node_monitor.pid_node_dict.keys():
#             stats = node_monitor.get_memory_usage_stats(node)
#             if stats is not None:
#                 average_memory_usage[node] = stats['average']
#
#         # Get the top 10 nodes by average memory usage
#         top_10_nodes = sorted(average_memory_usage.items(), key=lambda x: x[1], reverse=True)[:10]
#
#         print("Top 10 nodes by average memory usage:")
#         for rank, (node, avg_usage) in enumerate(top_10_nodes, 1):
#             print(f"Rank #{rank}: Node: {node}, Average Memory Usage: {avg_usage:.2f} MB")
#             # Check if the node changed its position
#             if previous_top_10_nodes and rank <= len(previous_top_10_nodes) and node != previous_top_10_nodes[rank - 1][0]:
#                 print(f"Position changed at rank #{rank}: {previous_top_10_nodes[rank - 1][0]} -> {node}")
#
#         # Save the current top 10 nodes for the next iteration
#         previous_top_10_nodes = top_10_nodes.copy()
#
#         # Wait for 1 second before updating the information
#         time.sleep(0.01)
#
# if __name__ == "__main__":
#     main()


