import json
import os
import psutil
import re
import statistics
import threading
import time
import logging
import rclpy
from rclpy.node import Node
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
        rclpy.init(args=None)
        self.ros2_node = Node('node_monitor')


    def __del__(self):
        self.ros2_node.destroy_node()
        rclpy.shutdown()

    def get_all_active_nodes(self):
        node_info_list = self.ros2_node.get_node_names_and_namespaces()
        full_names = [f'{namespace}/{name}' if namespace != '/' else f'/{name}' for name, namespace in node_info_list]
        return full_names

    def update_pid_node_dict(self):
        all_nodes = self.get_all_active_nodes()
        new_pid_node_dict = {}

        for node in all_nodes:
            # 从节点名中提取模块名，如果需要的话
            module_name = self.get_module_name(node)

            # 使用psutil来查找与每个节点名匹配的进程的PID
            for proc in psutil.process_iter(attrs=['pid', 'cmdline']):
                try:
                    if any(node in cmd for cmd in proc.info['cmdline']):
                        pid = proc.info['pid']
                        if module_name not in new_pid_node_dict:
                            new_pid_node_dict[module_name] = {}
                        new_pid_node_dict[module_name][node] = pid
                        break
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    continue

        # 更新类的pid_node_dict属性
        with self.lock:
            self.pid_node_dict = new_pid_node_dict

    def update_pid_for_node(self, target_node):
        """为特定的节点更新PID"""
        # 使用psutil来查找与特定节点名匹配的进程的PID
        for proc in psutil.process_iter(attrs=['pid', 'cmdline']):
            try:
                if any(target_node in cmd for cmd in proc.info['cmdline']):
                    pid = proc.info['pid']
                    module_name = self.get_module_name(target_node)

                    # 更新pid_node_dict内的PID
                    with self.lock:
                        if module_name not in self.pid_node_dict:
                            self.pid_node_dict[module_name] = {}
                        self.pid_node_dict[module_name][target_node] = pid
                    break
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue

    def _get_process_memory(self, pid):
        """
        返回给定PID的进程的内存使用量。

        参数:
        - pid (int): 进程的PID。

        返回:
        - float: 进程的内存使用量（单位为MB）。
        """
        try:
            process = psutil.Process(pid)
            mem_info = process.memory_info()
            return mem_info.rss / 1024 / 1024
        except psutil.NoSuchProcess:
            return None

    def _get_process_memory_with_timestamp(self, pid):
        """
        返回给定PID的进程的内存使用量和当前的时间戳。

        参数:
        - pid (int): 进程的PID。

        返回:
        - tuple: (时间戳, 内存使用量（单位为MB）) 或 (None, None)。
        """
        try:
            process = psutil.Process(pid)
            mem_info = process.memory_info()
            mem_usage = mem_info.rss / 1024 / 1024
            timestamp = time.time_ns()
            return timestamp, mem_usage
        except psutil.NoSuchProcess:
            return None, None

    def get_memory_usage(self, node):
        with self.lock:
            pid_info = self.pid_node_dict.get(node)
            if pid_info is None:
                return 0
            # 由于pid_info可能是字典，我们需要确保获取到实际的pid
            pid = pid_info if isinstance(pid_info, int) else pid_info.get(node)
            if pid is None:
                return 0

            mem_usage = self._get_process_memory(pid)
            if mem_usage is None:
                logging.warning(f"Process with PID: {pid} doesn't exist for node {node}. Updating PID and stats...")
                # 为了避免mem_usage为None，我们可以重新获取内存使用量
                self.update_pid_for_node(node)
                new_pid = self.pid_node_dict.get(node, {}).get(node)
                mem_usage = self._get_process_memory(new_pid) if new_pid else 0
            return mem_usage


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

    def monitor(self, MAX_HISTORY_PER_NODE=10):
        for module_name, node_dict in self.pid_node_dict.items():
            for node, pid in node_dict.items():
                try:
                    # 确保node的键存在
                    if node not in self.memory_usage_history:
                        self.memory_usage_history[node] = {}
                    # 确保pid的键存在
                    if pid not in self.memory_usage_history[node]:
                        self.memory_usage_history[node][pid] = []

                    timestamp, mem_usage = self._get_process_memory_with_timestamp(pid)

                    if timestamp is not None and mem_usage is not None:
                        # 只添加一次，包含时间戳和内存使用量的元组
                        self.memory_usage_history[node][pid].append((timestamp, mem_usage))
                        self.memory_usage_current[node] = mem_usage
                    else:
                        raise psutil.NoSuchProcess

                except psutil.NoSuchProcess:
                    logging.info(f"Process with PID: {pid} doesn't exist for node {node}. Updating PID...")
                    self.update_pid_for_node(node)  # 只更新该特定节点的PID

                    updated_pid = self.pid_node_dict.get(module_name, {}).get(node)
                    if updated_pid and updated_pid == pid:
                        self.node_status[node]['down'] = True
                    else:
                        self.node_status[node]['update'] = True

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
        pass

    def get_total_nodes(self):
        return len(self.pid_node_dict.keys())

    def get_total_memory_usage(self):
        total_memory = 0
        for node in self.pid_node_dict:
            memory = self.get_memory_usage(node)
            if memory is not None:
                total_memory += memory
        return round(total_memory, 2)

    def get_module_name(self, node_name):
        """
        从完整的node名称中提取module名。

        参数：
        - node_name (str): 完整的node名称，如 "/module_name/node_name"

        返回：
        - str: module名，如 "/module_name"
        """
        return node_name.split("/")[1]

    def get_modules(self):
        with self.lock:  # 使用锁来保护共享数据的读取
            # 从每个node名称中提取module名称，然后用集合来去重
            module_set = {self.get_module_name(node) for node in self.pid_node_dict.keys()}
            return list(module_set)

    def get_nodes_from_module(self, module_name):
        """
        返回指定module下的所有nodes。

        参数：
        - module_name (str): 模块名称，如 "/module_name"

        返回：
        - list[str]: 指定module下的所有nodes。
        """
        nodes_in_module = []
        for node in self.pid_node_dict.keys():
            if self.get_module_name(node) == module_name:
                nodes_in_module.append(node)
        return nodes_in_module

    def get_memory_usage_stats_for_module(self, module_name):
        """
        返回指定module下的所有nodes的内存使用统计数据。

        参数：
        - module_name (str): 模块名称，如 "/module_name"

        返回：
        - dict: 包含每个node的内存使用统计数据的字典。
        """
        nodes_in_module = self.get_nodes_from_module(module_name)
        memory_stats = {}
        for node in nodes_in_module:
            memory_stats[node] = self.get_memory_usage_stats(node)
        return memory_stats


if __name__ == "__main__":
    monitor_obj = NodeMonitor("/home/robotlab/.ros/log/", "INFO")  # 请将路径替换为实际的路径

    # 每隔5秒运行monitor方法并打印当前的内存使用量
    while True:
        monitor_obj.monitor()
        print(f"Total Memory Usage: {monitor_obj.get_total_memory_usage()} MB")
        time.sleep(5)


