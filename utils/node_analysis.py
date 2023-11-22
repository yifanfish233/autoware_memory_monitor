import os
import json
from collections import defaultdict

import yaml


def load_node_info_from_directory(directory):
    node_info = {}
    for filename in os.listdir(directory):
        filepath = os.path.join(directory, filename)
        with open(filepath, "r") as f:
            info = json.load(f)
            node_name = info["Node"]
            node_info[node_name] = {
                "publishers": set(info["publishers"].keys()),
                "subscribers": set(info["subscribers"].keys())
            }
    return node_info

def get_active_topics(yaml_file):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)

    active_topics = set()
    for topic_info in data['rosbag2_bagfile_information']['topics_with_message_count']:
        if topic_info['message_count'] > 0:
            active_topics.add(topic_info['topic_metadata']['name'])

    return active_topics

def build_graph(node_info, active_topics):
    graph = defaultdict(set)
    for node, info in node_info.items():
        for published_topic in info["publishers"]:
            if published_topic in active_topics:
                for other_node, other_info in node_info.items():
                    if published_topic in other_info["subscribers"]:
                        graph[node].add(other_node)
    return graph

def dfs_build_chains(node, graph, visited=None, current_path=None, max_depth=200):
    if visited is None:
        visited = set()
    if current_path is None:
        current_path = []

    if node in visited:
        return None

    if len(current_path) > max_depth:
        print("Max depth reached!")
        return None

    current_path.append(node)
    visited.add(node)

    chains = []
    for neighbor in graph[node]:
        if neighbor not in current_path:  # 避免环路
            result = dfs_build_chains(neighbor, graph, visited, current_path.copy(), max_depth)
            if result is not None:
                # 确保result是列表形式
                if isinstance(result[0], list):
                    chains.extend(result)
                else:
                    chains.append(result)

    current_path.pop()

    if not chains:
        return [[node]]
    else:
        # 为每个子链添加当前节点
        return [[node] + chain for chain in chains]




def get_all_chains(start_nodes, graph):
    all_chains = []
    for start_node in start_nodes:
        chains = dfs_build_chains(start_node, graph)
        if chains:
            print(f"Found {len(chains)} chains starting from {start_node}")
            all_chains.append(chains)
    return all_chains


def identify_start_nodes(node_info, graph):
    incoming_edges = {node: 0 for node in node_info}
    for node, neighbors in graph.items():
        for neighbor in neighbors:
            incoming_edges[neighbor] += 1
    return [node for node, count in incoming_edges.items() if count == 0]

if __name__ == "__main__":
    directory = "node_info"
    # node_info = load_node_info_from_directory(directory)
    # graph = build_graph(node_info)
    active_topics = get_active_topics('/home/robotlab/autoware/bag/rosbag2_2023_11_17-00_12_48/metadata.yaml')
    node_info = load_node_info_from_directory(directory)
    graph = build_graph(node_info, active_topics)

    start_nodes = identify_start_nodes(node_info, graph)

    all_chains = get_all_chains(start_nodes, graph)

    # 打印所有链
    print("\nAll Chains:")
    for chain in all_chains:
        print("-"*50)
        print(chain)