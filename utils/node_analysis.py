import os
import json
from collections import defaultdict





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

def build_graph(node_info):
    graph = defaultdict(list)
    for node, info in node_info.items():
        for published_topic in info["publishers"]:
            for other_node, other_info in node_info.items():
                if published_topic in other_info["subscribers"]:
                    graph[node].append(other_node)
    return graph

def dfs(node, graph, visited):
    if node in visited:
        return [node]  # Return the cyclic node
    visited.add(node)
    downstream = [node]
    for neighbor in graph[node]:
        downstream.extend(dfs(neighbor, graph, visited))
    return downstream

def identify_start_nodes(node_info, graph):
    incoming_edges = {node: 0 for node in node_info}
    for node, neighbors in graph.items():
        for neighbor in neighbors:
            incoming_edges[neighbor] += 1
    return [node for node, count in incoming_edges.items() if count == 0]

if __name__ == "__main__":
    directory = "node_info"
    node_info = load_node_info_from_directory(directory)
    graph = build_graph(node_info)
    start_nodes = identify_start_nodes(node_info, graph)

    all_paths = []
    for start in start_nodes:
        visited = set()
        path = dfs(start, graph, visited)
        if path:
            all_paths.append(path.copy())

    print("\nPaths:")
    for path in all_paths:
        print(" -> ".join(path))

    # Identify nodes at the same level
    levels = defaultdict(set)
    for path in all_paths:
        for index, node in enumerate(path):
            levels[index].add(node)

    print("\nNodes at the same level:")
    for level, nodes in levels.items():
        print(f"Level {level}: {' | '.join(nodes)}")

    # Print downstream nodes for each node
    print("\nDownstream nodes for each node:")
    for node in node_info:
        visited = set()
        downstream = dfs(node, graph, visited)
        downstream.remove(node)  # Exclude the node itself from the downstream list
        print(f"{node} has downstream nodes: {' -> '.join(downstream) if downstream else 'None'}")