import subprocess
import os
import json

def get_node_names():
    result = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True)
    nodes = result.stdout.splitlines()
    return nodes

def get_node_info(node_name):
    result = subprocess.run(['ros2', 'node', 'info', node_name], stdout=subprocess.PIPE)
    lines = result.stdout.decode('utf-8').splitlines()

    publishers = {}
    subscribers = {}
    collecting_publishers = False
    collecting_subscribers = False

    for line in lines:
        stripped_line = line.strip()

        if "Publishers:" in stripped_line:
            collecting_publishers = True
            collecting_subscribers = False
            continue
        elif "Subscribers:" in stripped_line:
            collecting_publishers = False
            collecting_subscribers = True
            continue
        elif not stripped_line:  # Empty line indicates section end
            collecting_publishers = False
            collecting_subscribers = False
            continue

        # Extract topic and msg_type only if line contains ": "
        if ": " in stripped_line:
            topic, msg_type = stripped_line.split(": ")
            if collecting_publishers:
                publishers[topic] = msg_type
            elif collecting_subscribers:
                subscribers[topic] = msg_type

    return {
        "Node": node_name,
        "publishers": publishers,
        "subscribers": subscribers
    }

if __name__ == "__main__":
    nodes = get_node_names()

    # Create a directory in the current folder to store JSON files
    directory = "node_info"
    os.makedirs(directory, exist_ok=True)

    for node in nodes:
        info = get_node_info(node)
        filename = os.path.join(directory, node.replace("/", "_").lstrip("_") + ".json")
        with open(filename, "w") as f:
            json.dump(info, f, indent=4)

    print(f"JSON files saved in {directory} directory.")
