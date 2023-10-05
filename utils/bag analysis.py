import os
import json

# 讀取bag_info.txt文件內容
with open('/home/robotlab/autoware/bag/bag_info.txt', 'r') as f:
    lines = f.readlines()

# 解析每一行來獲取主題名稱和計數
topics_counts = {}

for line in lines:
    if "Topic:" in line:
        topic_name = line.split("|")[0].split(":")[1].strip()
        count = int(line.split("|")[2].split(":")[1].strip())
        topics_counts[topic_name] = count

# 從topics_counts中刪除/rosout主題和消息數為0的topics
topics_to_remove = [topic for topic, count in topics_counts.items() if count == 0 or topic == "/rosout"]
for topic in topics_to_remove:
    del topics_counts[topic]

# 從node_info文件夾解析每個node的信息
node_dir = 'node_info'
node_topic_map = {}

for filename in os.listdir(node_dir):
    if filename.endswith(".json"):
        with open(os.path.join(node_dir, filename), 'r') as f:
            node_data = json.load(f)
            node_name = node_data['Node']
            published_topics = node_data.get('publishers', [])
            node_topic_map[node_name] = published_topics

# 計算每個node的總消息數
node_msg_counts = {}

for node, topics in node_topic_map.items():
    total_count = 0
    for topic in topics:
        total_count += topics_counts.get(topic, 0)
    node_msg_counts[node] = total_count

# 對node按照總消息數進行排序
sorted_nodes = sorted(node_msg_counts.items(), key=lambda x: x[1], reverse=True)

# 輸出排序後的結果，只顯示消息數量不為0的nodes
for node, count in sorted_nodes:
    if count != 0:
        print(f"Node: {node}")
        print(f"Published Topics:")
        for topic in node_topic_map[node]:
            if topics_counts.get(topic, 0) != 0:
                print(f"    Topic: {topic} | Count: {topics_counts.get(topic, 0)}")
        print(f"Total Messages: {count}")
        print("-"*50)

# 找到並打印共用topics的nodes
topic_node_map = {}
for node, topics in node_topic_map.items():
    for topic in topics:
        if topics_counts.get(topic, 0) != 0:
            topic_node_map.setdefault(topic, []).append(node)

print("Nodes sharing topics:")
for topic, nodes in topic_node_map.items():
    if len(nodes) > 1:
        print(f"Topic: {topic} | Messages: {topics_counts.get(topic, 0)}")
        for node in nodes:
            print(f"    Node: {node}")
        print("-"*50)
