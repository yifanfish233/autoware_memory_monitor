import os
import sqlite3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import yaml


def load_yaml_metadata(folder_path):
    """加载并返回YAML文件的内容。"""
    yaml_file_path = os.path.join(folder_path, 'metadata.yaml')
    with open(yaml_file_path, 'r') as file:
        metadata = yaml.safe_load(file)
    return metadata['rosbag2_bagfile_information']['topics_with_message_count']

def query_topic_data(db_file_path, topic_name):
    """从SQLite数据库中查询特定话题的时间戳数据。"""
    conn = sqlite3.connect(db_file_path)
    query = f"""
    SELECT m.timestamp 
    FROM messages m
    JOIN topics t ON m.topic_id = t.id
    WHERE t.name = '{topic_name}'
    """
    df = pd.read_sql_query(query, conn)
    conn.close()
    df['timestamp'] = pd.to_datetime(df['timestamp'], unit='ns')
    return df


def print_topics_and_select(metadata):
    """打印话题列表并允许用户选择多个话题，只包括消息数量不为零的话题。"""
    topics_with_messages = {topic['topic_metadata']['name']: topic['message_count']
                            for topic in metadata if topic['message_count'] > 0}

    # 按消息数量排序
    sorted_topics = sorted(topics_with_messages.items(), key=lambda x: x[1], reverse=True)

    print("Topics with messages:")
    for i, (topic_name, message_count) in enumerate(sorted_topics, start=1):
        print(f"{i}. Topic: {topic_name} - Message Count: {message_count}")

        # 打印消息数量为零的话题
        topics_without_messages = [topic['topic_metadata']['name']
                                   for topic in metadata if topic['message_count'] == 0]
    if topics_without_messages:
        print("\nTopics without messages:")
        for topic in topics_without_messages:
            print(f"- {topic}")

    choices = input("Enter the numbers of the topics to analyze, separated by spaces (at least three): ")
    selected_topics_indices = [int(choice.strip()) for choice in choices.split()]
    selected_topics = [sorted_topics[index - 1][0] for index in selected_topics_indices if
                       1 <= index <= len(sorted_topics)]



    return selected_topics


def visualize_sequence_analysis(correct_count, error_count):
    """用饼图可视化正确和错误顺序的百分比。"""
    labels = 'Correct Sequence', 'Incorrect Sequence'
    sizes = [correct_count, error_count]
    colors = ['green', 'red']

    plt.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', startangle=140)
    plt.axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.
    plt.title('Message Sequence Analysis')
    plt.show()


# def visualize_disorder_messages(all_timestamps, sequence_errors, topics):
#     topic_order = {topic: i for i, topic in enumerate(topics)}
#     all_timestamps['topic_order'] = all_timestamps['topic'].map(topic_order)
#
#     plt.figure(figsize=(15, 6))  # 根据需要调整图表尺寸
#     ax = plt.gca()  # 获取当前的Axes对象
#
#     # 绘制所有消息
#     for topic in topics:
#         topic_data = all_timestamps[all_timestamps['topic'] == topic]
#         plt.scatter(topic_data['timestamp'], topic_data['topic_order'], alpha=0.7, label=topic, s=10)
#
#     # 特别标注顺序错误的消息
#     if sequence_errors:
#         error_data = all_timestamps.iloc[sequence_errors]
#         plt.scatter(error_data['timestamp'], error_data['topic_order'], color='red', label='Disorder Messages', alpha=0.7, s=10)
#
#     # 设置y轴的范围，使得最高点以上有空间放置图例
#     plt.ylim(-1, len(topics) + 1)  # +1 为了在顶部留出空间
#
#     plt.xlabel('Timestamp')
#     plt.ylabel('Topic Order')
#     plt.yticks(range(len(topics)), topics)
#     plt.title('Message Sequence Analysis with Disorder Highlighted')
#
#     # 调整子图的布局参数
#     plt.subplots_adjust(top=0.85)  # 调整顶部边距以给图例留出空间
#
#     # 将图例移至图表的上方
#     ax.legend(loc='lower center', bbox_to_anchor=(0.5, 1.05), ncol=3, fontsize='small', markerscale=0.7)
#
#     plt.show()



# 接下来，您可以调用 visualize_disorder_messages 函数，就像之前示例中展示的那样。


def analyze_message_sequence(db_file_path, topics, type_of_matters):
    """分析消息是否遵循预期的链路结构，并可视化结果。"""
    dfs = []
    sequence_errors = []

    if type_of_matters == "sequence":
        # 加载每个话题的时间戳
        for topic in topics:
            df = query_topic_data(db_file_path, topic)
            df['topic'] = topic
            dfs.append(df)

        # 合并并按时间排序
        all_timestamps = pd.concat(dfs).sort_values(by='timestamp')

        # 检查是否符合预期的链路结构
        for i in range(len(topics) - 1):
            current_topic = topics[i]
            next_topic = topics[i + 1]

            current_topic_msgs = all_timestamps[all_timestamps['topic'] == current_topic]
            for index, row in current_topic_msgs.iterrows():
                locs = all_timestamps.index.get_loc(index)
                if isinstance(locs, np.ndarray):
                    next_index = locs[0] + 1
                else:
                    next_index = locs + 1

                if next_index < len(all_timestamps):
                    next_msg = all_timestamps.iloc[next_index]
                    if next_msg['topic'] != next_topic:
                        sequence_errors.append(next_index)

    # 可视化结果
    # visualize_disorder_messages(all_timestamps, sequence_errors, topics)
    visualize_sequence_analysis(len(all_timestamps) - len(sequence_errors), len(sequence_errors))
    return len(sequence_errors)





def calculate_frequency(df):
    """计算并返回消息的频率。"""
    if len(df) > 1:
        total_time = (df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]).total_seconds()
        frequency = len(df) / total_time if total_time > 0 else 0
        return frequency
    else:
        return 0

def get_frequency(db_file_path,metadata):
    for topic_info in metadata:
        topic_name = topic_info['topic_metadata']['name']
        df = query_topic_data(db_file_path, topic_name)
        frequency = calculate_frequency(df)
        print(f"Frequency of {topic_name}: {frequency:.2f} Hz")

def main(folder_path):
    metadata = load_yaml_metadata(folder_path)
    db_file_path = os.path.join(folder_path, next(f for f in os.listdir(folder_path) if f.endswith('.db3')))
    # get_frequency(db_file_path,metadata)
    selected_topics = print_topics_and_select(metadata)
    # 使用示例
    type_of_matters = "sequence"
    disorder_count = analyze_message_sequence(db_file_path, selected_topics, type_of_matters)
    print(f"Sequence errors: {disorder_count}")

if __name__ == "__main__":
    folder_path = '/home/robotlab/autoware/bag/rosbag2_2023_11_17-00_12_48'  # 替换为实际文件夹路径
    main(folder_path)
