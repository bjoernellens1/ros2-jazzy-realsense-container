import sys
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from collections import defaultdict

def inspect_bag(bag_path):
    print(f"Opening bag {bag_path}...")
    reader = rosbag2_py.SequentialReader()
    
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id="mcap"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    
    reader.open(storage_options, converter_options)
    
    # Get topic types
    topic_types = {}
    for topic_metadata in reader.get_all_topics_and_types():
        topic_types[topic_metadata.name] = topic_metadata.type
        
    print("Topics and types found in metadata:")
    for name, type_ in sorted(topic_types.items()):
        print(f"  {name} [{type_}]")
        
    counts = defaultdict(int)
    timestamps = defaultdict(list)
    
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        counts[topic] += 1
        timestamps[topic].append(t)
        
    print("\nMessage statistics:")
    for topic in sorted(counts.keys()):
        t_list = timestamps[topic]
        t_min = min(t_list) / 1e9 if t_list else 0
        t_max = max(t_list) / 1e9 if t_list else 0
        dur = t_max - t_min
        print(f"  {topic}:")
        print(f"    Count: {counts[topic]}")
        print(f"    First stamp: {t_min:.3f} s")
        print(f"    Last stamp:  {t_max:.3f} s")
        print(f"    Duration:    {dur:.3f} s")
        if len(t_list) > 1:
            avg_diff = (t_list[-1] - t_list[0]) / (len(t_list) - 1) / 1e9
            print(f"    Avg rate:    {1.0/avg_diff:.2f} Hz (avg interval: {avg_diff:.4f} s)")
        if len(t_list) >= 5:
            first_5_diffs = [(t_list[i+1] - t_list[i])/1e9 for i in range(min(5, len(t_list)-1))]
            print(f"    First few intervals: {first_5_diffs}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: inspect_mcap_ros2.py <bag_path>")
    else:
        inspect_bag(sys.argv[1])
