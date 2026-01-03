from pathlib import Path
from rosbags.highlevel import AnyReader
import argparse

def list_topics(bag_path):
    print(f"Inspecting: {bag_path}")
    with AnyReader([Path(bag_path)]) as reader:
        print("\nTopics found:")
        for connection in reader.connections:
            print(f" - Topic: {connection.topic:<50} Type: {connection.msgtype}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path")
    args = parser.parse_args()
    list_topics(args.bag_path)
