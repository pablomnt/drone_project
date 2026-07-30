#!/usr/bin/env python3
"""Dump /rosout from a rosbag2 bag as plain text, without playing it."""
import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
topic = sys.argv[2] if len(sys.argv) > 2 else "/rosout"

reader = SequentialReader()
reader.open(StorageOptions(uri=bag), ConverterOptions("", ""))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
if topic not in types:
    print(f"topic {topic} not in bag; have: {sorted(types)}")
    sys.exit(1)
msg_type = get_message(types[topic])

LEVELS = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
n = 0
while reader.has_next():
    tname, data, t = reader.read_next()
    if tname != topic:
        continue
    m = deserialize_message(data, msg_type)
    stamp = m.stamp.sec + m.stamp.nanosec * 1e-9
    print(f"[{stamp:.3f}] [{LEVELS.get(m.level, m.level)}] [{m.name}] {m.msg}")
    n += 1
print(f"--- {n} messages on {topic} ---", file=sys.stderr)
