#!/usr/bin/env python3
import os
import sys
import glob
import sqlite3
import argparse
from typing import List, Tuple

def parse_args():
    parser = argparse.ArgumentParser(
        description="Fast ROS 2 Rosbag IMU Timestamp Jump & Data Gap Detector",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "bag_path",
        type=str,
        help="Path to ROS 2 bag directory or .db3 file"
    )
    parser.add_argument(
        "-t", "--topic",
        type=str,
        default="/imu/data_raw",
        help="IMU topic name to check"
    )
    parser.add_argument(
        "-g", "--gap-threshold",
        type=float,
        default=0.05,
        help="Data loss threshold in seconds (e.g. 0.05s = 50ms gap)"
    )
    parser.add_argument(
        "--use-bag-time",
        action="store_true",
        help="Use rosbag message recording timestamp instead of IMU header.stamp"
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Print details of every anomaly"
    )
    return parser.parse_args()


def find_db3_files(bag_path: str) -> List[str]:
    if os.path.isfile(bag_path) and bag_path.endswith(".db3"):
        return [bag_path]
    elif os.path.isdir(bag_path):
        db3_files = sorted(glob.glob(os.path.join(bag_path, "*.db3")))
        if not db3_files:
            # Recursive check if nested
            db3_files = sorted(glob.glob(os.path.join(bag_path, "**", "*.db3"), recursive=True))
        return db3_files
    return []


def check_imu_bag(bag_path: str, topic_name: str, gap_threshold: float, use_bag_time: bool, verbose: bool):
    db3_files = find_db3_files(bag_path)
    if not db3_files:
        print(f"\033[91m[ERROR] No .db3 bag files found in '{bag_path}'\033[0m")
        sys.exit(1)

    print(f"\033[96m============================================================\033[0m")
    print(f"\033[1m ROS 2 Bag IMU Quality Detector\033[0m")
    print(f"\033[96m============================================================\033[0m")
    print(f" Bag Path      : {bag_path}")
    print(f" Found DB files: {len(db3_files)} file(s)")
    print(f" Target Topic  : {topic_name}")
    print(f" Gap Threshold : {gap_threshold:.4f} s ({gap_threshold*1000:.1f} ms)")
    print(f" Timestamp Src : {'Bag Record Time' if use_bag_time else 'IMU Header Stamp'}")
    print(f"------------------------------------------------------------")

    # Try importing rclpy for ROS msg deserialization if using header stamp
    use_deserialization = not use_bag_time
    deserialize_msg = None
    Imu_class = None

    if use_deserialization:
        try:
            from rclpy.serialization import deserialize_message
            from sensor_msgs.msg import Imu
            deserialize_msg = deserialize_message
            Imu_class = Imu
        except ImportError:
            print("\033[93m[WARN] rclpy / sensor_msgs not found in Python path. Falling back to Bag Record Time!\033[0m")
            print("\033[93m       (Tip: run 'source /opt/ros/humble/setup.bash' first)\033[0m")
            use_deserialization = False

    timestamps: List[float] = []

    for db_path in db3_files:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()

        # Find topic_id
        cursor.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
        row = cursor.fetchone()
        if not row:
            # Check all available topics for user help
            cursor.execute("SELECT name FROM topics")
            all_topics = [r[0] for r in cursor.fetchall()]
            print(f"\033[91m[ERROR] Topic '{topic_name}' not found in {os.path.basename(db_path)}\033[0m")
            print(f"Available topics: {', '.join(all_topics)}")
            conn.close()
            continue

        topic_id = row[0]

        # Fetch messages
        cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (topic_id,))
        messages = cursor.fetchall()
        conn.close()

        for bag_ts, data_blob in messages:
            if use_deserialization:
                try:
                    imu_msg = deserialize_msg(data_blob, Imu_class)
                    t = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
                except Exception as e:
                    t = bag_ts * 1e-9
            else:
                t = bag_ts * 1e-9

            timestamps.append(t)

    total_msgs = len(timestamps)
    if total_msgs == 0:
        print(f"\033[91m[ERROR] 0 messages retrieved for topic '{topic_name}'!\033[0m")
        sys.exit(1)

    print(f" Total IMU Msgs: {total_msgs}")
    duration = timestamps[-1] - timestamps[0] if total_msgs > 1 else 0.0
    avg_hz = (total_msgs - 1) / duration if duration > 0 else 0.0
    print(f" Time Span     : {duration:.3f} s ({timestamps[0]:.3f} -> {timestamps[-1]:.3f})")
    print(f" Avg Frequency : {avg_hz:.2f} Hz")
    print(f"------------------------------------------------------------")

    # Analyze intervals
    time_jumps: List[Tuple[int, float, float, float]] = [] # (index, t_prev, t_curr, delta)
    data_gaps: List[Tuple[int, float, float, float]] = []  # (index, t_prev, t_curr, delta)
    deltas: List[float] = []

    for i in range(1, total_msgs):
        t_prev = timestamps[i-1]
        t_curr = timestamps[i]
        dt = t_curr - t_prev
        deltas.append(dt)

        if dt <= 0:
            time_jumps.append((i, t_prev, t_curr, dt))
        elif dt > gap_threshold:
            data_gaps.append((i, t_prev, t_curr, dt))

    min_dt = min(deltas) if deltas else 0.0
    max_dt = max(deltas) if deltas else 0.0
    sorted_dt = sorted(deltas) if deltas else [0.0]
    median_dt = sorted_dt[len(sorted_dt) // 2]

    print(f" Min Interval  : {min_dt*1000:.2f} ms")
    print(f" Median Interv : {median_dt*1000:.2f} ms ({1.0/median_dt:.1f} Hz)")
    print(f" Max Interval  : {max_dt*1000:.2f} ms ({1.0/max_dt if max_dt>0 else 0:.1f} Hz)")
    print(f"============================================================")

    # Anomaly Reports
    has_issues = len(time_jumps) > 0 or len(data_gaps) > 0

    if len(time_jumps) > 0:
        print(f"\033[91m[FAIL] Non-monotonic / Backward Timestamps Detected: {len(time_jumps)} event(s)\033[0m")
        for idx, t_prev, t_curr, dt in time_jumps:
            print(f"  - [# {idx:6d}] Prev: {t_prev:.6f}s | Curr: {t_curr:.6f}s | Jump: {dt*1000:+.2f} ms")
    else:
        print(f"\033[92m[ PASS ] Timestamp Monotonicity: OK (No backward time jumps)\033[0m")

    if len(data_gaps) > 0:
        print(f"\033[93m[WARN] IMU Data Drops (> {gap_threshold:.3f}s / {gap_threshold*1000:.0f}ms): {len(data_gaps)} gap(s)\033[0m")
        for idx, t_prev, t_curr, dt in data_gaps:
            print(f"  - [# {idx:6d}] Gap Start: {t_prev:.4f}s | Gap End: {t_curr:.4f}s | Duration: \033[91m{dt:.4f} s ({dt*1000:.1f} ms)\033[0m")
    else:
        print(f"\033[92m[ PASS ] Data Continuity: OK (No data gap > {gap_threshold:.3f}s)\033[0m")

    print(f"============================================================")
    if not has_issues:
        print(f"\033[92m\033[1m[SUCCESS] Rosbag IMU data is clean and continuous!\033[0m")
    else:
        print(f"\033[91m\033[1m[ATTENTION] Found {len(time_jumps)} time jumps and {len(data_gaps)} data gaps.\033[0m")
    print(f"============================================================\n")

if __name__ == "__main__":
    args = parse_args()
    check_imu_bag(args.bag_path, args.topic, args.gap_threshold, args.use_bag_time, args.verbose)
