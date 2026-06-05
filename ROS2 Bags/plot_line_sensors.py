#!/usr/bin/env python3
"""
Plot line sensor data from a ROS 2 bag (.db3).

Usage:
  python3 plot_line_sensors.py
  python3 plot_line_sensors.py --bag /path/to/line_sensors_bag_0.db3
  python3 plot_line_sensors.py --topic /line_sensors --window 10
"""

import argparse
import array
import sqlite3
import sys
from pathlib import Path
from typing import Iterable, Optional

import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


SENSOR_COLUMNS = ["sensor1", "sensor2", "sensor3", "sensor4", "sensor5"]


def moving_average(signal: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return signal.copy()
    kernel = np.ones(window, dtype=float) / float(window)
    valid = np.convolve(signal, kernel, mode="valid")
    pad = np.full(window - 1, np.nan, dtype=float)
    return np.concatenate([pad, valid])


def _iter_numeric_values(obj) -> Iterable[float]:
    """Recursively yield numeric leaves from ROS messages."""
    if isinstance(obj, (int, float)):
        yield float(obj)
        return

    if isinstance(obj, (list, tuple, np.ndarray, array.array)):
        for item in obj:
            yield from _iter_numeric_values(item)
        return

    slots = getattr(obj, "__slots__", None)
    if slots:
        for name in slots:
            yield from _iter_numeric_values(getattr(obj, name))


def _extract_sensor_values(msg) -> list[float]:
    """Return a flat numeric list for one ROS message."""
    # Common case for std_msgs/msg/Int32MultiArray and similar messages.
    if hasattr(msg, "data"):
        data = getattr(msg, "data")
        if isinstance(data, (list, tuple, np.ndarray, array.array)):
            return [float(v) for v in data]
    return list(_iter_numeric_values(msg))


def _pick_topic(conn: sqlite3.Connection, requested_topic: Optional[str]) -> tuple[str, str]:
    topic_rows = conn.execute("SELECT id, name, type FROM topics").fetchall()
    if not topic_rows:
        raise ValueError("Bag does not contain any topics.")

    if requested_topic:
        for _, name, msg_type in topic_rows:
            if name == requested_topic:
                return name, msg_type
        available = ", ".join(name for _, name, _ in topic_rows)
        raise ValueError(f"Topic '{requested_topic}' not found. Available topics: {available}")

    for _, name, msg_type in topic_rows:
        candidate = name.lower()
        if "line" in candidate and "sensor" in candidate:
            return name, msg_type

    _, name, msg_type = topic_rows[0]
    return name, msg_type


def inspect_bag(db3_path: Path, requested_topic: Optional[str], sample_count: int = 3) -> None:
    """Print human-readable ROS bag/topic/message structure information."""
    conn = sqlite3.connect(db3_path)
    try:
        topic_rows = conn.execute(
            """
            SELECT t.id, t.name, t.type, COUNT(m.id)
            FROM topics t
            LEFT JOIN messages m ON m.topic_id = t.id
            GROUP BY t.id, t.name, t.type
            ORDER BY t.name
            """
        ).fetchall()

        print(f"[inspect] Bag file: {db3_path}")
        print(f"[inspect] Topics found: {len(topic_rows)}")
        for _, name, msg_type, msg_count in topic_rows:
            print(f"[inspect] - {name} ({msg_type}) -> {msg_count} messages")

        topic_name, msg_type = _pick_topic(conn, requested_topic)
        msg_type_cls = get_message(msg_type)
        print(f"[inspect] Selected topic: {topic_name}")
        print(f"[inspect] Selected type : {msg_type}")

        rows = conn.execute(
            """
            SELECT m.timestamp, m.data
            FROM messages m
            JOIN topics t ON m.topic_id = t.id
            WHERE t.name = ?
            ORDER BY m.timestamp ASC
            LIMIT ?
            """,
            (topic_name, sample_count),
        ).fetchall()
    finally:
        conn.close()

    if not rows:
        print(f"[inspect] No messages in selected topic: {topic_name}")
        return

    for idx, (ts, raw_data) in enumerate(rows):
        msg = deserialize_message(raw_data, msg_type_cls)
        values = list(_iter_numeric_values(msg))
        print(f"[inspect] Sample #{idx} timestamp(ns): {ts}")
        print(f"[inspect] Sample #{idx} message repr: {msg!r}")
        slots = getattr(msg, "__slots__", None)
        if slots:
            print(f"[inspect] Sample #{idx} top-level fields: {list(slots)}")
        print(f"[inspect] Sample #{idx} numeric leaves count: {len(values)}")
        print(f"[inspect] Sample #{idx} numeric leaves preview: {values[:12]}")


def parse_rosbag(db3_path: Path, requested_topic: Optional[str]):
    conn = sqlite3.connect(db3_path)
    try:
        topic_name, msg_type = _pick_topic(conn, requested_topic)
        msg_type_cls = get_message(msg_type)

        rows = conn.execute(
            """
            SELECT m.timestamp, m.data
            FROM messages m
            JOIN topics t ON m.topic_id = t.id
            WHERE t.name = ?
            ORDER BY m.timestamp ASC
            """,
            (topic_name,),
        ).fetchall()
    finally:
        conn.close()

    if not rows:
        raise ValueError(f"No messages found in topic '{topic_name}'.")

    timestamps = []
    sensor_rows = []
    for ts, raw_data in rows:
        msg = deserialize_message(raw_data, msg_type_cls)
        values = _extract_sensor_values(msg)
        if len(values) < 5:
            continue
        sensor_rows.append(values[:5])
        timestamps.append(float(ts) / 1e9)  # ROS bag timestamp is in nanoseconds

    if not sensor_rows:
        raise ValueError(
            f"Topic '{topic_name}' was read, but no messages exposed at least 5 numeric sensor values."
        )

    return np.array(timestamps, dtype=float), np.array(sensor_rows, dtype=float), topic_name


def main():
    parser = argparse.ArgumentParser(description="Plot line sensor raw + moving average.")
    parser.add_argument(
        "--bag",
        default="line_sensors_bag/line_sensors_bag_0.db3",
        help="Path to ROS 2 bag database (.db3).",
    )
    parser.add_argument(
        "--topic",
        default=None,
        help="ROS topic to read. If omitted, auto-detects topic containing 'line' and 'sensor'.",
    )
    parser.add_argument(
        "--inspect",
        action="store_true",
        help="Print bag/topic/message debug info before plotting.",
    )
    parser.add_argument(
        "--inspect-only",
        action="store_true",
        help="Print bag/topic/message debug info and exit without plotting.",
    )
    parser.add_argument("--window", type=int, default=10, help="Moving-average window in samples.")
    args = parser.parse_args()

    bag_path = Path(args.bag).expanduser()
    if not bag_path.exists():
        raise FileNotFoundError(f"ROS bag file not found: {bag_path}")

    if args.inspect or args.inspect_only:
        inspect_bag(bag_path, args.topic)
        if args.inspect_only:
            return

    try:
        x_axis, sensor_data, topic_name = parse_rosbag(bag_path, args.topic)
    except ValueError as exc:
        print(f"[error] {exc}", file=sys.stderr)
        print("[hint] Run with --inspect-only to print the exact bag/message structure.", file=sys.stderr)
        raise
    if sensor_data.shape[1] < 5:
        raise ValueError("Need 5 sensor columns.")

    colors = ["#264653", "#2A9D8F", "#E9C46A", "#F4A261", "#E76F51"]  # left -> right gradient
    labels = ["sensor1 (left)", "sensor2", "sensor3", "sensor4", "sensor5 (right)"]

    fig, (ax_raw, ax_avg) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f"Line Sensors from {topic_name}: Raw and Moving Average", fontsize=14)

    for i in range(5):
        raw_series = sensor_data[:, i]
        avg_series = moving_average(raw_series, args.window)

        ax_raw.plot(x_axis, raw_series, color=colors[i], linewidth=1.4, label=labels[i])
        ax_avg.plot(
            x_axis,
            avg_series,
            color=colors[i],
            linewidth=1.8,
            label=f"{labels[i]} (MA{args.window})",
        )

    ax_raw.set_ylabel("Raw value")
    ax_raw.grid(alpha=0.25)
    ax_raw.legend(loc="upper right", ncol=2, fontsize=9)

    ax_avg.set_ylabel(f"Moving avg ({args.window})")
    ax_avg.set_xlabel("Timestamp (seconds)")
    ax_avg.grid(alpha=0.25)
    ax_avg.legend(loc="upper right", ncol=2, fontsize=9)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
