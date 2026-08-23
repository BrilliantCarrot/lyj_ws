#!/usr/bin/env python3
"""Analyze GPS-denied VIO PX4 rosbag results.

The script expects a rosbag2 sqlite directory containing at least /nav/odom,
/vio_aligned/odom, /guidance/setpoint, /fmu/in/vehicle_visual_odometry, and
/fmu/out/estimator_status_flags.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path
from typing import Callable

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


Vector = tuple[float, float, float]


def position(msg) -> Vector:
    p = msg.pose.pose.position
    return (float(p.x), float(p.y), float(p.z))


def velocity(msg) -> Vector:
    v = msg.twist.twist.linear
    return (float(v.x), float(v.y), float(v.z))


def norm(v: Vector) -> float:
    return math.sqrt(sum(x * x for x in v))


def sub(a: Vector, b: Vector) -> Vector:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def interp(samples, times, t: float, getter: Callable) -> Vector:
    i = bisect.bisect_left(times, t)
    if i <= 0:
        return getter(samples[0][1])
    if i >= len(samples):
        return getter(samples[-1][1])
    t0, m0 = samples[i - 1]
    t1, m1 = samples[i]
    ratio = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    v0 = getter(m0)
    v1 = getter(m1)
    return tuple(v0[j] * (1.0 - ratio) + v1[j] * ratio for j in range(3))


def load_bag(uri: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {name: get_message(type_name) for name, type_name in topic_types.items()}
    data = {name: [] for name in topic_types}
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        data[topic].append((stamp_ns * 1e-9, deserialize_message(raw, msg_types[topic])))
    return data


def topic_rate(samples) -> tuple[int, float, float]:
    if not samples:
        return 0, 0.0, 0.0
    duration = samples[-1][0] - samples[0][0]
    rate = (len(samples) - 1) / duration if duration > 0.0 and len(samples) > 1 else 0.0
    return len(samples), duration, rate


def relative_error(a_samples, b_samples, getter: Callable) -> dict:
    if not a_samples or not b_samples:
        return {}
    t0 = max(a_samples[0][0], b_samples[0][0])
    t1 = min(a_samples[-1][0], b_samples[-1][0])
    a_overlap = [(t, m) for t, m in a_samples if t0 <= t <= t1]
    if not a_overlap:
        return {}

    b_times = [t for t, _ in b_samples]
    a0 = getter(a_overlap[0][1])
    b0 = interp(b_samples, b_times, a_overlap[0][0], getter)
    sum_sq = [0.0, 0.0, 0.0]
    sum_norm_sq = 0.0
    max_norm = 0.0
    final_err = (0.0, 0.0, 0.0)

    for t, msg in a_overlap:
        av = sub(getter(msg), a0)
        bv = sub(interp(b_samples, b_times, t, getter), b0)
        err = sub(bv, av)
        final_err = err
        for i in range(3):
            sum_sq[i] += err[i] * err[i]
        err_norm = norm(err)
        sum_norm_sq += err_norm * err_norm
        max_norm = max(max_norm, err_norm)

    n = len(a_overlap)
    return {
        "samples": n,
        "rmse": math.sqrt(sum_norm_sq / n),
        "axis_rmse": tuple(math.sqrt(x / n) for x in sum_sq),
        "max": max_norm,
        "final": norm(final_err),
        "final_axis": final_err,
    }


def absolute_motion(samples, getter: Callable) -> dict:
    if len(samples) < 2:
        return {}
    p0 = getter(samples[0][1])
    p1 = getter(samples[-1][1])
    final_delta = sub(p1, p0)
    max_radius = 0.0
    path_length = 0.0
    prev = p0
    for _, msg in samples:
        p = getter(msg)
        max_radius = max(max_radius, norm(sub(p, p0)))
        path_length += norm(sub(p, prev))
        prev = p
    return {
        "start": p0,
        "end": p1,
        "return_error": norm(final_delta),
        "return_axis": final_delta,
        "max_radius": max_radius,
        "path_length": path_length,
    }


def flag_summary(samples) -> dict:
    if not samples:
        return {}
    keys = [
        "cs_gnss_pos",
        "cs_gnss_vel",
        "cs_gps_hgt",
        "cs_ev_pos",
        "cs_ev_vel",
        "cs_ev_hgt",
        "cs_baro_hgt",
        "reject_hor_pos",
        "reject_ver_pos",
        "reject_hor_vel",
        "reject_ver_vel",
    ]
    result = {}
    for key in keys:
        vals = [bool(getattr(msg, key)) for _, msg in samples]
        result[key] = {
            "true_count": sum(vals),
            "total": len(vals),
            "first": vals[0],
            "last": vals[-1],
        }
    return result


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--csv", type=Path, default=None)
    args = parser.parse_args()

    data = load_bag(args.bag)

    print(f"bag: {args.bag}")
    print("topic rates:")
    for topic in sorted(data):
      n, duration, rate = topic_rate(data[topic])
      print(f"  {topic}: n={n} duration={duration:.2f}s rate={rate:.2f}Hz")

    nav = data.get("/nav/odom", [])
    vio = data.get("/vio_aligned/odom", [])
    sp = data.get("/guidance/setpoint", [])

    metrics = {
        "nav_return": absolute_motion(nav, position),
        "vio_return": absolute_motion(vio, position),
        "nav_vs_vio_position": relative_error(nav, vio, position),
        "nav_vs_vio_velocity": relative_error(nav, vio, velocity),
        "nav_tracking_setpoint": relative_error(nav, sp, position),
        "flags": flag_summary(data.get("/fmu/out/estimator_status_flags", [])),
    }

    print("\nreturn/drift:")
    for name in ["nav_return", "vio_return"]:
        m = metrics[name]
        if not m:
            continue
        print(
            f"  {name}: return_error={m['return_error']:.3f}m "
            f"axis=({m['return_axis'][0]:.3f}, {m['return_axis'][1]:.3f}, {m['return_axis'][2]:.3f}) "
            f"max_radius={m['max_radius']:.3f}m path_length={m['path_length']:.3f}m"
        )

    print("\nrelative RMSE:")
    for name in ["nav_vs_vio_position", "nav_vs_vio_velocity", "nav_tracking_setpoint"]:
        m = metrics[name]
        if not m:
            continue
        axis = m["axis_rmse"]
        final_axis = m["final_axis"]
        print(
            f"  {name}: rmse={m['rmse']:.3f} max={m['max']:.3f} final={m['final']:.3f} "
            f"axis_rmse=({axis[0]:.3f}, {axis[1]:.3f}, {axis[2]:.3f}) "
            f"final_axis=({final_axis[0]:.3f}, {final_axis[1]:.3f}, {final_axis[2]:.3f})"
        )

    print("\nflags:")
    for key, value in metrics["flags"].items():
        print(
            f"  {key}: {value['true_count']}/{value['total']} "
            f"first={value['first']} last={value['last']}"
        )

    if args.csv:
        rows = []
        for name in ["nav_return", "vio_return"]:
            m = metrics[name]
            if m:
                rows.append({
                    "metric": name,
                    "value": m["return_error"],
                    "x": m["return_axis"][0],
                    "y": m["return_axis"][1],
                    "z": m["return_axis"][2],
                    "max": m["max_radius"],
                    "path_length": m["path_length"],
                })
        for name in ["nav_vs_vio_position", "nav_vs_vio_velocity", "nav_tracking_setpoint"]:
            m = metrics[name]
            if m:
                rows.append({
                    "metric": name,
                    "value": m["rmse"],
                    "x": m["axis_rmse"][0],
                    "y": m["axis_rmse"][1],
                    "z": m["axis_rmse"][2],
                    "max": m["max"],
                    "path_length": "",
                })
        write_csv(args.csv, rows)
        print(f"\nwrote: {args.csv}")


if __name__ == "__main__":
    main()
