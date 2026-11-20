#!/usr/bin/env python3
"""Converts a raw ROS 1 Livox bag (e.g. reg-1.bag) into the ROS 2 MCAP bag
Pelorus reads: PointCloud2 clouds plus the IMU.

The LiDAR is recorded as Livox CustomMsg (livox_ros_driver / livox_ros_driver2),
which Pelorus cannot read. The clouds are built exactly as MapsHD's
livox_bag_aggregate builds reg-1.bag-pc.bag, the input of the benchmark's
other PointCloud2 algorithms (and, through reg-1-ros2, of the orchestration):
- windows of 0.1 s on a fixed grid starting at the first point of the first
  message; a point goes to the window its time falls in, and a window is
  written when the first point after it arrives;
- one PointCloud2 per non-empty window, header stamp = window start, frame
  livox_frame, record time = header stamp;
- fields x, y, z, intensity (float32, the reflectivity), tag, line (uint8) and
  time (float32, seconds after the header stamp), 22 bytes per point.
The IMU messages are copied unchanged (ROS 2 form: the header loses `seq`) at
their record time. Only these two topics are written.

Each stage reports its own failures: a message that cannot be read is skipped
and counted; a bag without the topics, or an output that cannot be written,
ends the conversion with an error.
"""

import argparse
import struct
import sys
import time
import traceback
from pathlib import Path

import numpy as np

WINDOW_NS = 100_000_000          # livox_bag_aggregate: agg_secs = 0.1
FRAME_ID = "livox_frame"         # livox_bag_aggregate's default frame
CUSTOM_POINT = np.dtype([("offset_time", "<u4"), ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                         ("reflectivity", "u1"), ("tag", "u1"), ("line", "u1")])   # 19 bytes, packed
PC_POINT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"),
                     ("tag", "u1"), ("line", "u1"), ("time", "<f4")])               # 22 bytes, packed
CUSTOM_MSG_FIELDS = ["timebase", "point_num", "lidar_id", "rsvd", "points"]
CUSTOM_POINT_FIELDS = ["offset_time", "x", "y", "z", "reflectivity", "tag", "line"]


class ConversionError(Exception):
    """An error that makes the converted bag unusable; reported, conversion stops."""


def log(msg):
    print(f"[livox-raw] {msg}", flush=True)


def msgdef_field_names(text, section=None):
    """Field names of a ROS 1 message definition (or of one of its MSG: sections)."""
    lines = text.splitlines()
    if section is not None:
        start = None
        for i, line in enumerate(lines):
            if line.startswith("MSG:") and line.split(":", 1)[1].strip().endswith(section):
                start = i + 1
                break
        if start is None:
            return []
        lines = lines[start:]
    names = []
    for line in lines:
        if line.startswith("====") or line.startswith("MSG:"):
            break
        line = line.split("#", 1)[0].strip()
        if not line or "=" in line:
            continue
        parts = line.split()
        if len(parts) >= 2:
            names.append(parts[1])
    return names


def check_custom_msgdef(msgdef):
    """The parser below assumes the CustomMsg layout of livox_ros_driver(2)."""
    top = [n for n in msgdef_field_names(msgdef) if n != "header"]
    point = msgdef_field_names(msgdef, "CustomPoint")
    if top != CUSTOM_MSG_FIELDS or point != CUSTOM_POINT_FIELDS:
        raise ConversionError(f"unexpected CustomMsg layout: {top} / CustomPoint {point}")


def parse_custom_msg(raw):
    """(timebase_ns, points array) of a ROS 1 serialized Livox CustomMsg."""
    off = 12                                          # header: seq, stamp sec, stamp nsec
    (flen,) = struct.unpack_from("<I", raw, off)
    off += 4 + flen                                   # frame_id
    timebase, point_num = struct.unpack_from("<QI", raw, off)
    off += 8 + 4 + 1 + 3                              # timebase, point_num, lidar_id, rsvd[3]
    (n,) = struct.unpack_from("<I", raw, off)
    off += 4
    if off + n * CUSTOM_POINT.itemsize != len(raw):
        raise ValueError(f"{n} points do not fill the message ({len(raw)} bytes)")
    points = np.frombuffer(raw, dtype=CUSTOM_POINT, count=n, offset=off)
    return int(timebase), points


def convert(bag, out, livox_topic, cloud_topic, imu_topic):
    from rosbags.rosbag1 import Reader
    from rosbags.rosbag2 import StoragePlugin, Writer
    from rosbags.typesys import Stores, get_typestore

    ros1 = get_typestore(Stores.ROS1_NOETIC)
    ros2 = get_typestore(Stores.ROS2_HUMBLE)
    T = ros2.types
    Header, Time = T["std_msgs/msg/Header"], T["builtin_interfaces/msg/Time"]
    PointField, PointCloud2, Imu = T["sensor_msgs/msg/PointField"], T["sensor_msgs/msg/PointCloud2"], T["sensor_msgs/msg/Imu"]
    fields = [PointField(name=name, offset=PC_POINT.fields[name][1], datatype=dt, count=1)
              for name, dt in (("x", 7), ("y", 7), ("z", 7), ("intensity", 7), ("tag", 2), ("line", 2), ("time", 7))]

    def ros2_header(stamp_ns):
        return Header(stamp=Time(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000), frame_id=FRAME_ID)

    started = time.time()
    with Reader(Path(bag)) as reader:
        topics = reader.topics
        lidar = [c for c in reader.connections if c.topic == livox_topic]
        imu = [c for c in reader.connections if c.topic == imu_topic]
        if not lidar or not all(c.msgtype.endswith("/msg/CustomMsg") for c in lidar):
            raise ConversionError(f"no Livox CustomMsg topic {livox_topic} in {bag}")
        if not imu or not all(c.msgtype == "sensor_msgs/msg/Imu" for c in imu):
            raise ConversionError(f"no sensor_msgs/Imu topic {imu_topic} in {bag}")
        for c in lidar:
            check_custom_msgdef(c.msgdef.data)
        n_lidar, n_imu = topics[livox_topic].msgcount, topics[imu_topic].msgcount
        log(f"{bag}: {n_lidar} CustomMsg on {livox_topic}, {n_imu} IMU messages on {imu_topic}")

        with Writer(Path(out), version=9, storage_plugin=StoragePlugin.MCAP) as writer:
            wc = writer.add_connection(cloud_topic, "sensor_msgs/msg/PointCloud2", typestore=ros2)
            wi = writer.add_connection(imu_topic, "sensor_msgs/msg/Imu", typestore=ros2)

            stats = {"lidar": 0, "imu": 0, "clouds": 0, "points": 0, "empty": 0, "bad_lidar": 0, "bad_imu": 0}
            w0 = None            # start of window 0 (first point of the first message)
            cur = None           # index of the window being filled
            buf = []             # PC_POINT arrays of that window
            t_max = None         # latest point time so far: windows only move forward

            def flush():
                if cur is None or not buf:
                    return
                pts = np.concatenate(buf)
                start = w0 + cur * WINDOW_NS
                msg = PointCloud2(header=ros2_header(start), height=1, width=len(pts), fields=fields,
                                  is_bigendian=False, point_step=PC_POINT.itemsize,
                                  row_step=PC_POINT.itemsize * len(pts),
                                  data=np.frombuffer(pts.tobytes(), dtype=np.uint8), is_dense=False)
                writer.write(wc, start, ros2.serialize_cdr(msg, "sensor_msgs/msg/PointCloud2"))
                stats["clouds"] += 1
                stats["points"] += len(pts)
                buf.clear()

            for conn, rec_ns, raw in reader.messages(connections=lidar + imu):
                if conn.topic == imu_topic:
                    stats["imu"] += 1
                    try:
                        m = ros1.deserialize_ros1(raw, "sensor_msgs/msg/Imu")
                        s = m.header.stamp
                        msg = Imu(header=ros2_header(int(s.sec) * 1_000_000_000 + int(s.nanosec)),
                                  orientation=m.orientation, orientation_covariance=m.orientation_covariance,
                                  angular_velocity=m.angular_velocity,
                                  angular_velocity_covariance=m.angular_velocity_covariance,
                                  linear_acceleration=m.linear_acceleration,
                                  linear_acceleration_covariance=m.linear_acceleration_covariance)
                        msg.header.frame_id = m.header.frame_id
                        writer.write(wi, rec_ns, ros2.serialize_cdr(msg, "sensor_msgs/msg/Imu"))
                    except Exception as e:  # rosbags raises several types for a broken message
                        stats["bad_imu"] += 1
                        if stats["bad_imu"] <= 5:
                            log(f"WARNING: IMU message at {rec_ns / 1e9:.3f} cannot be converted ({e}), skipped")
                    continue

                stats["lidar"] += 1
                try:
                    timebase, points = parse_custom_msg(raw)
                except (ValueError, struct.error) as e:
                    stats["bad_lidar"] += 1
                    if stats["bad_lidar"] <= 5:
                        log(f"WARNING: CustomMsg at {rec_ns / 1e9:.3f} cannot be read ({e}), skipped")
                    continue
                if len(points) == 0:
                    stats["empty"] += 1
                    continue
                t_ns = np.uint64(timebase) + points["offset_time"].astype(np.uint64)
                if w0 is None:
                    w0 = int(t_ns[0])
                    cur = 0
                    t_max = w0
                # The window index never decreases (livox_bag_aggregate only rolls
                # forward), so a late point stays in the current window.
                t_eff = np.maximum.accumulate(np.maximum(t_ns, np.uint64(t_max)))
                t_max = int(t_eff[-1])
                win = ((t_eff - np.uint64(w0)) // np.uint64(WINDOW_NS)).astype(np.int64)
                cuts = np.flatnonzero(np.diff(win)) + 1
                for seg in np.split(np.arange(len(points)), cuts):
                    k = int(win[seg[0]])
                    if k != cur:
                        flush()
                        cur = k
                    p = points[seg]
                    rec = np.empty(len(seg), dtype=PC_POINT)
                    rec["x"], rec["y"], rec["z"] = p["x"], p["y"], p["z"]
                    rec["intensity"] = p["reflectivity"].astype(np.float32)
                    rec["tag"], rec["line"] = p["tag"], p["line"]
                    start = w0 + k * WINDOW_NS
                    rec["time"] = ((t_ns[seg].astype(np.int64) - start) * 1e-9).astype(np.float32)
                    buf.append(rec)
                if stats["lidar"] % 500 == 0:
                    log(f"{stats['lidar']} / {n_lidar} CustomMsg, {stats['clouds']} clouds written, "
                        f"{(time.time() - started) / 60:.1f} min")
            flush()

    log(f"done in {(time.time() - started) / 60:.1f} min: {stats['clouds']} clouds ({stats['points']} points) "
        f"from {stats['lidar']} CustomMsg ({stats['empty']} empty, {stats['bad_lidar']} unreadable), "
        f"{stats['imu'] - stats['bad_imu']} of {stats['imu']} IMU messages")
    if stats["clouds"] == 0:
        raise ConversionError("no point cloud was written")
    if stats["imu"] - stats["bad_imu"] == 0:
        raise ConversionError("no IMU message was written")
    return stats


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="raw ROS 1 bag (e.g. reg-1.bag)")
    ap.add_argument("--out", required=True, help="output ROS 2 bag directory (must not exist)")
    ap.add_argument("--livox-topic", default="/livox/lidar", help="Livox CustomMsg topic in the bag")
    ap.add_argument("--cloud-topic", default="/livox/pointcloud", help="PointCloud2 topic to write")
    ap.add_argument("--imu-topic", default="/livox/imu", help="IMU topic (read and written)")
    args = ap.parse_args()
    try:
        convert(args.bag, args.out, args.livox_topic, args.cloud_topic, args.imu_topic)
    except ConversionError as e:
        log(f"ERROR: {e}")
        sys.exit(1)
    except Exception:
        log("ERROR: the conversion failed unexpectedly:")
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
