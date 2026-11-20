#!/usr/bin/env python3
"""Converts a Pelorus run to an HDMapping session (chunked LAZ + trajectory
CSVs + poses.reg + session.json), the format the C++ *-to-hdmapping converters
of the other benchmarks produce.

Pelorus (`pelorus offline` with output.mcap.file) writes an MCAP with, per
scan, its registered cloud on /registered and its odometry on /odom, both with
the same stamp: the end of the scan, in bag time. The cloud is the full,
deskewed scan in the odometry's child frame (the LiDAR frame), so world points
are pose * point, exactly as a ROS consumer of /odom and /registered places
them. Cloud and odometry are paired by that stamp.

Each stage reports its own failures: a message that cannot be decoded is
skipped and counted, a chunk that cannot be written ends the conversion with an
error.
"""

import argparse
import json
import os
import sys
import traceback

import numpy as np

CHUNK_SIZE = 2_000_000
MIN_RANGE_M = 0.5            # blind-zone filter, as in the other converters
CLOUD_TOPIC = "/registered"  # Pelorus' output.mcap.cloud_topic default
ODOM_TOPIC = "/odom"         # Pelorus' output.mcap.odom_topic default
CLOUD_TYPE = "sensor_msgs/msg/PointCloud2"
ODOM_TYPE = "nav_msgs/msg/Odometry"


class ConversionError(Exception):
    """An error that makes the session incomplete; reported, conversion stops."""


def quat_to_rot(qx, qy, qz, qw):
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0:
        raise ValueError("zero quaternion")
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def tait_bryan_from_matrix(m):
    # Same convention as pose_tait_bryan_from_affine_matrix in the C++ converters.
    if m[0, 2] < 1:
        if m[0, 2] > -1:
            fi = np.arcsin(m[0, 2])
            om = np.arctan2(-m[1, 2], m[2, 2])
            ka = np.arctan2(-m[0, 1], m[0, 0])
        else:
            fi = -np.pi / 2.0
            om = -np.arctan2(m[1, 0], m[1, 1])
            ka = 0.0
    else:
        fi = np.pi / 2.0
        om = np.arctan2(m[1, 0], m[1, 1])
        ka = 0.0
    return om, fi, ka


def iter_mcap(mcap_path, topics):
    """Yields (topic, raw CDR bytes) in log-time order; falls back to a plain
    sequential read when the file has no index (e.g. Pelorus was killed)."""
    from mcap.reader import NonSeekingReader, make_reader

    yielded = 0
    try:
        with open(mcap_path, "rb") as f:
            reader = make_reader(f)
            for _, channel, message in reader.iter_messages(topics=topics):
                yielded += 1
                yield channel.topic, message.data
        return
    except Exception as e:  # mcap raises several types for a damaged file
        if yielded:
            # A sequential re-read would repeat these messages; keep what was read.
            print(f"WARNING: reading {mcap_path} stopped after {yielded} messages "
                  f"({type(e).__name__}: {e}); the session holds the scans before that")
            return
        print(f"WARNING: indexed read of {mcap_path} failed ({type(e).__name__}: {e}); "
              "reading it sequentially")
    try:
        with open(mcap_path, "rb") as f:
            # File order (Pelorus writes in time order); time-ordering here would
            # load the whole file and lose everything at the first broken record.
            for _, channel, message in NonSeekingReader(f).iter_messages(topics=topics, log_time_order=False):
                yielded += 1
                yield channel.topic, message.data
    except Exception as e:  # a truncated file ends in a partial record
        print(f"WARNING: sequential read of {mcap_path} stopped after {yielded} messages "
              f"({type(e).__name__}: {e}); the session holds the scans before that")


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def odom_to_matrix(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(q.x, q.y, q.z, q.w)
    T[:3, 3] = (p.x, p.y, p.z)
    if not np.all(np.isfinite(T)):
        raise ValueError("non-finite pose")
    return T


def parse_pointcloud2(msg):
    """Returns (N,3) float32 xyz and (N,) float32 intensity, or raises ValueError."""
    offsets = {f.name: (f.offset, f.datatype) for f in msg.fields}
    missing = [k for k in ("x", "y", "z") if k not in offsets]
    if missing:
        raise ValueError(f"point cloud lacks the fields {missing}")
    step = msg.point_step
    n = msg.width * msg.height
    buf = np.frombuffer(msg.data, dtype=np.uint8)[: n * step].reshape(n, step)

    def field_f32(name):
        off = offsets[name][0]
        return buf[:, off:off + 4].copy().view(np.float32).reshape(n)

    xyz = np.stack([field_f32("x"), field_f32("y"), field_f32("z")], axis=1)
    intensity = field_f32("intensity") if "intensity" in offsets else np.zeros(n, np.float32)
    return xyz, intensity


def read_scans(mcap_path):
    """Yields (stamp_ns, 4x4 pose, PointCloud2) per scan, paired by stamp; returns counters."""
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    clouds, odoms = {}, {}
    stats = {"clouds": 0, "odoms": 0, "undecodable": 0}
    for topic, raw in iter_mcap(mcap_path, [CLOUD_TOPIC, ODOM_TOPIC]):
        try:
            if topic == CLOUD_TOPIC:
                msg = typestore.deserialize_cdr(raw, CLOUD_TYPE)
                stats["clouds"] += 1
                t = stamp_ns(msg.header)
                if t in odoms:
                    yield t, odoms.pop(t), msg
                else:
                    clouds[t] = msg
            else:
                msg = typestore.deserialize_cdr(raw, ODOM_TYPE)
                stats["odoms"] += 1
                t = stamp_ns(msg.header)
                T = odom_to_matrix(msg)
                if t in clouds:
                    yield t, T, clouds.pop(t)
                else:
                    odoms[t] = T
        except (ValueError, TypeError, AttributeError, KeyError, IndexError) as e:
            stats["undecodable"] += 1
            if stats["undecodable"] <= 5:
                print(f"WARNING: a {topic} message cannot be decoded ({type(e).__name__}: {e}), skipped")
    stats["clouds_without_odom"] = len(clouds)
    stats["odoms_without_cloud"] = len(odoms)
    return stats


def save_laz(path, xyz, intensity, ts_ns):
    import laspy

    header = laspy.LasHeader(version="1.2", point_format=1)
    header.scales = np.array([0.0001, 0.0001, 0.0001])
    header.offsets = np.array([0.0, 0.0, 0.0])
    las = laspy.LasData(header)
    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]
    las.intensity = np.clip(intensity, 0, 65535).astype(np.uint16)
    las.gps_time = ts_ns.astype(np.float64)
    las.write(str(path))
    print(f"saved {path} ({xyz.shape[0]} points)")


def convert(mcap_path, out, point_skip=1):
    if not os.path.isfile(mcap_path):
        raise ConversionError(f"{mcap_path} does not exist")
    os.makedirs(out, exist_ok=True)

    chunks = []          # dicts: xyz, intensity, ts, ts_min, ts_max
    cur_xyz, cur_int, cur_ts = [], [], []
    cur_count = 0
    poses = []           # (stamp_ns, pose) of every scan, in time order
    unreadable = 0
    intensity_scale = None

    def flush_chunk():
        nonlocal cur_xyz, cur_int, cur_ts, cur_count
        if cur_count == 0:
            return
        xyz = np.concatenate(cur_xyz)
        inten = np.concatenate(cur_int)
        ts = np.concatenate(cur_ts)
        chunks.append({"xyz": xyz, "intensity": inten, "ts": ts,
                       "ts_min": int(ts.min()), "ts_max": int(ts.max())})
        cur_xyz, cur_int, cur_ts = [], [], []
        cur_count = 0

    print(f"Reading {mcap_path} ({CLOUD_TOPIC} + {ODOM_TOPIC})...")
    gen = read_scans(mcap_path)
    stats = {}
    while True:
        try:
            t_ns, T, msg = next(gen)
        except StopIteration as stop:
            stats = stop.value or {}
            break
        poses.append((t_ns, T))
        try:
            xyz, intensity = parse_pointcloud2(msg)
        except (ValueError, TypeError, AttributeError) as e:
            unreadable += 1
            if unreadable <= 5:
                print(f"WARNING: cloud at {t_ns / 1e9:.3f} cannot be read ({e}), its pose is kept without points")
            continue
        finite = np.isfinite(xyz).all(axis=1)
        keep = finite & ((xyz * xyz).sum(axis=1) > MIN_RANGE_M * MIN_RANGE_M)
        xyz, intensity = xyz[keep], intensity[keep]
        if point_skip > 1:
            xyz, intensity = xyz[::point_skip], intensity[::point_skip]
        if xyz.shape[0] == 0:
            continue
        if intensity_scale is None:
            # Pelorus normalizes intensity to 0..1 (raw / 255); LAZ keeps the raw value.
            intensity_scale = 255.0 if float(np.max(intensity)) <= 1.0 else 1.0
        world = xyz.astype(np.float64) @ T[:3, :3].T + T[:3, 3]
        cur_xyz.append(world)
        cur_int.append(intensity * intensity_scale)
        cur_ts.append(np.full(world.shape[0], t_ns, dtype=np.int64))
        cur_count += world.shape[0]
        if cur_count > CHUNK_SIZE:
            flush_chunk()
    flush_chunk()

    print(f"Clouds: {stats.get('clouds', '?')}, odometry messages: {stats.get('odoms', '?')}, "
          f"paired scans: {len(poses)}, clouds without odometry: {stats.get('clouds_without_odom', '?')}, "
          f"odometry without cloud: {stats.get('odoms_without_cloud', '?')}, "
          f"undecodable: {stats.get('undecodable', '?')}, unreadable clouds: {unreadable}; chunks: {len(chunks)}")
    if not poses:
        raise ConversionError(f"no scan with both {CLOUD_TOPIC} and {ODOM_TOPIC} in {mcap_path}")
    if not chunks:
        raise ConversionError("no scan has points; the session would be empty")
    poses.sort(key=lambda p: p[0])

    # ── Index the trajectory into chunks (first matching chunk, like the C++) ─
    chunks_traj = [[] for _ in chunks]
    for ts_ns, T in poses:
        for j, ch in enumerate(chunks):
            if ch["ts_min"] <= ts_ns <= ch["ts_max"]:
                chunks_traj[j].append((ts_ns, T))
                break
    for j, trj in enumerate(chunks_traj):
        print(f"chunk {j}: {len(trj)} trajectory elements, {chunks[j]['xyz'].shape[0]} points")

    # ── Offset = mean of the trajectory positions in chunks ──────────────────
    all_pos = [T[:3, 3] for trj in chunks_traj for (_, T) in trj]
    offset = np.mean(all_pos, axis=0) if all_pos else np.zeros(3)

    # ── Save LAZ + trajectory CSVs ───────────────────────────────────────────
    m_poses, file_names = [], []
    csv_header = ("timestamp_nanoseconds pose00 pose01 pose02 pose03 "
                  "pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 "
                  "timestampUnix_nanoseconds om_rad fi_rad ka_rad")

    for i, (ch, trj) in enumerate(zip(chunks, chunks_traj)):
        if not trj:
            print(f"chunk {i}: no trajectory element, left out")
            continue
        first_T = trj[0][1]
        first_inv = np.linalg.inv(first_T)
        local = ch["xyz"] @ first_inv[:3, :3].T + first_inv[:3, 3]

        laz_name = f"scan_lio_{i}.laz"
        try:
            save_laz(os.path.join(out, laz_name), local, ch["intensity"], ch["ts"])
        except Exception as e:  # laspy/lazrs raise several types; any failure leaves a hole
            raise ConversionError(f"cannot write {laz_name}: {type(e).__name__}: {e}") from e
        file_names.append(laz_name)
        m_poses.append(first_T.copy())

        csv_path = os.path.join(out, f"trajectory_lio_{i}.csv")
        try:
            with open(csv_path, "w") as f:
                f.write(csv_header + "\n")
                for ts_ns, T in trj:
                    rel = first_inv @ T
                    om, fi, ka = tait_bryan_from_matrix(T)
                    vals = " ".join(f"{rel[r, c]:.10g}" for r in range(3) for c in range(4))
                    f.write(f"{ts_ns} {vals} {ts_ns} {om:.20g} {fi:.20g} {ka:.20g} \n")
        except OSError as e:
            raise ConversionError(f"cannot write {csv_path}: {e}") from e
        print(f"saved {csv_path}")

    if not file_names:
        raise ConversionError("no chunk has a trajectory element; the session would be empty")

    for T in m_poses:
        T[:3, 3] -= offset

    # ── poses.reg / lio_initial_poses.reg ────────────────────────────────────
    def save_poses_reg(path):
        with open(path, "w") as f:
            f.write(f"{len(m_poses)}\n")
            for name, T in zip(file_names, m_poses):
                f.write(name + "\n")
                for r in range(3):
                    f.write(" ".join(f"{T[r, c]:.6g}" for c in range(4)) + "\n")
                f.write("0 0 0 1\n")

    try:
        save_poses_reg(os.path.join(out, "lio_initial_poses.reg"))
        save_poses_reg(os.path.join(out, "poses.reg"))
    except OSError as e:
        raise ConversionError(f"cannot write the pose files in {out}: {e}") from e

    # ── session.json ─────────────────────────────────────────────────────────
    out_abs = os.path.abspath(out)
    session = {
        "Session Settings": {
            "folder_name": out_abs,
            "initial_poses_file_name": os.path.join(out_abs, "lio_initial_poses.reg"),
            "lidar_odometry_version": "HdMap",
            "offset_x": 0.0,
            "offset_y": 0.0,
            "offset_z": 0.0,
            "out_folder_name": out_abs,
            "out_poses_file_name": os.path.join(out_abs, "poses.reg"),
            "poses_file_name": os.path.join(out_abs, "poses.reg"),
        },
        "laz_file_names": [{"file_name": os.path.join(out_abs, n)} for n in file_names],
    }
    session_path = os.path.join(out_abs, "session.json")
    try:
        with open(session_path, "w") as f:
            json.dump(session, f, indent=2, sort_keys=True)
    except OSError as e:
        raise ConversionError(f"cannot write {session_path}: {e}") from e
    print(f"saving file: '{session_path}'")
    print("=== CONVERSION DONE ===")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mcap", required=True, help="Pelorus output MCAP (output.mcap.file)")
    ap.add_argument("--out", required=True, help="output HDMapping session directory")
    ap.add_argument("--point-skip", type=int, default=1, help="keep every Nth point (default 1 = all)")
    args = ap.parse_args()
    try:
        convert(args.mcap, args.out, args.point_skip)
    except ConversionError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception:
        print("ERROR: the conversion failed unexpectedly:", file=sys.stderr)
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
