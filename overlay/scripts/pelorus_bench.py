#!/usr/bin/env python3
"""Runs Pelorus offline on a ROS 2 bag and converts the result to an HDMapping session.

Stages, each reporting its own failure:
  1. Input check (exit 42, nothing run), then the bag Pelorus reads:
     - a raw ROS 1 bag (reg-1.bag) with Livox CustomMsg and IMU: converted to
       an MCAP bag of PointCloud2 + IMU, the clouds built as livox_bag_aggregate
       builds them (livox_raw_to_mcap.py);
     - a ROS 2 bag directory (reg-1-ros2) holding the point cloud and IMU
       topics named in the Pelorus config: a sqlite3 (.db3) bag is copied,
       those two topics only, into an MCAP bag; an MCAP bag is used as it is;
     - an .mcap file: used as it is.
     Converted bags are kept for later runs (see mcap_input).
  2. Viewer (optional): a Rerun viewer in its own process. GPU rendering is
     tried first, then software rendering, then no viewer. Pelorus runs the
     same in every case, and a viewer that dies later only loses the picture.
  3. Pelorus: `pelorus offline <bag> -c <config>`. Minot (Pelorus' bag player)
     releases the next scan only when the previous one is registered, so no
     scan is dropped and every output is stamped with bag time. Pelorus writes
     an MCAP (registered clouds + odometry) and a TUM trajectory.
  4. Conversion of the MCAP to the HDMapping session.
"""

import argparse
import glob
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path

EXIT_INPUT = 42
VIEWER_HOST = "127.0.0.1"
VIEWER_PORT = 9876               # Pelorus' default output.rerun.server_addr
VIEWER_START_TIMEOUT_S = 40
VIEWER_SETTLE_S = 5
PROGRESS_PERIOD_S = 15
MCAP_NAME = "pelorus.mcap"
TUM_NAME = "trajectory_tum.txt"
# Pelorus' own defaults, used when the config does not name a topic.
DEFAULT_LIDAR_TOPIC = "/ouster/points"
DEFAULT_IMU_TOPIC = "/ouster/imu"


def log(msg):
    print(f"[pelorus-bench] {msg}", flush=True)


# ── 1. Input check ──────────────────────────────────────────────────────────

def config_topics(config_path):
    """lidar.topic and imu.topic of a Ratslang config (nested or dotted keys).

    A small reader for the plain `key = "value"` lines this needs, not a full
    Ratslang parser. A topic it cannot find falls back to Pelorus' default.
    """
    found = {}
    stack = []
    try:
        with open(config_path) as f:
            lines = f.readlines()
    except OSError as e:
        raise RuntimeError(f"cannot read the config {config_path}: {e}") from e
    for raw in lines:
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        m = re.match(r'^([A-Za-z_][\w.]*)\s*=\s*"([^"]*)"', line)
        if m:
            key = ".".join(stack + [m.group(1)])
            if key in ("lidar.topic", "imu.topic"):
                found[key] = m.group(2)
        m = re.match(r"^([A-Za-z_][\w.]*)\s*\{", line)
        if m:
            stack.append(m.group(1))
            line = line[m.end():]
        for ch in line:
            if ch == "{":
                stack.append("?")
            elif ch == "}" and stack:
                stack.pop()
    return found.get("lidar.topic", DEFAULT_LIDAR_TOPIC), found.get("imu.topic", DEFAULT_IMU_TOPIC)


def bag_topics(bag):
    """{topic: (type, message_count)} from a ROS 2 bag's metadata.yaml."""
    import yaml

    meta_path = os.path.join(bag, "metadata.yaml")
    try:
        with open(meta_path) as f:
            meta = yaml.safe_load(f)
        info = meta["rosbag2_bagfile_information"]
        topics = {}
        for entry in info.get("topics_with_message_count", []):
            tm = entry["topic_metadata"]
            topics[tm["name"]] = (tm["type"], int(entry.get("message_count", 0)))
        return topics, info.get("storage_identifier", "?")
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as e:
        raise RuntimeError(f"cannot read {meta_path}: {e}") from e


def check_ros1_input(bag, livox_topic, imu_topic):
    """Returns the number of Livox CustomMsg in a raw ROS 1 bag. Exits 42 on a bad input."""
    try:
        from rosbags.rosbag1 import Reader

        with Reader(Path(bag)) as reader:
            topics = {name: (info.msgtype, info.msgcount) for name, info in reader.topics.items()}
    except Exception as e:  # rosbags raises several types for a file that is no ROS 1 bag
        log(f"ERROR: cannot read {bag} as a ROS 1 bag: {type(e).__name__}: {e}")
        sys.exit(EXIT_INPUT)
    problems = []
    lidar_type = topics.get(livox_topic, (None, 0))[0] or ""
    if not lidar_type.endswith("/msg/CustomMsg"):
        problems.append(f"topic {livox_topic} (Livox CustomMsg) is not in the bag"
                        + (f", it is {lidar_type}" if lidar_type else ""))
    if topics.get(imu_topic, (None, 0))[0] != "sensor_msgs/msg/Imu":
        problems.append(f"topic {imu_topic} (sensor_msgs/Imu) is not in the bag")
    if problems:
        for p in problems:
            log(f"ERROR: {p}")
        log("Topics in the bag:")
        for name, (typ, count) in sorted(topics.items()):
            log(f"  {name}  {typ}  ({count} messages)")
        sys.exit(EXIT_INPUT)
    n = topics[livox_topic][1]
    log(f"input OK: ROS 1 bag, {n} Livox CustomMsg on {livox_topic}, "
        f"{topics[imu_topic][1]} IMU messages on {imu_topic}")
    return n


def check_input(bag, lidar_topic, imu_topic, livox_topic):
    """Returns the number of point clouds in the bag (0 = unknown). Exits 42 on a bad input."""
    if os.path.isfile(bag) and bag.endswith(".mcap"):
        log(f"input is an MCAP file; topics are not checked before the run")
        return 0
    if os.path.isfile(bag) and bag.endswith(".bag"):
        return check_ros1_input(bag, livox_topic, imu_topic)
    if os.path.isfile(bag):
        log(f"ERROR: {bag} is neither a ROS 1 .bag, an .mcap file nor a ROS 2 bag directory.")
        sys.exit(EXIT_INPUT)
    if not os.path.isdir(bag) or not os.path.isfile(os.path.join(bag, "metadata.yaml")):
        log(f"ERROR: {bag} is not a ROS 2 bag directory (no metadata.yaml).")
        sys.exit(EXIT_INPUT)
    try:
        topics, storage = bag_topics(bag)
    except RuntimeError as e:
        log(f"ERROR: {e}")
        sys.exit(EXIT_INPUT)
    problems = []
    for topic, want in ((lidar_topic, "sensor_msgs/msg/PointCloud2"), (imu_topic, "sensor_msgs/msg/Imu")):
        if topic not in topics:
            problems.append(f"topic {topic} ({want}) is not in the bag")
        elif topics[topic][0] != want:
            problems.append(f"topic {topic} is {topics[topic][0]}, Pelorus needs {want}")
    if problems:
        for p in problems:
            log(f"ERROR: {p}")
        log("Topics in the bag:")
        for name, (typ, count) in sorted(topics.items()):
            log(f"  {name}  {typ}  ({count} messages)")
        sys.exit(EXIT_INPUT)
    n_clouds = topics[lidar_topic][1]
    log(f"input OK: {storage} bag, {n_clouds} clouds on {lidar_topic}, "
        f"{topics[imu_topic][1]} IMU messages on {imu_topic}")
    return n_clouds


def source_signature(bag, *topics):
    """Names, sizes and modification times of the bag's files, plus the topics."""
    paths = [bag] if os.path.isfile(bag) else [os.path.join(bag, n) for n in sorted(os.listdir(bag))]
    entries = []
    for path in paths:
        st = os.stat(path)
        entries.append(f"{os.path.basename(path)} {st.st_size} {st.st_mtime_ns}")
    return "\n".join(entries + ["topics " + " ".join(topics)]) + "\n"


def mcap_input(bag, lidar_topic, imu_topic, livox_topic, cache_root):
    """The bag Pelorus reads: an MCAP bag made from a raw ROS 1 bag or a
    sqlite3 (.db3) bag, else the bag itself.

    A raw ROS 1 bag (reg-1.bag) holds the LiDAR as Livox CustomMsg, which
    Pelorus cannot read; livox_raw_to_mcap.py builds the PointCloud2 clouds
    as livox_bag_aggregate does and writes them with the IMU.

    With a .db3 bag (reg-1-ros2), Minot 0.15.2's lockstep replay never gets
    Pelorus past the IMU initialization: Pelorus acknowledged 16384 IMU
    messages and never saw their stamps advance by 100 ms, although the bag's
    stamps are monotonic at 200 Hz. The same two topics as an MCAP bag (the
    format of the author's own examples) replay correctly. The copy is made
    with rosbags-convert, which changes no message.

    Either result is kept and reused while the source bag is unchanged.
    """
    ros1 = os.path.isfile(bag) and bag.endswith(".bag")
    if os.path.isfile(bag) and not ros1:
        return bag
    if not ros1:
        try:
            _, storage = bag_topics(bag)
        except RuntimeError as e:
            log(f"WARNING: {e}; the bag is passed to Pelorus as it is")
            return bag
        if storage != "sqlite3":
            return bag
    name = os.path.basename(os.path.normpath(bag))
    if ros1:
        name = name[: -len(".bag")]
    dst = os.path.join(cache_root, f"{name}-mcap")
    stamp_file = dst + ".source"
    signature = (source_signature(bag, livox_topic, lidar_topic, imu_topic) if ros1
                 else source_signature(bag, lidar_topic, imu_topic))
    try:
        with open(stamp_file) as f:
            if f.read() == signature and os.path.isfile(os.path.join(dst, "metadata.yaml")):
                log(f"input: reusing the MCAP copy {dst}")
                return dst
    except OSError:
        pass
    # rosbags-convert names the .mcap after its destination folder, so the copy
    # is written under the final name in a staging folder and moved when complete.
    staging = os.path.join(cache_root, ".partial")
    tmp = os.path.join(staging, os.path.basename(dst))
    for old in (staging, dst):
        if os.path.exists(old):
            shutil.rmtree(old)
    if os.path.exists(stamp_file):
        os.remove(stamp_file)
    os.makedirs(staging, exist_ok=True)
    started = time.time()
    if ros1:
        log(f"input: converting the Livox CustomMsg on {livox_topic} to PointCloud2 on {lidar_topic} "
            f"(as livox_bag_aggregate) and copying {imu_topic} into an MCAP bag (once; later runs reuse it)")
        try:
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            import livox_raw_to_mcap
        except Exception as e:
            raise RuntimeError(f"cannot load livox_raw_to_mcap.py: {type(e).__name__}: {e}") from e
        try:
            livox_raw_to_mcap.convert(bag, tmp, livox_topic, lidar_topic, imu_topic)
        except livox_raw_to_mcap.ConversionError as e:
            raise RuntimeError(str(e)) from e
        except Exception as e:  # rosbags/numpy raise several types; report, never crash
            traceback.print_exc()
            raise RuntimeError(f"the conversion of {bag} failed: {type(e).__name__}: {e}") from e
    else:
        cmd = ["rosbags-convert", "--src", bag, "--dst", tmp, "--dst-storage", "mcap",
               "--include-topic", lidar_topic, imu_topic]
        log(f"input: Minot cannot replay this sqlite3 bag; copying {lidar_topic} and {imu_topic} "
            f"into an MCAP bag (once; later runs reuse it): {' '.join(cmd)}")
        try:
            status = subprocess.run(cmd).returncode
        except OSError as e:
            raise RuntimeError(f"cannot run rosbags-convert: {e}") from e
        if status != 0:
            raise RuntimeError(f"rosbags-convert failed (exit {status}); see the messages above")
    if not os.path.isfile(os.path.join(tmp, "metadata.yaml")):
        raise RuntimeError(f"the conversion wrote no bag in {tmp}")
    os.rename(tmp, dst)
    shutil.rmtree(staging, ignore_errors=True)
    with open(stamp_file, "w") as f:
        f.write(signature)
    log(f"input: MCAP copy written in {(time.time() - started) / 60:.1f} min: {dst}")
    return dst


# ── 2. Viewer ───────────────────────────────────────────────────────────────

def port_open(host=VIEWER_HOST, port=VIEWER_PORT, timeout=0.5):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def log_vulkan_devices():
    """Logs the Vulkan devices the container sees (GPU or CPU = software)."""
    try:
        out = subprocess.run(["vulkaninfo", "--summary"], capture_output=True, text=True, timeout=30).stdout
    except (OSError, subprocess.SubprocessError) as e:
        log(f"[viewer] vulkaninfo failed: {e}")
        return
    names = re.findall(r"deviceName\s*=\s*(.+)", out)
    kinds = re.findall(r"deviceType\s*=\s*(\S+)", out)
    if not names:
        log("[viewer] no Vulkan device found")
    for name, kind in zip(names, kinds):
        log(f"[viewer] Vulkan device: {name.strip()} ({kind.replace('PHYSICAL_DEVICE_TYPE_', '')})")


def viewer_attempts(software_only):
    lvp = sorted(glob.glob("/usr/share/vulkan/icd.d/lvp_icd*.json"))
    attempts = []
    if not software_only:
        attempts += [
            ("Vulkan (GPU when one is passed in)", ["--renderer", "vulkan"], {}),
            ("OpenGL (GPU when one is passed in)", ["--renderer", "gl"], {}),
        ]
    if lvp:
        attempts.append(("software rendering, Vulkan lavapipe", ["--renderer", "vulkan"],
                         {"VK_ICD_FILENAMES": ":".join(lvp), "VK_DRIVER_FILES": ":".join(lvp)}))
    attempts.append(("software rendering, OpenGL llvmpipe", ["--renderer", "gl"], {"LIBGL_ALWAYS_SOFTWARE": "1"}))
    return attempts


def stop_process(proc, name, timeout=5):
    if proc is None or proc.poll() is not None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            log(f"WARNING: {name} (pid {proc.pid}) did not stop")
    except OSError as e:
        log(f"WARNING: cannot stop {name}: {e}")


def start_viewer(log_dir, software_only):
    """Starts a Rerun viewer. Returns (process or None, True if Pelorus may stream to a viewer)."""
    if port_open():
        log(f"[viewer] a Rerun viewer already listens on {VIEWER_HOST}:{VIEWER_PORT}; Pelorus streams to it")
        return None, True
    if shutil.which("rerun") is None:
        log("[viewer] off: the rerun command is missing in the image")
        return None, False
    log_vulkan_devices()
    viewer_log = os.path.join(log_dir, "rerun-viewer.log")
    for label, args, env_extra in viewer_attempts(software_only):
        env = dict(os.environ)
        env.update(env_extra)
        env.setdefault("RUST_LOG", "info")
        cmd = ["rerun", "--port", str(VIEWER_PORT), "--bind", VIEWER_HOST,
               # The viewer drops its oldest data beyond this, never Pelorus' results.
               "--memory-limit", "25%"] + args
        try:
            with open(viewer_log, "a") as vlog:
                vlog.write(f"\n===== {label}: {' '.join(cmd)} =====\n")
                vlog.flush()
                proc = subprocess.Popen(cmd, env=env, stdout=vlog, stderr=subprocess.STDOUT,
                                        stdin=subprocess.DEVNULL, start_new_session=True)
        except OSError as e:
            log(f"[viewer] {label}: cannot start ({e})")
            continue
        deadline = time.time() + VIEWER_START_TIMEOUT_S
        while time.time() < deadline and proc.poll() is None and not port_open():
            time.sleep(0.5)
        if proc.poll() is None and port_open():
            time.sleep(VIEWER_SETTLE_S)          # a renderer failure shows up right after start
            if proc.poll() is None:
                log(f"[viewer] Rerun viewer on: {label}")
                return proc, True
        reason = f"exited with {proc.returncode}" if proc.poll() is not None else "did not open its port"
        stop_process(proc, "rerun viewer")
        log(f"[viewer] {label}: {reason}, see {viewer_log}")
    log("[viewer] off: no rendering option worked; Pelorus runs without it")
    return None, False


# ── 3. Pelorus ──────────────────────────────────────────────────────────────

def count_lines(path):
    try:
        with open(path, "rb") as f:
            return sum(1 for _ in f)
    except OSError:
        return 0


def progress_reporter(tum_path, total, stop_event):
    last = -1
    started = time.time()
    while not stop_event.wait(PROGRESS_PERIOD_S):
        n = count_lines(tum_path)
        if n == last:
            continue
        last = n
        elapsed = time.time() - started
        if total > 0:
            log(f"[progress] {n} / {total} scans ({100.0 * n / total:.0f} %), {elapsed / 60:.1f} min")
        else:
            log(f"[progress] {n} scans, {elapsed / 60:.1f} min")


def run_pelorus(bag, config, results_dir, total_clouds, rerun_on):
    mcap_path = os.path.join(results_dir, MCAP_NAME)
    tum_path = os.path.join(results_dir, TUM_NAME)
    cmd = [
        "pelorus", "offline", bag, "-c", config, "-y",
        "-o", f'output.mcap.file="{mcap_path}"',
        "-o", f'output.trajectory="{tum_path}"',
        # The interactive terminal UI ends the run on q, Esc or Ctrl+C; the
        # author's setting for non-interactive sessions turns it off. Display
        # only: the processing is the same.
        "-o", "output.terminal.ui=false",
        "-o", f"output.rerun={'true' if rerun_on else 'false'}",
    ]
    log("running: " + " ".join(cmd))
    stop_event = threading.Event()
    reporter = threading.Thread(target=progress_reporter, args=(tum_path, total_clouds, stop_event), daemon=True)
    started = time.time()
    proc = None
    try:
        proc = subprocess.Popen(cmd, cwd=results_dir)
        reporter.start()
        status = proc.wait()
    except OSError as e:
        log(f"ERROR: cannot start pelorus: {e}")
        return 1
    except KeyboardInterrupt:
        log("interrupted; stopping Pelorus")
        if proc is not None:
            try:
                proc.send_signal(signal.SIGINT)
                proc.wait(timeout=30)
            except (subprocess.TimeoutExpired, OSError):
                proc.kill()
        raise
    finally:
        stop_event.set()
    n = count_lines(tum_path)
    log(f"Pelorus finished with exit code {status} after {(time.time() - started) / 60:.1f} min; "
        f"{n} poses in {TUM_NAME}" + (f" for {total_clouds} clouds" if total_clouds else ""))
    if status == 0 and total_clouds and n < 0.9 * total_clouds:
        log(f"WARNING: Pelorus wrote {n} poses for {total_clouds} clouds")
    return status


# ── main ────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="raw ROS 1 bag, ROS 2 bag directory or .mcap file")
    ap.add_argument("--livox-topic", default="/livox/lidar", help="Livox CustomMsg topic of a raw ROS 1 bag")
    ap.add_argument("--config", required=True, help="Pelorus Ratslang config")
    ap.add_argument("--out-root", required=True, help="parent of the timestamped results directories")
    ap.add_argument("--session", required=True, help="output HDMapping session directory")
    ap.add_argument("--point-skip", type=int, default=1, help="converter keeps every Nth point")
    ap.add_argument("--visualize", action="store_true", help="start a Rerun viewer")
    ap.add_argument("--software-render", action="store_true", help="viewer: skip the GPU attempts")
    ap.add_argument("--results-dir", default="", help="existing results dir: skip Pelorus, only convert")
    args = ap.parse_args()

    viewer = None
    try:
        if args.results_dir:
            results_dir = args.results_dir
            log(f"conversion only, results from {results_dir}")
        else:
            lidar_topic, imu_topic = config_topics(args.config)
            log(f"config {args.config}: lidar.topic {lidar_topic}, imu.topic {imu_topic}")
            total = check_input(args.bag, lidar_topic, imu_topic, args.livox_topic)

            out_parent = os.path.dirname(args.out_root.rstrip("/")) or "."
            try:
                run_bag = mcap_input(args.bag, lidar_topic, imu_topic, args.livox_topic,
                                     os.path.join(out_parent, "pelorus-input"))
            except (RuntimeError, OSError) as e:
                log(f"ERROR: preparing the input failed: {e}")
                return 1

            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            results_dir = os.path.join(args.out_root, stamp)
            log_dir = os.path.join(out_parent, "logs")
            os.makedirs(results_dir, exist_ok=True)
            os.makedirs(log_dir, exist_ok=True)
            try:
                shutil.copy2(args.config, os.path.join(results_dir, "pelorus.rl"))
                latest = os.path.join(args.out_root, "latest")
                if os.path.islink(latest) or os.path.exists(latest):
                    os.remove(latest)
                os.symlink(stamp, latest)
            except OSError as e:
                log(f"WARNING: results bookkeeping failed ({e}); the run continues")

            rerun_on = False
            if args.visualize:
                try:
                    viewer, rerun_on = start_viewer(log_dir, args.software_render)
                except Exception as e:  # the viewer must never cost the run
                    log(f"[viewer] off: unexpected error {type(e).__name__}: {e}")
                    stop_process(viewer, "rerun viewer")
                    viewer, rerun_on = None, False

            status = run_pelorus(run_bag, args.config, results_dir, total, rerun_on)
            if status != 0:
                log(f"ERROR: Pelorus failed (exit {status}); see the messages above")
                return 1

        mcap_path = os.path.join(results_dir, MCAP_NAME)
        if not os.path.isfile(mcap_path) or os.path.getsize(mcap_path) == 0:
            log(f"ERROR: Pelorus wrote no output ({mcap_path} is missing or empty)")
            return 1

        try:
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from pelorus_to_hdmapping import ConversionError, convert
        except Exception as e:
            log(f"ERROR: cannot load the converter: {type(e).__name__}: {e}")
            return 1
        log(f"converting {mcap_path} -> {args.session}")
        try:
            convert(mcap_path, args.session, args.point_skip)
        except ConversionError as e:
            log(f"ERROR: conversion failed: {e}")
            return 1
        except Exception:
            log("ERROR: the conversion failed unexpectedly:")
            traceback.print_exc()
            return 1
        return 0
    except KeyboardInterrupt:
        log("interrupted")
        return 130
    except Exception:
        log("ERROR: unexpected failure:")
        traceback.print_exc()
        return 1
    finally:
        stop_process(viewer, "rerun viewer")


if __name__ == "__main__":
    sys.exit(main())
