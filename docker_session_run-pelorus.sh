#!/bin/bash
# Runs Pelorus on a ROS 2 bag and converts the result to an HDMapping session.
#
# Pelorus runs OFFLINE (`pelorus offline`), like PIN-SLAM and KISS-SLAM: its
# bag player Minot reads the ROS 2 bag directly and releases the next scan only
# when the previous one is registered, so there is no ROS, no playback in real
# time, no dropped scan, and every output is stamped with bag time. Pelorus
# writes its registered clouds and odometry to an MCAP; the converter turns
# that into the HDMapping session. Live progress is shown in a Rerun viewer
# (instead of RViz).
#
# Minot 0.15.2 cannot replay a sqlite3 (.db3) bag in this mode (Pelorus never
# gets past its IMU initialization), so a .db3 input such as reg-1-ros2 is
# first copied, the two topics Pelorus reads only, into an MCAP bag under
# <output_dir>/pelorus-input/. Later runs reuse it.
#
# Usage:
#   ./docker_session_run-pelorus.sh <ros2_bag_dir> <output_dir>
#
# Environment variables:
#   CONFIG_FILE         - Pelorus config (Ratslang .rl) on the host
#                         (default: overlay/config/pelorus_bunker.rl, built into the image)
#   VIS                 - 1 = Rerun viewer (default: 1; auto-off without DISPLAY)
#   LIBGL_SW            - 1 = viewer always rendered on the CPU
#                         (default: 0 = the GPU when one is found, see below)
#   POINT_SKIP          - converter keeps every Nth point (default 1 = all)
#   CONVERT_RESULTS_DIR - container path of an existing results dir
#                         (e.g. /output/pelorus-results/<timestamp>): skip Pelorus, only convert

IMAGE_NAME='pelorus_standalone'
OUTPUT_NAME='output_hdmapping-pelorus'
RESULTS_NAME='pelorus-results'

DATASET_CONTAINER_DIR='/pelorus_ws/dataset'
OUTPUT_CONTAINER='/output'
DEFAULT_CONFIG_CONTAINER='/pelorus_ws/config/pelorus_bunker.rl'
CONFIG_MOUNT_CONTAINER='/pelorus_ws/config/custom.rl'

CONFIG_FILE="${CONFIG_FILE:-}"
VIS="${VIS:-1}"
POINT_SKIP="${POINT_SKIP:-1}"
CONVERT_RESULTS_DIR="${CONVERT_RESULTS_DIR:-}"

LIBGL_SW="${LIBGL_SW:-0}"

usage() {
  echo "Usage:"
  echo "  $0 <ros2_bag_dir> <output_dir>"
  echo
  echo "  ros2_bag_dir : ROS 2 bag directory with /livox/pointcloud and /livox/imu (e.g. reg-1-ros2)"
  echo
  echo "If no arguments are provided, a GUI file selector will be used."
  echo
  echo "Environment variables:"
  echo "  CONFIG_FILE - Pelorus config (.rl) on the host   (default: the Bunker config in the image)"
  echo "  VIS         - 1/0, Rerun live viewer             (default: 1)"
  echo "  LIBGL_SW    - 1 = viewer always on the CPU       (default: 0 = GPU if found)"
  echo "  POINT_SKIP  - converter keeps every Nth point    (default: 1)"
  exit 1
}

echo "=== Pelorus offline pipeline ==="

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
fi

if [[ $# -eq 2 ]]; then
  DATASET_HOST_PATH="$1"
  OUTPUT_HOST_DIR="$2"
elif [[ $# -eq 0 ]]; then
  command -v zenity >/dev/null || {
    echo "Error: zenity is not available"
    exit 1
  }
  DATASET_HOST_PATH=$(zenity --file-selection --directory --title="Select the ROS 2 bag directory")
  OUTPUT_HOST_DIR=$(zenity --file-selection --directory --title="Select output directory")
else
  usage
fi

if [[ -z "$DATASET_HOST_PATH" || -z "$OUTPUT_HOST_DIR" ]]; then
  echo "Error: no file or directory selected"
  exit 1
fi

if [[ ! -e "$DATASET_HOST_PATH" ]]; then
  echo "Error: input does not exist: $DATASET_HOST_PATH"
  exit 1
fi

CONFIG_MOUNT=()
CONFIG_CONTAINER="$DEFAULT_CONFIG_CONTAINER"
CONFIG_DESC="pelorus_bunker.rl (in the image)"
if [[ -n "$CONFIG_FILE" ]]; then
  if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "Error: CONFIG_FILE does not exist: $CONFIG_FILE"
    exit 1
  fi
  CONFIG_FILE=$(realpath "$CONFIG_FILE")
  CONFIG_MOUNT=(-v "$CONFIG_FILE":"$CONFIG_MOUNT_CONTAINER":ro)
  CONFIG_CONTAINER="$CONFIG_MOUNT_CONTAINER"
  CONFIG_DESC="$CONFIG_FILE"
fi

mkdir -p "$OUTPUT_HOST_DIR"
DATASET_HOST_PATH=$(realpath "$DATASET_HOST_PATH")
OUTPUT_HOST_DIR=$(realpath "$OUTPUT_HOST_DIR")
DATASET_BASE=$(basename "$DATASET_HOST_PATH")

# Viewer needs X11 (same recipe as the RViz-based benchmarks).
VIS_ARG=""
if [[ "$VIS" == "1" && -n "$DISPLAY" ]]; then
  VIS_ARG="--visualize"
  xhost +local:docker >/dev/null 2>&1 || true
elif [[ "$VIS" == "1" ]]; then
  echo "No DISPLAY: the viewer is off."
fi
SW_ARG=""
[[ "$LIBGL_SW" == "1" ]] && SW_ARG="--software-render"

# ── GPU for the viewer: used when present, never required ────────────────────
# Pelorus itself runs on the CPU (CPU build). Only the Rerun viewer, a separate
# process, uses the GPU. AMD/Intel: the /dev/dri device nodes, plus their
# groups (the container runs as 1000:1000, outside the host's render/video
# groups). NVIDIA: the container toolkit's graphics libraries (Vulkan and GL)
# through its runtime (--runtime=nvidia works in both the legacy and the CDI
# mode of the toolkit; --gpus fails in CDI mode). If Docker refuses to start
# the container with these options, the run is retried with fewer (below);
# inside the container the viewer falls back to software rendering, then to no
# viewer. A viewer failure never stops Pelorus.
DRI_ARGS=()
NVIDIA_ARGS=()
if [[ -n "$VIS_ARG" && "$LIBGL_SW" != "1" ]]; then
  if compgen -G "/dev/dri/renderD*" >/dev/null; then
    DRI_ARGS+=(--device /dev/dri)
    for gid in $(for dev in /dev/dri/card* /dev/dri/renderD*; do [[ -c "$dev" ]] && stat -c %g "$dev"; done | sort -u); do
      DRI_ARGS+=(--group-add "$gid")
    done
  fi
  # As in step 9 of the orchestration: the nvidia runtime when Docker lists it,
  # otherwise --gpus (toolkit setups without the registered runtime).
  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
    if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; then
      NVIDIA_ARGS=(--runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all)
    else
      NVIDIA_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
    fi
  fi
fi

# Options to try, best first; Docker exit code 125 (the container could not be
# created) moves on to the next.
ATTEMPT_DESC=()
ATTEMPT_ARGS=()
if [[ -n "$VIS_ARG" ]]; then
  if [[ ${#DRI_ARGS[@]} -gt 0 && ${#NVIDIA_ARGS[@]} -gt 0 ]]; then
    ATTEMPT_DESC+=("GPU: /dev/dri + NVIDIA"); ATTEMPT_ARGS+=("${DRI_ARGS[*]} ${NVIDIA_ARGS[*]}")
  fi
  if [[ ${#NVIDIA_ARGS[@]} -gt 0 && ${#DRI_ARGS[@]} -eq 0 ]]; then
    ATTEMPT_DESC+=("GPU: NVIDIA"); ATTEMPT_ARGS+=("${NVIDIA_ARGS[*]}")
  fi
  if [[ ${#DRI_ARGS[@]} -gt 0 ]]; then
    ATTEMPT_DESC+=("GPU: /dev/dri"); ATTEMPT_ARGS+=("${DRI_ARGS[*]}")
  fi
  ATTEMPT_DESC+=("CPU rendering"); ATTEMPT_ARGS+=("")
  if [[ "$LIBGL_SW" == "1" ]]; then
    VIEWER_DESC="on, CPU rendering (LIBGL_SW=1)"
  else
    VIEWER_DESC="on, ${ATTEMPT_DESC[0]}"
  fi
else
  ATTEMPT_DESC+=("no viewer"); ATTEMPT_ARGS+=("")
  VIEWER_DESC="off"
fi

echo "Input           : $DATASET_HOST_PATH"
echo "Output dir      : $OUTPUT_HOST_DIR"
echo "Session         : $OUTPUT_HOST_DIR/$OUTPUT_NAME"
echo "Config          : $CONFIG_DESC"
echo "Viewer          : $VIEWER_DESC"

RESULTS_ARG=""
if [[ -n "$CONVERT_RESULTS_DIR" ]]; then
  RESULTS_ARG="--results-dir $CONVERT_RESULTS_DIR"
  echo "Pelorus         : skipped, converting $CONVERT_RESULTS_DIR"
fi

# ── Run Pelorus, then convert, in one container ──────────────────────────────
# HOME: Minot and the viewer write caches there; the container user has none.
# --shm-size: Pelorus and Minot pass the clouds through shared memory, and
# Docker's default /dev/shm is only 64 MB.
# $@: the GPU options of one attempt.
run_container() {
docker run -it --rm \
  --network host \
  --shm-size 2g \
  -e DISPLAY="$DISPLAY" \
  "$@" \
  -e HOME=/tmp/pelorus-home \
  -e XDG_RUNTIME_DIR=/tmp/xdg \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DATASET_HOST_PATH":"$DATASET_CONTAINER_DIR/$DATASET_BASE":ro \
  -v "$OUTPUT_HOST_DIR":"$OUTPUT_CONTAINER" \
  "${CONFIG_MOUNT[@]}" \
  "$IMAGE_NAME" \
  /bin/bash -c "
    mkdir -p /tmp/pelorus-home /tmp/xdg '$OUTPUT_CONTAINER/logs'
    chmod 700 /tmp/xdg
    cd /tmp/pelorus-home
    python3 -u /pelorus_ws/scripts/pelorus_bench.py \
      --bag '$DATASET_CONTAINER_DIR/$DATASET_BASE' \
      --config '$CONFIG_CONTAINER' \
      --out-root '$OUTPUT_CONTAINER/$RESULTS_NAME' \
      --session '$OUTPUT_CONTAINER/$OUTPUT_NAME' \
      --point-skip '$POINT_SKIP' \
      $VIS_ARG $SW_ARG $RESULTS_ARG 2>&1 | tee '$OUTPUT_CONTAINER/logs/pelorus.log'
    exit \${PIPESTATUS[0]}
  "
}

status=125
for i in "${!ATTEMPT_ARGS[@]}"; do
  if [[ $i -gt 0 ]]; then
    echo "=== Docker could not start the container with ${ATTEMPT_DESC[$((i - 1))]} (see above); retrying with ${ATTEMPT_DESC[$i]} ==="
  fi
  # Word splitting is intended: the options hold no spaces.
  # shellcheck disable=SC2086
  run_container ${ATTEMPT_ARGS[$i]}
  status=$?
  [[ $status -eq 125 ]] || break
done

if [[ $status -eq 42 ]]; then
  echo "=== ABORTED: input check failed, nothing was run ==="
  exit 1
elif [[ $status -eq 125 ]]; then
  echo "=== FAILED: Docker could not start the container (see the Docker error above) ==="
  exit 1
elif [[ $status -ne 0 ]]; then
  echo "=== FAILED (exit $status): see $OUTPUT_HOST_DIR/logs/pelorus.log ==="
  exit 1
elif [[ ! -f "$OUTPUT_HOST_DIR/$OUTPUT_NAME/session.json" ]]; then
  echo "=== FAILED: no session.json in $OUTPUT_HOST_DIR/$OUTPUT_NAME; see $OUTPUT_HOST_DIR/logs/pelorus.log ==="
  exit 1
fi

echo "=== DONE === Results in: $OUTPUT_HOST_DIR/$OUTPUT_NAME"
