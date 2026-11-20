# Pelorus (Osnabrück University, resource-aware LiDAR-inertial odometry),
# built from the pinned submodule src/pelorus, plus its bag player Minot, the
# Rerun viewer and the HDMapping converter.
#
# Ubuntu 24.04: rerun-sdk 0.43 (the viewer matching Pelorus' Rerun 0.43.2)
# needs Python >= 3.11, and Ubuntu 22.04 has 3.10. Pelorus' own release builds
# include 24.04.

# ── Stage 1: build the Pelorus CLI ───────────────────────────────────────────
FROM ubuntu:24.04 AS build

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    pkg-config \
    cmake \
    git \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Rust 1.100.0: the stable release when Pelorus v0.1.0 was built by its release
# CI (2026-11-19, which installs the current stable); the CLI needs >= 1.98.
ENV RUSTUP_HOME=/opt/rustup CARGO_HOME=/opt/cargo PATH=/opt/cargo/bin:$PATH
RUN curl -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path --profile minimal --default-toolchain 1.100.0 && \
    rustc --version && cargo --version

WORKDIR /src
COPY ./src/pelorus ./pelorus

# The CPU build with the CLI's default features, as in the author's CPU
# packages: offline mode (Minot), Rerun visualization, multi-core CPU (rayon).
# No CUDA/Vulkan backend: every stage runs on the CPU, on any machine.
# --locked: exactly the dependency versions of the author's Cargo.lock.
RUN test -f pelorus/pelorus_cli/Cargo.toml || \
        { echo "[build] src/pelorus is empty: clone with --recursive (or git submodule update --init)"; exit 1; } && \
    cd pelorus && \
    cargo build --release --locked -p pelorus_cli && \
    install -m 755 target/release/pelorus /usr/local/bin/pelorus && \
    pelorus config generate full > /dev/null && \
    rm -rf target

# ── Stage 2: runtime ─────────────────────────────────────────────────────────
FROM ubuntu:24.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# python3-venv: Ubuntu 24.04's pip refuses system-wide installs (PEP 668).
# The rest is what the Rerun viewer binary links or loads: libudev, X11 and
# xkbcommon (window and keyboard), fontconfig, dbus, Wayland and EGL, Vulkan
# (libvulkan1 + Mesa's drivers: RADV/ANV for AMD/Intel GPUs and lavapipe for
# software rendering) and Mesa OpenGL (GPU, or llvmpipe on the CPU).
# vulkan-tools: the run log names the Vulkan devices the viewer can use.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-venv \
    ca-certificates \
    libudev1 \
    libdbus-1-3 \
    libfontconfig1 \
    libx11-6 \
    libx11-xcb1 \
    libxcursor1 \
    libxi6 \
    libxrandr2 \
    libxkbcommon0 \
    libxkbcommon-x11-0 \
    libwayland-client0 \
    libwayland-egl1 \
    libegl1 \
    libegl-mesa0 \
    libgl1 \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libvulkan1 \
    mesa-vulkan-drivers \
    vulkan-tools \
    && rm -rf /var/lib/apt/lists/*

# minot-cli 0.15.2: the bag player Pelorus v0.1.0 requires (>= 0.15.2, < 0.16).
# rerun-sdk 0.43.2: the viewer, same version as the Rerun crate in Pelorus.
# mcap + rosbags: the converter reads Pelorus' MCAP and decodes its CDR
# messages; laspy + lazrs write the LAZ chunks; pyyaml reads metadata.yaml.
RUN python3 -m venv /opt/venv && \
    /opt/venv/bin/pip install --no-cache-dir \
    "minot-cli==0.15.2" "rerun-sdk==0.43.2" \
    "mcap==1.6.0" "rosbags==0.11.9" "laspy==2.7.0" "lazrs==0.8.2" "pyyaml==6.0.3" numpy
ENV PATH=/opt/venv/bin:$PATH

COPY --from=build /usr/local/bin/pelorus /usr/local/bin/pelorus

WORKDIR /pelorus_ws
COPY ./overlay/config/ ./config/
COPY ./overlay/scripts/ ./scripts/

# The final guards fail the image build loudly: shared libraries of pelorus,
# minot and the Rerun viewer (all missing ones listed at once), the commands,
# the converter imports, and the benchmark config, validated by Pelorus itself.
RUN RERUN_BIN=$(python3 -c "import importlib.util, os; print(os.path.join(os.path.dirname(importlib.util.find_spec('rerun_cli').origin), 'rerun'))") && \
    missing=$(for b in /usr/local/bin/pelorus "$(command -v minot)" "$RERUN_BIN"; do ldd "$b" 2>/dev/null | grep "not found"; done | sort -u) && \
    if [ -n "$missing" ]; then echo "[build] missing libraries:"; echo "$missing"; exit 1; fi && \
    command -v pelorus && command -v minot && command -v rerun && command -v vulkaninfo && \
    python3 -c "import mcap.reader, rosbags.typesys, laspy, lazrs, yaml, numpy; \
from rosbags.typesys import Stores, get_typestore; ts = get_typestore(Stores.ROS2_HUMBLE); \
ts.types['sensor_msgs/msg/PointCloud2']; ts.types['nav_msgs/msg/Odometry']; print('[build] converter deps OK')" && \
    pelorus config verify /pelorus_ws/config/pelorus_bunker.rl && echo "[build] pelorus_bunker.rl is valid" && \
    test -f /pelorus_ws/scripts/pelorus_bench.py && test -f /pelorus_ws/scripts/pelorus_to_hdmapping.py && \
    python3 -c "import sys; sys.path.insert(0, '/pelorus_ws/scripts'); \
import livox_raw_to_mcap, pelorus_bench, pelorus_to_hdmapping; \
from rosbags.rosbag1 import Reader; from rosbags.rosbag2 import Writer, StoragePlugin; \
print('[build] bench scripts and the raw-bag converter import OK')"

CMD ["bash"]
