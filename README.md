## Hint

Please change branch to [Bunker-DVI-Dataset-reg-1](https://github.com/MapsHD/benchmark-Pelorus-to-HDMapping/tree/Bunker-DVI-Dataset-reg-1) for quick experiment.

## Example Dataset:

Download the dataset from [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset/)

# benchmark-Pelorus-to-HDMapping

Runs [Pelorus](https://github.com/uos/pelorus) on a ROS 1 or ROS 2 bag and
converts the output to an [HDMapping](https://github.com/MapsHD/HDMapping)
session.

Pelorus (C. Sieh and M. Porrmann, "Pelorus: Resource-Aware Lidar-Inertial
Odometry", [arXiv:2611.22850](https://arxiv.org/abs/2611.22850), 2026) is a
LiDAR-inertial odometry written in Rust, from the Computer Engineering Group
at Osnabrück University. An iterated equivariant Kalman filter fuses the IMU
with plane-based scan-to-map registration against a voxel map of bounded size.

The submodule `src/pelorus` is pinned to the release tag `v0.1.0` (commit
`c8bad2e`, 2026-11-19). The canonical repository is
[codeberg.org/stelzo/pelorus](https://codeberg.org/stelzo/pelorus);
`github.com/uos/pelorus` is its mirror.

## Prerequisites

- Docker
- One of:
  - the raw ROS 1 bag `reg-1.bag`, with the Livox LiDAR as `CustomMsg` on
    `/livox/lidar` and its IMU on `/livox/imu`;
  - a ROS 2 bag directory, such as `reg-1-ros2` from the benchmark's step 1,
    with a `sensor_msgs/PointCloud2` topic and a `sensor_msgs/Imu` topic;
  - an `.mcap` file with those two topics.

Pelorus needs no ROS here: it runs in its offline mode, and its bag player
[Minot](https://minot.steado.tech/docs/) reads an MCAP bag directly. Pelorus
deskews the scans with a per-point time field, whose layout is chosen in the
config (`lidar.vendor`).

Before Pelorus starts, the run converts the input into an MCAP bag with only
the two topics Pelorus reads. The bag goes to
`<output_dir>/pelorus-input/<input>-mcap` (about 1.7 GB for reg-1), and later
runs reuse it as long as the input is unchanged:
- **`reg-1.bag`**: Pelorus cannot read Livox `CustomMsg`.
  [overlay/scripts/livox_raw_to_mcap.py](overlay/scripts/livox_raw_to_mcap.py)
  builds the `PointCloud2` clouds on `/livox/pointcloud` exactly as
  [livox_bag_aggregate](https://github.com/MapsHD/livox_bag_aggregate) builds
  `reg-1.bag-pc.bag` (0.1 s windows; fields x, y, z, intensity, tag, line,
  time) and copies `/livox/imu`. On reg-1 the clouds are identical to
  `reg-1.bag-pc.bag`'s (checked on the first 30 s: 290 of 290 clouds).
- **A ROS 2 bag stored as sqlite3 (`.db3`)**, like `reg-1-ros2`: Minot 0.15.2
  never gets Pelorus past its IMU initialization with such a bag, so no scan
  is ever played. The same data as MCAP, the format of the author's own
  examples, runs through. The two topics are copied with `rosbags-convert`,
  which changes no message.
- **An MCAP bag or file** is used as it is.

## Step 1 — Clone with submodules

```bash
git clone https://github.com/MapsHD/benchmark-Pelorus-to-HDMapping.git --recursive
cd benchmark-Pelorus-to-HDMapping
```

## Step 2 — Build the Docker image

```bash
docker build -t pelorus_standalone .
```

This builds:
- the Pelorus CLI from the submodule with Rust 1.100.0 and the author's
  `Cargo.lock`: the CPU build with the default features (offline mode, Rerun
  visualization, multi-core CPU). No GPU is needed: every stage of Pelorus
  runs on the CPU. The first build takes a while.
- on Ubuntu 24.04: Minot 0.15.2 (the version Pelorus 0.1.0 requires), the
  Rerun 0.43.2 viewer, and the HDMapping converter (mcap, rosbags, laspy).

The build ends with a check that Pelorus accepts the benchmark config.

## Step 3 — Run the pipeline

```bash
chmod +x docker_session_run-pelorus.sh
./docker_session_run-pelorus.sh /path/to/reg-1.bag /path/to/output/dir
# or a ROS 2 bag directory:
./docker_session_run-pelorus.sh /path/to/reg-1-ros2 /path/to/output/dir
```

With no arguments, a GUI file selector for the ROS 1 bag is used (requires
`zenity`).

| Variable | Default | Meaning |
|----------|---------|---------|
| `CONFIG_FILE` | none (`overlay/config/pelorus_bunker.rl`, in the image) | Pelorus config, see below |
| `VIS` | `1` | Rerun live viewer (off without a `DISPLAY`) |
| `LIBGL_SW` | `0` | `1` = the viewer always renders on the CPU; `0` = on the GPU when one is found |
| `POINT_SKIP` | `1` | the converter keeps every Nth point |
| `CONVERT_RESULTS_DIR` | none | an existing results directory in the container, e.g. `/output/pelorus-results/<timestamp>`: skip Pelorus, only convert |

**Config.** Pelorus reads one [Ratslang](https://ratslang.steado.tech/docs/)
file, and every option it leaves out keeps the author's default.
[overlay/config/pelorus_bunker.rl](overlay/config/pelorus_bunker.rl) sets only
what the Bunker DVI dataset requires:
- `lidar.topic = "/livox/pointcloud"`, `imu.topic = "/livox/imu"`;
- `lidar.vendor = Velodyne`. That cloud's points are x, y, z, intensity
  (float32) and `time` (float32, seconds after the header stamp), which is
  Pelorus' Velodyne point layout. The setting selects only how point times are
  read for deskewing. Pelorus' `LivoxMid360` layout expects the float64
  `timestamp` of livox_ros_driver2 clouds, which this bag does not have;
- `imu.from_lidar_transform`: the Mid-360's IMU position in the LiDAR frame.
  The default belongs to an Ouster sensor;
- `preprocessing.filter.distance.range = 0.5m..`: points closer than the
  Mid-360's blind zone (0.5 m, as in the benchmark's other Mid-360 setups) are
  dropped. The default `4m..` is meant to remove "the robot itself" on the
  author's platforms. reg-1 is indoors, and 91% of its points are closer than
  4 m. With the default, about 1,800 of ~20,000 points per scan were left, and
  the trajectory diverged after ~200 s.

The Mid-360 IMU reports acceleration in g, not m/s². Pelorus scales every
sample by 9.81 / (mean acceleration at start-up), so no conversion is needed.

Everything else is the author's default (registration, map, noise values). To
change an option, copy the config, add the option and pass the copy:

```bash
cp overlay/config/pelorus_bunker.rl my.rl   # edit my.rl
CONFIG_FILE=my.rl ./docker_session_run-pelorus.sh /path/to/reg-1.bag /path/to/output/dir
```

`docker run --rm pelorus_standalone pelorus config generate full` prints every
option with its documentation.

Before anything starts, the script checks that the bag holds the topics it
needs, with the right message types: `/livox/lidar` (Livox `CustomMsg`) and
the config's IMU topic in a ROS 1 bag, or the config's two topics in a ROS 2
bag.

**What happens:**

Pelorus runs offline: Minot releases the next scan only when Pelorus has
registered the previous one, so the run goes as fast as Pelorus computes, and
no scan is dropped. There is no playback in real time and no recording. The
log shows the progress every 15 seconds.

The run sets three output options on the command line, not in the config:
- the MCAP file of registered clouds and odometry;
- the TUM trajectory;
- `output.terminal.ui=false`. Pelorus' interactive terminal UI would end the
  run on `q`, `Esc` or `Ctrl+C`. The author's documentation says to turn it off
  for non-interactive sessions. It changes only the display.

With `VIS=1`, a [Rerun](https://rerun.io/) viewer opens in its own window. It
shows the current scan coloured by intensity and the trajectory as an orange
line. The time panel at the bottom scrubs back through the run. Closing the
viewer does not stop Pelorus.

**GPU.** The viewer uses a GPU when the run script finds one, and never needs
one:
- AMD or Intel graphics: the script passes `/dev/dri` into the container
  (Mesa's Vulkan and OpenGL drivers are in the image).
- NVIDIA with the
  [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html):
  the script adds `--runtime=nvidia` (or `--gpus all` when Docker does not list
  the nvidia runtime).
- Otherwise the viewer renders on the CPU, which is slower.

If Docker refuses to start the container with these options, the script
retries with fewer of them. Inside the container the viewer tries Vulkan, then
OpenGL, then software rendering (Mesa lavapipe, then llvmpipe). If none of
them starts, Pelorus runs without the viewer. The log names the Vulkan devices
the container sees; the viewer's own output goes to `logs/rerun-viewer.log`.

When Pelorus has finished, the same container converts its output into the
HDMapping session format.

## Step 4 — Open in HDMapping

Output files appear in `<output_dir>/output_hdmapping-pelorus/`:

```
lio_initial_poses.reg
poses.reg
scan_lio_0.laz
...
session.json
trajectory_lio_0.csv
...
```

Open `session.json` with the
[multi_view_tls_registration_step_2](https://github.com/MapsHD/HDMapping)
application.

Pelorus' own results stay in `<output_dir>/pelorus-results/<timestamp>/`
(`latest` links to the newest run):
- `pelorus.mcap`: `/registered` and `/odom` per scan, plus `/path` and `/tf`;
- `trajectory_tum.txt`;
- `pelorus.rl`: a copy of the config used.

The log is in `<output_dir>/logs/pelorus.log`. The converted input stays in
`<output_dir>/pelorus-input/` for the next run; delete it to free the space.

## Notes on Pelorus

**Output.** For every scan, Pelorus writes to the MCAP:
- its registered cloud on `/registered`: the full scan, deskewed, in the LiDAR
  frame;
- its odometry on `/odom`: the LiDAR pose in Pelorus' start frame `lio_init`.

Both carry the same stamp, the end of the scan. The converter pairs them by
that stamp and places each cloud with its pose, as a ROS node reading
`/odom` and `/registered` would. It then writes the chunked session. Points
closer than 0.5 m to the sensor are left out, as in the other converters.
Pelorus reports intensity as 0..1, and the LAZ files store it as 0..255.

**Timestamps.** The benchmark does not change Pelorus' clock. Every output
stamp is the bag time at the end of its scan: the header stamp of the input
cloud plus its last point time.

**Code.** Pelorus' code is not changed. Everything the benchmark sets is in
the config file and on the command line (Step 3).

## Contact

januszbedkowski@gmail.com
