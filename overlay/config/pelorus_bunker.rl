# Pelorus configuration for the Bunker DVI dataset (Livox Mid-360 LiDAR and its
# built-in IMU), Ratslang format. Only what the sensor and the bag require is
# set here; every other option keeps the author's default (see
# `pelorus config generate full`), including the registration, the filters and
# the noise values. The run script adds the output paths on the command line.

lidar {
  # PointCloud2 topic of reg-1-ros2, made by livox_bag_aggregate from the
  # Livox CustomMsg scans of reg-1.bag (100 ms per cloud).
  topic = "/livox/pointcloud"

  # Per-point layout of that cloud: x, y, z, intensity (float32) and `time`
  # (float32, seconds after the header stamp, which is the first point of the
  # cloud). That is exactly Pelorus' Velodyne layout, so this selects only how
  # point times are read for deskewing. Pelorus' LivoxMid360 setting expects the
  # float64 `timestamp` field of livox_ros_driver2 clouds, which this bag lacks.
  vendor = Velodyne
}

imu {
  # The Mid-360's built-in IMU. Its acceleration is in g, not m/s^2; Pelorus
  # scales every sample by 9.81 / (mean static acceleration), so no conversion
  # is needed.
  topic = "/livox/imu"

  from_lidar_transform {
    # Lidar frame -> IMU frame of the Mid-360 (Livox: the IMU sits at
    # 11.0, 23.29, -44.12 mm in the point cloud frame, axes aligned). The
    # default belongs to an Ouster sensor.
    translation = [-11.0mm, -23.29mm, 44.12mm]
    rotation = [ [1.0, 0.0, 0.0],
                 [0.0, 1.0, 0.0],
                 [0.0, 0.0, 1.0] ]
  }
}
