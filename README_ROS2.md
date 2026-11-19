# SchurVINS for ROS 2 Humble

Native ROS 2 package `schurvins`, targeting Ubuntu 22.04 / ROS 2 Humble with C++17, OpenCV 4 and Eigen 3.4. The original algorithm, camera models and licenses are retained.

## Supported scope

- EuRoC-style **stereo + IMU** SchurVINS pipeline.
- Native `rclcpp`, ROS 2 parameters, exact-time stereo synchronization, sensor-data QoS, Python launch, RViz2.
- Timestamped world-to-IMU pose in `nav_msgs/Odometry`, bounded `nav_msgs/Path`, TF and optional TUM trajectory file.
- Image pairs are buffered until the IMU stream covers their timestamp. Processing and IMU insertion are serialized to avoid frontend/filter races.
- Invalid image dimensions/conversions and non-increasing sensor timestamps are dropped. Restart the node after rewinding or looping a bag.

The original ROS 1 subpackages remain in the repository as reference. The repository-root manifest is the single ROS 2 package; colcon does not traverse into those nested Catkin packages. The ROS 2 build excludes ROS 1 nodes/nodelets, rqt, legacy benchmarking scripts, Ceres/GTSAM backend, loop closure and global mapping. Attempts to enable `use_ceres_backend`, `runlc` or `use_global_map` fail explicitly. Monocular/array launch and full legacy visualization/message parity are not implemented.

## Dependencies and build

```bash
source /opt/ros/humble/setup.bash
sudo apt-get install python3-colcon-common-extensions git \
  libeigen3-dev libboost-dev libopencv-dev libyaml-cpp-dev libgoogle-glog-dev libgflags-dev \
  ros-humble-cv-bridge ros-humble-message-filters ros-humble-tf2-ros \
  ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-std-msgs \
  ros-humble-geometry-msgs ros-humble-launch-ros ros-humble-rviz2
cd /home/ros/ros_ws/schurvins_ws
MAKEFLAGS='-j2 -l2' colcon build --packages-select schurvins \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

On the first configure, CMake downloads FAST, minkindr and OpenGV at fixed Git revisions (see `cmake/Dependencies.cmake`). Their old Catkin build files are not used. A network connection to GitHub is required unless source directories are supplied explicitly:

```bash
MAKEFLAGS='-j2 -l2' colcon build --packages-select schurvins --cmake-args \
  -DFETCHCONTENT_SOURCE_DIR_FAST=/absolute/path/to/fast \
  -DFETCHCONTENT_SOURCE_DIR_MINKINDR=/absolute/path/to/minkindr \
  -DFETCHCONTENT_SOURCE_DIR_OPENGV_SRC=/absolute/path/to/opengv
```

Use an empty build directory if switching compilers or dependency locations. If another installed SDK overrides Eigen, select Ubuntu's Eigen with `-DEigen3_DIR=/usr/share/eigen3/cmake`. Limit parallelism on machines with little free RAM; OpenGV and Eigen-heavy filter code require substantial compiler memory. Source-only dependencies under the workspace but outside `src` should have a `COLCON_IGNORE` marker in their parent directory.

## Run

```bash
source /opt/ros/humble/setup.bash
source /home/ros/ros_ws/schurvins_ws/install/setup.bash
ros2 launch schurvins euroc_vio_stereo.launch.py rviz:=true
```

For ROS 2 bag playback, use simulation time and publish `/clock`:

```bash
ros2 launch schurvins euroc_vio_stereo.launch.py use_sim_time:=true rviz:=true
# In another sourced terminal:
ros2 bag play /absolute/path/to/ros2_bag --clock
```

The original EuRoC `.bag` examples are ROS 1 bags. Convert them to a ROS 2 bag or use a suitable bridge before playback; changing the launch command does not convert the bag format.

Custom topics/calibration and trajectory output:

```bash
ros2 launch schurvins euroc_vio_stereo.launch.py \
  calib_file:=/absolute/path/to/stereo_calibration.yaml \
  cam0_topic:=/camera/left/image_raw cam1_topic:=/camera/right/image_raw \
  imu_topic:=/imu/data trajectory_file:=/tmp/imu_trajectory.txt
```

The calibration retains the upstream camera-bundle YAML schema, including `T_B_C`, `imu_params` and `imu_initialization`. The images must match calibrated dimensions and have **identical timestamps** within each stereo pair. Input QoS is best-effort/volatile, compatible with sensor-data publishers and reliable bag publishers. No stereo rectification is applied by the adapter; use intrinsics/distortion corresponding to the supplied images.

Algorithm parameters are in `ros2/config/euroc_stereo.yaml`, using the ROS 2 `ros__parameters` envelope. Override with `params_file:=/absolute/path/to/params.yaml`. ROS 1 parameter keys such as `T_world_imuinit/qx` use ROS 2 dot notation, e.g. `T_world_imuinit.qx`. Parameters are read at startup; restart to apply changes.

| Topic | Type | Meaning |
|---|---|---|
| `/cam0/image_raw`, `/cam1/image_raw` | `sensor_msgs/msg/Image` | Default stereo inputs |
| `/imu0` | `sensor_msgs/msg/Imu` | Default angular velocity / acceleration input |
| `/svo/odometry` | `nav_msgs/msg/Odometry` | IMU/body pose in `world` |
| `/svo/path` | `nav_msgs/msg/Path` | At most `path_max_poses` poses (default 10000) |
| `/svo/tracked_image` | `sensor_msgs/msg/Image` | BGR stereo feature overlay, cam0 followed by cam1 |
| `/svo/cam0/tracked_image`, `/svo/cam1/tracked_image` | `sensor_msgs/msg/Image` | Per-camera feature overlays |
| `/svo/landmarks` | `sensor_msgs/msg/PointCloud2` | All historical XYZ/RGB points with unique `id`, in `world` |
| `/tf` | `tf2_msgs/msg/TFMessage` | `world` → `imu`, unless `publish_tf=false` |
| `/svo/remote_key` | `std_msgs/msg/String` | `r` reset/pause, `s` start, `q` quit, `c`/`C` enable/disable compensation |

`world_frame` and `body_frame` can be changed in the parameter file; adjust RViz's fixed frame accordingly. Odometry twist and estimator covariance are not exposed by the upstream public API. The adapter publishes pose only, leaves twist at its message default and sets covariance diagonal entries to `1e6` as an explicit placeholder. Do not use these covariance/twist fields as calibrated filter output. A trajectory file contains `timestamp tx ty tz qx qy qz qw` and is flushed on orderly shutdown. Create its parent directory first.

## Verification

```bash
sudo apt-get install ros-humble-ament-cmake-test python3-opencv python3-numpy python3-yaml
colcon test --packages-select schurvins --event-handlers console_direct+
colcon test-result --verbose
```

The integration test runs the native executable in an isolated ROS domain with a deterministic textured stereo plane projected through the supplied EuRoC calibration and stationary IMU measurements. It checks delayed-IMU buffering, visual tracking, finite normalized poses, increasing output timestamps, path/TF publication, invalid dimensions, duplicate timestamps, filter reset/start, trajectory-file output and clean shutdown. This verifies software integration, not real-world VIO accuracy or equivalence to the paper's results. Validate on real EuRoC/TUM-VI sequences before relying on estimated trajectories.

Local verification: the package was built in this workspace with GCC 12, Eigen 3.4 and OpenCV 4.5.4 (`Release`, `-O2 -DNDEBUG`). The synthetic integration test and installed launch startup/shutdown with `use_sim_time=true` passed. No real dataset trajectory accuracy evaluation has been run for this port.

## TideSense / TideDrive

已接入 `/home/ros/lib/tideSense/TideDrive` 的实时双目 + IMU 驱动，参考其 `scripts/openvio.sh`。直接运行：

```bash
/home/ros/lib/tideSense/TideDrive/scripts/schurvins.sh
# 无 RViz：末尾加 rviz:=false
# 记录轨迹：末尾加 trajectory_file:=/tmp/tidesense_schurvins.txt
```

脚本启动驱动、SchurVINS 和可选 RViz；先停止其他占用设备的采集。默认使用设备密码 `tidesense`，不再询问。设置 `TIDESENSE_PASSWORD` 可覆盖；显式设为空则使用已有 SSH 密钥。`SCHURVINS_WS` 可覆盖工作空间路径。

- `ros2/config/tidesense_calib.yaml`：实测 640×400 equidistant 双目标定、IMU 噪声与 204 Hz 频率。cam0 为物理右目，cam1 为物理左目，图像方向由驱动处理一次。标定从现有 OpenVIO 文件转换，原文件未修改。
- `ros2/config/tidesense_stereo.yaml`：SchurVINS 参数；鱼眼使用球面极线搜索和前端位姿残差。
- SchurVINS 的 `delay_imu_cam` 与 OpenVIO `rig.clock_offset` 符号相反。图像进入滤波器前使用 `t_imu = t_cam - delay_imu_cam` 修正时间戳，IMU handler 内部偏移随后置零。TideSense 修正为 -8.147591 ms；输出里程计、TF、轨迹也使用这个 IMU 时钟时间。
- IMU 内参校正只由 TideDrive 做一次；默认保留比例/轴间修正，关闭 gyro acceleration sensitivity `A`，与其 OpenVIO 启动脚本一致。SchurVINS 重力使用标定值。

重建转换后的标定：

```bash
python3 /home/ros/ros_ws/schurvins_ws/src/SchurVINS/ros2/scripts/import_tidesense_calibration.py   --source /home/ros/ros_ws/openvio/config/tidesense_stereo.yaml   --imu /home/ros/lib/tideSense/TideDrive/ros2/src/tidesense_ros2/config/imu.yaml   --output /home/ros/ros_ws/schurvins_ws/src/SchurVINS/ros2/config/tidesense_calib.yaml
# 重新 colcon build 安装更新后的配置。
```

`acc_max`、`omega_max` 保留 SVO 的通用默认值以满足旧标定读取器，当前 Schur 滤波器不使用它们；这些字段不是该设备的实测量程。初始速度/零偏先验同样为估计器默认值。

新增 `ros2_tidesense_imu_smoke` 测试以实测鱼眼内外参投影合成场景，验证非零时间偏移仅应用一次，并在复位前后共得到 42 个有限位姿。EuRoC 回归测试同时通过。联合 launch、有/无 RViz 开关和驱动现有测试共 11 项通过；本次尚未进行设备采集或真实轨迹精度评估。

## Feature and landmark visualization

The bundled RViz configurations now include **Stereo feature tracks (cam0 | cam1)** and **All historical landmarks (XYZRGB)**. Restart the combined TideDrive script after rebuilding so both the executable and RViz config are reloaded.

Image colors: green = landmark observation; yellow = unconverged seed; cyan = converged seed; red = outlier; blue/orange = other feature without an associated landmark. A line connects observations with the same landmark ID in adjacent processed frames. The overlay labels report landmark/seed counts independently for cam0 and cam1. Colors describe feature categories, not statistical confidence; the original algorithm can promote unconverged seeds to landmarks. Annotated images are published during initialization as well as tracking when frontend frames are available. The stereo composite preserves topic order; TideSense cam0 on its left is the physical right camera.

`/svo/landmarks` accumulates every finite estimated landmark seen in the current stereo bundle or retained local keyframes, keyed by point ID. Points remain after leaving the local map; their last finite position/color is retained. Existing IDs receive updated coordinates and the latest valid original-image color. Older keyframe observations do not overwrite newer colors; cam0 is preferred when timestamps tie. Only plain coordinate/color values are stored, so discarded keyframes and estimator points can still be freed. There is no count cap, downsampling or age-based expiry; memory and cloud size grow with historical point count.

The PointCloud2 fields are `x`, `y`, `z`, packed `rgb` and `id` (compatible with XYZRGB consumers that ignore the extra ID field). TideSense publishes MONO8: each point's unannotated source pixel is encoded as `R=G=B=gray`, not an artificial label color. A point without a valid pixel keeps its prior color; a never-colored point uses neutral gray 128. RViz uses 3-pixel Points with RGB8 color and Decay Time=0. The publisher is Reliable / Transient Local / KeepLast(1), so a late RViz subscriber receives the complete historical snapshot. RViz does not need to accumulate old messages itself. Untriangulated seeds are not fabricated into 3D points.

History spans one estimator/map coordinate frame. Manual reset, automatic reinitialization or node restart clears it and publishes an empty cloud; retaining pre-reset points under a new origin would mix incompatible coordinates. Adjacent-frame image tracks are also cleared on reset.

Visualization snapshots share the depth-filter worker's mutex for seed-array initialization/updates, then release it before drawing or publishing. The image publishers use Reliable / Volatile / KeepLast(2) and only send images with connected subscribers. Coordinates and images use the same corrected timestamps as the estimator.

Both EuRoC and TideSense integration tests check colored stereo/per-camera overlays, image dimensions/timestamps, historical ID retention, coordinate/color updates, actual grayscale RGB, finite unique landmark IDs, late-subscriber cloud delivery and reset/recovery, in addition to pose/TF checks. No new real-device accuracy claim is made by adding visualization.
