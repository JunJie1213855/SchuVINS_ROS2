# SchurVINS · ROS 2 Humble

基于 [ByteDance SchurVINS](https://github.com/bytedance/SchurVINS) 的 ROS 2 Humble 移植版本，面向 Ubuntu 22.04，支持双目相机 + IMU、EuRoC 数据输入和 TideSense 实时接入。

SchurVINS 基于 SVO 2.0，通过 Schur 补分解观测模型，并使用 EKF 估计运动状态和地图点。本分支保留原算法及许可证，新增原生 ROS 2 构建、节点、参数、launch 与可视化。原论文的精度结论不等同于本移植版本的实机验证结果。

## 功能与范围

- 单个 `ament_cmake` 包：`schurvins`，使用 `colcon` 构建。
- 双目精确时间同步、IMU 缓冲与相机—IMU 时间补偿。
- 发布位姿、轨迹、TF，可保存 TUM 格式轨迹。
- 双目拼接特征跟踪图，以及独立的两路特征叠加图。
- **全部历史 XYZRGB landmarks**：按 ID 更新坐标与原图颜色，离开局部地图后仍保留。
- TideSense 640×400 equidistant 双目标定与参数示例。
- 历史点存储单元测试，以及 EuRoC / TideSense 合成双目 + IMU 集成测试。

当前 ROS 2 入口支持**双目 + IMU**。单目/多相机入口、ROS 1 nodelet/rqt、Ceres/GTSAM 后端、回环和全局建图尚未移植；启用 `use_ceres_backend`、`runlc` 或 `use_global_map` 会明确报错。历史点云是可视化记录，不代表新增了全局建图或回环功能。

## 环境与依赖

目标环境：Ubuntu 22.04、ROS 2 Humble、C++17、OpenCV 4、Eigen 3.4。本机验证使用 GCC 12 和 OpenCV 4.5.4。

安装 ROS 2 Humble 后，安装构建与测试依赖：

```bash
sudo apt-get install git python3-colcon-common-extensions \
  libeigen3-dev libboost-dev libopencv-dev libyaml-cpp-dev \
  libgoogle-glog-dev libgflags-dev \
  ros-humble-ament-cmake ros-humble-ament-cmake-test \
  ros-humble-rclcpp ros-humble-rclpy ros-humble-cv-bridge \
  ros-humble-message-filters ros-humble-tf2-ros ros-humble-tf2-msgs \
  ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-std-msgs \
  ros-humble-geometry-msgs ros-humble-launch-ros ros-humble-rviz2 \
  python3-opencv python3-numpy python3-yaml
```

## 构建

将 `YOUR_REPOSITORY_URL` 替换为本 ROS 2 分支的仓库地址：

```bash
export SCHURVINS_WS="$HOME/schurvins_ws"
mkdir -p "$SCHURVINS_WS/src"
git clone YOUR_REPOSITORY_URL "$SCHURVINS_WS/src/SchurVINS"

source /opt/ros/humble/setup.bash
cd "$SCHURVINS_WS"
MAKEFLAGS='-j2' colcon build --packages-select schurvins \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

首次配置会通过 CMake FetchContent 下载固定 Git revision 的 FAST、minkindr 和 OpenGV，具体版本见 [Dependencies.cmake](cmake/Dependencies.cmake)。这些依赖使用源码直接构建，不使用其旧 Catkin 构建文件。ROS 2 构建无需执行旧版 `dependencies.yaml` 的 `vcs-import`。

原 ROS 1 子包仍保留在源码中，但仓库根目录的 `package.xml` 是 ROS 2 构建入口，colcon 将整个仓库识别为单个包。离线依赖路径、Eigen 路径及构建说明见 [README_ROS2.md](README_ROS2.md)。

## EuRoC / 自定义双目输入

在已 source 工作空间的终端启动：

```bash
ros2 launch schurvins euroc_vio_stereo.launch.py rviz:=true
```

默认输入为 `/cam0/image_raw`、`/cam1/image_raw` 和 `/imu0`。两目图像时间戳必须完全一致，图像尺寸、方向与畸变模型必须匹配标定。

播放 ROS 2 bag：

```bash
# 终端 1
ros2 launch schurvins euroc_vio_stereo.launch.py use_sim_time:=true rviz:=true

# 终端 2：同样 source ROS 2 与工作空间
ros2 bag play /path/to/ros2_bag --clock
```

原始 EuRoC `.bag` 是 ROS 1 格式，需要先转换为 ROS 2 bag 或通过桥接输入。回放倒带或循环时需重启估计器；节点丢弃不递增的传感器时间戳。

自定义标定、话题与轨迹输出：

```bash
ros2 launch schurvins euroc_vio_stereo.launch.py \
  calib_file:=/path/to/stereo_calibration.yaml \
  params_file:=/path/to/algorithm_parameters.yaml \
  cam0_topic:=/camera/left/image_raw \
  cam1_topic:=/camera/right/image_raw \
  imu_topic:=/imu/data \
  trajectory_file:=/tmp/imu_trajectory.txt \
  rviz:=true
```

标定使用原 SVO camera-bundle YAML 格式，包含 `cameras`、`T_B_C`、`imu_params` 和 `imu_initialization`，示例见 [EuRoC 标定](svo_ros/param/calib/euroc_stereo.yaml)。算法参数使用 ROS 2 `ros__parameters` 格式，示例见 [euroc_stereo.yaml](ros2/config/euroc_stereo.yaml)。轨迹输出格式为 `timestamp tx ty tz qx qy qz qw`，父目录需预先存在。

## TideSense 实时运行

仓库提供 [TideSense 标定](ros2/config/tidesense_calib.yaml) 与 [算法参数](ros2/config/tidesense_stereo.yaml)。这些参数来自当前设备的实测标定，其他设备、分辨率或图像方向需要匹配的标定。

TideSense 的约定：

| 项目 | 约定 |
|---|---|
| 图像 | 640×400 MONO8，equidistant 畸变 |
| `/cam0/image_raw` | 物理右目，方向已由驱动处理 |
| `/cam1/image_raw` | 物理左目，方向已由驱动处理 |
| 时间戳 | 两目共用 cam0 时间戳；相机约 20.4 Hz、IMU 约 204 Hz |
| 时间补偿 | `t_imu = t_cam - 0.008147591034761545 s`，仅应用一次 |
| IMU 校正 | 由 TideDrive 做一次比例/轴间校正，默认关闭 gyro acceleration sensitivity `A` 项 |

**TideDrive 是独立的设备驱动项目，不包含在本仓库中。** 已安装 TideDrive 并编译其 ROS 2 wrapper 后，可以使用其联合入口：

```bash
# 按实际安装位置设置路径
export TIDEDRIVE_ROOT="$HOME/lib/tideSense/TideDrive"
export SCHURVINS_WS="$HOME/schurvins_ws"

"$TIDEDRIVE_ROOT/scripts/schurvins.sh"
# 不启动 RViz：
"$TIDEDRIVE_ROOT/scripts/schurvins.sh" rviz:=false
# 保存轨迹：
"$TIDEDRIVE_ROOT/scripts/schurvins.sh" trajectory_file:=/tmp/tidesense_schurvins.txt
```

联合脚本启动设备驱动、SchurVINS 和可选 RViz，使用 TideDrive 自身的设备凭据配置；可通过 `TIDESENSE_PASSWORD` 覆盖，显式设为空时使用 SSH 密钥。切换前停止其他占用设备的采集，不同时运行 OpenVIO、另一份驱动或 bag 回放。

如果已通过其他方式发布相同的 ROS 2 话题，也可仅启动本仓库的估计器：

```bash
schurvins_share="$(ros2 pkg prefix schurvins)/share/schurvins"
ros2 launch schurvins euroc_vio_stereo.launch.py \
  calib_file:="$schurvins_share/config/tidesense_calib.yaml" \
  params_file:="$schurvins_share/config/tidesense_stereo.yaml" \
  rviz:=true
```

实机采集使用 `use_sim_time=false`。驱动需保持原设备顺序和图像方向，不再交换或翻转图像；IMU 内参和时间偏移也不要重复校正。标定转换方法见 [ROS 2 详细说明](README_ROS2.md#tidesense-标定与时间)。

## RViz 与话题

默认 RViz 包含双目特征图、全部历史点云、轨迹和 TF，固定坐标系为 `world`。

| 话题 | 类型 | 内容 |
|---|---|---|
| `/svo/odometry` | `nav_msgs/msg/Odometry` | IMU/body 在世界坐标系的位姿 |
| `/svo/path` | `nav_msgs/msg/Path` | 最多保留 `path_max_poses` 个轨迹位姿，默认 10000 |
| `/svo/tracked_image` | `sensor_msgs/msg/Image` | 双目特征图，左半图 cam0，右半图 cam1 |
| `/svo/cam0/tracked_image` | `sensor_msgs/msg/Image` | cam0 特征叠加图 |
| `/svo/cam1/tracked_image` | `sensor_msgs/msg/Image` | cam1 特征叠加图 |
| `/svo/landmarks` | `sensor_msgs/msg/PointCloud2` | 全部历史点，字段 `x/y/z/rgb/id` |
| `/tf` | `tf2_msgs/msg/TFMessage` | 默认 `world` → `imu` |
| `/svo/remote_key` | `std_msgs/msg/String` | `r` 复位并暂停，`s` 恢复，`q` 退出，`c/C` 开/关补偿 |

特征图颜色：绿色为地图点观测，黄色为未收敛种子，青色为已收敛种子，红色为异常点。短线连接同一地图点在相邻处理帧的像素位置；颜色是特征类别，不代表统计置信度。

历史点云按 ID 去重，更新已有点的坐标和最新有效颜色，离开局部地图后仍保留，不按数量或时间截断。TideSense 为黑白相机，因此颜色来自未叠加特征的原图灰度，**R=G=B=gray**；没有有效像素的新点暂用灰度 128。点云内存和消息大小随累计点数增长。

历史属于同一地图坐标系：手动复位、自动重新初始化或节点重启时清空，避免混合重置原点前后的坐标。历史点云使用 Reliable / Transient Local / KeepLast(1)，晚打开 RViz 也能收到完整快照。

手动添加 RViz 显示时，Image 使用 Reliable / Volatile；PointCloud2 选择 `/svo/landmarks`，Position Transformer=`XYZ`、Color Transformer=`RGB8`、Style=`Points`、Size=3 pixels、Decay Time=0。

## 测试与验证边界

```bash
cd "$SCHURVINS_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select schurvins --event-handlers console_direct+
colcon test-result --verbose
```

当前包含 3 项测试：

1. 历史点存储：ID 更新、离开局部地图后的保留、灰度颜色更新/优先级、非法坐标过滤及复位。
2. EuRoC 合成双目 + IMU：跟踪、位姿、轨迹、TF、图像叠加和点云。
3. TideSense 鱼眼合成双目 + IMU：非零时间补偿、灰度 XYZRGB 历史点、晚订阅、异常图像及复位恢复。

上述构建与测试已在开发环境通过；也已完成 TideSense 实机话题接入，但实机运行中观察到过低特征匹配和深度未收敛告警，**尚未完成实机轨迹精度评估，也未验证与论文指标等价**。

当前 ROS 2 适配器未导出滤波器真实速度和协方差：`Odometry.twist` 保持消息默认值，pose/twist 协方差对角线使用 `1e6` 占位，不应当作已标定的滤波器输出使用。更多实现与排查说明见 [README_ROS2.md](README_ROS2.md)。

## 目录

```text
SchurVINS/
├── CMakeLists.txt / package.xml   # ROS 2 单包构建入口
├── cmake/Dependencies.cmake       # 固定版本源码依赖
├── ros2/
│   ├── src/                      # ROS 2 节点、参数工厂与可视化
│   ├── include/schurvins/        # 历史点存储及节点辅助接口
│   ├── launch/                   # ROS 2 launch
│   ├── config/                   # EuRoC/TideSense 参数、标定和 RViz
│   ├── scripts/                  # TideSense 标定转换工具
│   └── test/                     # 历史点单元测试和合成集成测试
├── svo/、svo_common/、svo_direct/  # 原算法和视觉前端
├── vikit/                        # 相机与数学工具
├── README_ROS2.md                # ROS 2 详细说明
└── README_ROS1.md                # 上游 ROS 1 历史说明
```

## 上游、许可证与引用

原项目：[bytedance/SchurVINS](https://github.com/bytedance/SchurVINS)。本项目基于 [rpg_svo_pro_open](https://github.com/uzh-rpg/rpg_svo_pro_open)，保留相关源码中的原始版权与许可证声明，项目许可证见 [LICENSE](LICENSE)。第三方依赖保留各自许可证，安装时复制到 `share/schurvins/licenses/`。

原论文：Yunfei Fan, Tianyu Zhao, Guidong Wang. *SchurVINS: Schur Complement-Based Lightweight Visual Inertial Navigation System*. CVPR 2024, pp. 17964–17973。

```bibtex
@inproceedings{fan2024schurvins,
  title={SchurVINS: Schur Complement-Based Lightweight Visual Inertial Navigation System},
  author={Fan, Yunfei and Zhao, Tianyu and Wang, Guidong},
  booktitle={Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition},
  pages={17964--17973},
  year={2024}
}
```

使用本代码开展研究时，请同时遵循原 SchurVINS 与 SVO 2.0 的引用要求。原 ROS 1 安装、复现说明与上游原始介绍保存在 [README_ROS1.md](README_ROS1.md)。
