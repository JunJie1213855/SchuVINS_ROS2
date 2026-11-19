# ROS 2 Humble 详细说明

安装、快速启动与话题总览见 [README.md](README.md)。本文件补充构建方式、标定约定、可视化语义和排查说明。原 ROS 1 文档见 [README_ROS1.md](README_ROS1.md)。

## 构建结构与离线依赖

仓库根目录是单个 `ament_cmake` 包 `schurvins`，C++17。原 ROS 1 子包的 CMake/package.xml 保留为参考，ROS 2 构建不使用这些 Catkin 入口。CPU 视觉前端、相机模型、SchurVINS 滤波器和 ROS 2 wrapper 由根目录 CMake 直接构建。

首次配置通过 [Dependencies.cmake](cmake/Dependencies.cmake) 下载固定 revision 的 FAST、minkindr、OpenGV。离线构建可提供对应源码目录：

```bash
source /opt/ros/humble/setup.bash
cd "$SCHURVINS_WS"
MAKEFLAGS='-j2' colcon build --packages-select schurvins --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DFETCHCONTENT_SOURCE_DIR_FAST=/path/to/fast \
  -DFETCHCONTENT_SOURCE_DIR_MINKINDR=/path/to/minkindr \
  -DFETCHCONTENT_SOURCE_DIR_OPENGV_SRC=/path/to/opengv
```

这些目录应对应 CMake 中固定的版本。位于工作空间内的依赖源码应放在 `src` 之外，并在其父目录加入 `COLCON_IGNORE`，避免被 colcon 当作旧 ROS 1 包扫描。

切换编译器或依赖位置时使用新的 build/install 目录。如果其他 SDK 覆盖了 Eigen，可通过 `-DEigen3_DIR=/usr/share/eigen3/cmake` 选择 Ubuntu Eigen。OpenGV 和滤波器的 Eigen 模板编译占用较多内存，建议限制构建并行度。

默认启用测试目标。仅构建节点时可在 `--cmake-args` 后加入 `-DBUILD_TESTING=OFF`。已完成的本机验证使用 GCC 12、Eigen 3.4、OpenCV 4.5.4，Release 编译时使用 `-O2 -DNDEBUG`。

## 参数与标定约定

- `calib_file` 使用 SVO camera-bundle YAML：两组相机内参/畸变、`T_B_C`、IMU 噪声和初始化参数。
- `T_B_C` 表示 camera → body/IMU 变换，不能直接填入方向相反的矩阵。
- 参数文件使用 ROS 2 `ros__parameters` envelope；原来的 `T_world_imuinit/qx` 等斜杠键改成 `T_world_imuinit.qx`。
- 参数在启动时读取，不支持运行时自动重配；修改后重启节点。
- 适配器将输入转换为 MONO8，不执行双目校正；应使用与输入方向、尺寸和畸变模型一致的标定。
- 两目使用 ExactTime 同步，时间戳必须完全一致。`sync_queue_size` 控制同步队列，`image_queue_size` 控制等待 IMU 的图像对队列，默认均为 30。
- 图像/IMU 输入使用 Best Effort / Volatile，兼容传感器数据发布器及 Reliable 发布器。
- 输入尺寸或图像转换失败时丢弃该帧；非递增时间戳也会被丢弃。回放倒带或循环需重启节点。

适配器先将相机帧转到 IMU 时钟：

```text
t_frame_imu = t_camera - delay_imu_cam
```

然后将内部 IMU handler 的偏移置零，避免采样提取或旋转先验重复补偿。SchurVINS 前向积分使用修正后的帧时间，输出位姿、TF、轨迹和特征图也使用该时间。零偏移的 EuRoC 输入不改变帧时间。

图像仅在 IMU 缓冲覆盖其时间、且存在更早的 IMU 样本时进入算法。ROS 回调采用单线程执行器串行处理图像和 IMU 插入；深度滤波器自身可运行工作线程。

## 轨迹、坐标系与复位

`world_frame` 默认 `world`，`body_frame` 默认 `imu`；两者必须非空且不同。修改后也需同步 RViz 的 Fixed Frame 和 Target Frame。

`trajectory_file` 可保存 TUM 格式：

```text
timestamp tx ty tz qx qy qz qw
```

父目录需预先存在，文件在正常退出时刷新。`path_max_poses` 默认 10000，仅限制轨迹消息中的位姿数，不限制历史 landmarks 数量。

发送远程命令示例：

```bash
ros2 topic pub --once /svo/remote_key std_msgs/msg/String '{data: r}'
ros2 topic pub --once /svo/remote_key std_msgs/msg/String '{data: s}'
```

`r` 重建前端和滤波器并暂停处理图像，`s` 恢复处理，`q` 退出；`c`/`C` 启用/禁用仿射补偿。自动重新初始化同样重建滤波器，不仅重置前端地图。

当前 `Odometry` 只导出 pose；twist 为消息默认值，pose/twist 协方差对角线 `1e6` 是占位值。适配器未通过上游公共 API 获得真实速度和滤波器协方差。

## 特征图

默认 RViz 显示 `/svo/tracked_image`。拼接图保留 cam0、cam1 顺序，不翻转或交换图像；TideSense 的 cam0 是物理右目，因此在拼接图左半边。

两目独立图像为 `/svo/cam0/tracked_image` 和 `/svo/cam1/tracked_image`，均为 BGR8，Reliable / Volatile / KeepLast(2)。只有存在订阅者时才发送图像。

| 颜色 | 特征类别 |
|---|---|
| 绿色 | 地图点观测 |
| 黄色 | 未收敛种子 |
| 青色 | 已收敛种子 |
| 红色 | 异常点 |
| 蓝橙色 | 其他未关联地图点的特征 |

短线连接同一地图点 ID 在相邻处理帧中的像素位置。图中分别标注两目的 landmark/seed 数量。类别颜色不代表统计置信度：原算法可能将未收敛种子提升为地图点。初始化阶段有前端帧时也可以发布特征图，不必等待完整跟踪成功。

## 全部历史 XYZRGB 点云

`/svo/landmarks` 使用 `sensor_msgs/msg/PointCloud2`，字段为 `x/y/z/rgb/id`；忽略额外 `id` 字段即可由 XYZRGB 消费者读取。

- 从当前双目帧和局部关键帧中收集有限的已估计三维点，按 ID 去重。
- 已有 ID 更新到最新有限坐标，离开局部地图后保留最后的坐标与颜色。
- 不按数量、时间截断或降采样，内存及完整点云消息大小随历史点数增长。
- 颜色来自未叠加标记的原图像素。当前输入为 MONO8，因此 `R=G=B=gray`；这不是伪彩色。
- 更新的有效像素优先于旧关键帧观测；同一时间优先 cam0。无有效像素时保留旧颜色，尚无颜色的新点使用灰度 128。黑色像素 0 也是有效颜色。
- 仅保存坐标与颜色数值，不持有已移除关键帧或算法点对象。未三角化的深度种子不会被虚构成三维点。
- 手动复位、自动重新初始化或节点重启时清空历史，因为新的地图原点不能直接与旧坐标混合。

发布器使用 Reliable / Transient Local / KeepLast(1)，每次发布完整快照，晚打开 RViz 也可收到全部历史点。RViz 配置为 Points、3 pixels、Position Transformer=`XYZ`、Color Transformer=`RGB8`、Decay Time=0，无需用 RViz 的消息累积来拼历史地图。

可视化复制特征与地图点时，与深度滤波器工作线程的种子初始化/更新共享互斥锁。复制完成后释放锁，再绘图及发布，避免复制过程中读取被扩容的特征数组。

## TideSense 标定与时间

示例文件：[tidesense_calib.yaml](ros2/config/tidesense_calib.yaml)、[tidesense_stereo.yaml](ros2/config/tidesense_stereo.yaml)。联合设备入口由独立 TideDrive 项目提供，使用方法见 [主 README](README.md#tidesense-实时运行)。

标定来自现有设备的 OpenVIO 配置：

- 640×400 equidistant，cam0=物理右目，cam1=物理左目，保持驱动已处理的图像方向。
- OpenVIO 的 `imu_from_camera` / `imu_from_camera1` 直接对应 SVO 的 `T_B_C`。
- OpenVIO：`t_imu = t_cam + rig.clock_offset`；SVO：`t_imu = t_cam - delay_imu_cam`，两者符号相反。
- 示例 `clock_offset=-0.008147591034761545` 转换为 `delay_imu_cam=+0.008147591034761545`，在适配器中只修正一次。
- 驱动负责所有传感器共用的设备到主机时间平移；这不替代相机—IMU 标定偏移，也不等于跨主机精密时钟同步。
- 驱动只做一次 IMU 比例/轴间校正，默认关闭陀螺加速度灵敏度 `A` 项。SchurVINS 读取相同标定的噪声、随机游走与重力，不重复做 IMU 内参校正。

转换工具使用 Python/PyYAML，并核对相机 rig 配置与 IMU 文件的噪声参数一致：

```bash
cd "$SCHURVINS_WS/src/SchurVINS"
python3 ros2/scripts/import_tidesense_calibration.py \
  --source /path/to/openvio/config/tidesense_stereo.yaml \
  --imu /path/to/TideDrive/ros2/src/tidesense_ros2/config/imu.yaml \
  --output ros2/config/tidesense_calib.yaml

cd "$SCHURVINS_WS"
MAKEFLAGS='-j2' colcon build --packages-select schurvins
```

输出文件记录源标定的 SHA256。提供的标定只适用于相应设备与图像约定。`acc_max`、`omega_max` 为满足旧读取器而保留的 SVO 默认值，当前 Schur 滤波器不使用它们，不能视为该设备的实测量程；初始速度/零偏先验同样为估计器默认值。

## 测试与实机排查

本仓库的 3 项测试分别为历史点存储、EuRoC 合成集成测试、TideSense 鱼眼合成集成测试。测试命令见 [主 README](README.md#测试与验证边界)。

测试覆盖历史 ID 保留、坐标/颜色更新、有限坐标过滤、特征叠加、时间戳与非零偏移、晚订阅、复位恢复、位姿/轨迹/TF 和正常退出。合成测试使用确定性纹理平面与静止 IMU，不代替真实轨迹精度评估。已完成的软件测试通过；实机连接过程中观察到过低匹配和深度未收敛问题，需要结合场景、图像质量与标定继续排查。

| 日志/现象 | 含义与检查方向 |
|---|---|
| `Not enough matched features` | 匹配数低于 `quality_min_fts`，默认 40；结合特征图检查覆盖、模糊与纹理 |
| `matches are unconverged seeds` | 匹配中未收敛深度种子占比较高，不等于整个地图都未收敛 |
| `updated seeds are unconverged` | 本次被提升为地图点的种子中未收敛点的比例 |
| `obs is from too far away` | 新旧观测方向不相容；日志数值为方向余弦，不是米数，位姿/点深度错误也会触发 |
| `Downsizing storage not implemented` | 申请容量小于已有容量时保留原存储并打印日志，该分支没有中止程序 |
| 图像队列满/缺少 IMU | 检查话题、时间戳、偏移和处理/回放速率 |

不应仅通过降低质量阈值掩盖持续退化。当前分支未完成实机精度评估；原论文和原 ROS 1 上游的复现说明见 [历史文档](README_ROS1.md)。
