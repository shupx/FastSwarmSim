# FastSwarmSim

FastSwarmSim 是 `sim_px4_drone` 的 ROS 2 版本原型，用于轻量级多无人机仿真、MAVROS-like 接口测试、局部点云感知仿真，以及分布式逻辑时间同步实验。

当前版本已经包含：

- `fss_time`: 分布式保守时间同步核心、`/clock` bridge、倍速/暂停控制接口。
- `fss_px4_sim`: ROS 2 版理想 MAVROS 无人机节点，保留 `mavros/...` 话题和服务命名。
- `fss_sensing`: ROS 2 版局部点云渲染节点。
- `marsim_render`: 点云渲染库和示例地图资源。
- `fss_bringup`: 常用 launch 文件。

完整 trimmed PX4 控制器和动力学栈仍在迁移中；当前 `fss_px4_sim` 先提供 perfect MAVROS drone 作为上层算法和时间同步的验证基线。

## 安装

推荐环境：Ubuntu 22.04 + ROS 2 Humble。

[安装 ROS 2, colcon, rosdep](https://gitee.com/shu-peixuan/install_ros2) （网络问题解决方法见链接）

```bash
sudo apt update
sudo apt install -y curl software-properties-common python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

`fss_time` 使用 HELICS 作为分布式仿真时间协调层。每个仿真参与者作为 HELICS federate 请求下一次安全仿真时间，只有在 HELICS grant 后才推进本地仿真时间。HELICS 源码已经 vendored 在 `src/fss_time/third_party/HELICS`，默认会随 `fss_time` 一起编译。

`fss_time` 只使用仓库内的 HELICS 源码，不查找系统安装的 HELICS。HELICS 构建中已改用系统 apt 包提供的 `fmt`、`spdlog` 和 ZeroMQ：

```bash
sudo apt update
sudo apt install -y cmake libfmt-dev libspdlog-dev libzmq3-dev
cmake --version  # 需要 3.22 或更新版本
```

说明：Ubuntu 22.04 apt 中虽然有 `libtoml11-dev` 和 `nlohmann-json3-dev`，但版本/API 与 HELICS 3.6.1 不兼容；`toml11`、`nlohmann_json` 以及 HELICS/GMLC 自有的 `networking`、`concurrency`、`containers`、`utilities`、`units` 仍保留在 `src/fss_time/third_party/HELICS/ThirdParty`。已禁用或由系统包替代的较大子依赖只保留 license 文件。

拉取并构建：

```bash
git clone git@gitee.com:shu-peixuan/FastSwarmSim.git
cd FastSwarmSim
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

如果使用的是 Ubuntu 24.04 + ROS 2 Jazzy，整体结构相同，但部分依赖包名可能需要按本机 ROS 发行版调整。

## 启动

启动分布式仿真时钟：

```bash
ros2 launch fss_bringup distributed_clock.launch.py max_speed_ratio:=1.0
```

`distributed_clock.launch.py` 默认启动本机 HELICS broker：

```bash
ros2 launch fss_bringup distributed_clock.launch.py \
  helics_core_type:=zmq \
  helics_broker_address:=127.0.0.1 \
  helics_broker_port:=23404 \
  start_helics_broker:=true
```

`max_speed_ratio` 含义：

- `1.0`: 按真实时间 1x 运行。
- `10.0`: 最高 10x 倍速运行。
- `0.0`: as-fast-as-safe，不按真实时间限速。

启动单架理想 MAVROS 无人机：

```bash
ros2 launch fss_bringup perfect_drone.launch.py namespace:=uav1 init_z:=1.0
```

启动多架理想无人机：

```bash
ros2 launch fss_bringup perfect_swarm.launch.py num_drones:=5
```

跨机器运行时，可以手动启动一个外部 HELICS broker，并让每台机器上的 clock bridge 和仿真参与者连接同一个 broker：

```bash
helics_broker -t zmq --port 23404 --terminate_on_disconnect
ros2 launch fss_bringup distributed_clock.launch.py \
  start_helics_broker:=false \
  helics_broker_address:=<broker-host> \
  helics_broker_port:=23404
ros2 launch fss_bringup perfect_drone.launch.py \
  helics_broker_address:=<broker-host> \
  helics_broker_port:=23404
```

启动局部点云感知节点：

```bash
ros2 launch fss_bringup sensing.launch.py
```

常用话题示例：

```bash
ros2 topic echo /clock
ros2 topic echo /uav1/mavros/state
ros2 topic echo /uav1/mavros/local_position/pose
ros2 topic echo /cloud_registered
```

向理想无人机发送位置 setpoint：

```bash
ros2 topic pub /uav1/mavros/setpoint_raw/local mavros_msgs/msg/PositionTarget "{
  coordinate_frame: 1,
  type_mask: 3576,
  position: {x: 1.0, y: 2.0, z: 1.5},
  yaw: 0.0
}"
```

暂停或恢复仿真时钟：

```bash
ros2 service call /fss/clock_control fss_time_interfaces/srv/SimClockControl "{proceed: false, max_sim_speed: 0.0}"
ros2 service call /fss/clock_control fss_time_interfaces/srv/SimClockControl "{proceed: true, max_sim_speed: 10.0}"
```

## 开发备注

构建测试：

```bash
colcon test
colcon test-result --verbose
```

只验证 `fss_time` 和 HELICS time coordination：

```bash
scripts/verify_fss_time_helics.sh
```

`fss_time` 的 HELICS time coordination 细节见：

```bash
docs/fss_time_helics.md
```

完整 PX4/MAVROS dynamics 栈的迁移边界见：

```bash
src/fss_px4_sim/MIGRATION.md
```

HELICS 参考文档：

- Timing configuration：<https://docs.helics.org/en/latest/user-guide/fundamental_topics/timing_configuration.html>
- Linking：<https://docs.helics.org/en/latest/user-guide/installation/linking.html>
