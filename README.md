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

[安装 ROS 2](https://gitee.com/shu-peixuan/install_ros2) 后，安装构建工具和常用依赖。`colcon` 是必需工具：

```bash
sudo apt update
sudo apt install -y curl software-properties-common python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

如果 `sudo rosdep init` 因为无法访问 `raw.githubusercontent.com` 失败，可以改用国内 rosdistro 镜像。以下命令会手动创建 rosdep 源列表，并把 `rosdep update` 的索引切到中科大镜像：

```bash
sudo mkdir -p /etc/ros/rosdep/sources.list.d/
sudo curl -o /etc/ros/rosdep/sources.list.d/20-default.list \
  https://mirrors.ustc.edu.cn/rosdistro/rosdep/sources.list.d/20-default.list
sudo sed -i \
  's#raw.githubusercontent.com/ros/rosdistro/master#mirrors.ustc.edu.cn/rosdistro#g' \
  /etc/ros/rosdep/sources.list.d/20-default.list
echo 'export ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml' >> ~/.bashrc
export ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml
rosdep update
```

`fss_time` 使用 eCAL 作为唯一分布式时间通信层。eCAL 在本机多进程场景使用共享内存，在局域网场景可使用 UDP multicast；没有 eCAL SDK 时 `fss_time` 不会构建。

安装 eCAL：

```bash
sudo add-apt-repository ppa:ecal/ecal-latest
sudo apt update
sudo apt install -y ecal libprotobuf-dev protobuf-compiler
```

拉取并构建：

```bash
git clone git@gitee.com:shu-peixuan/FastSwarmSim.git
cd FastSwarmSim
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

如果使用的是 Ubuntu 22.04 + ROS 2 Humble，整体结构相同，但部分依赖包名可能需要按本机 ROS 发行版调整。

## 启动

启动分布式仿真时钟：

```bash
ros2 launch fss_bringup distributed_clock.launch.py max_speed_ratio:=1.0
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

跨机器运行 eCAL transport 时，需要在每台机器的 eCAL 配置中打开 network mode，例如 `/etc/ecal/ecal.yaml`：

```yaml
communication_mode: "network"
```

本机多进程不需要打开 network mode，eCAL 默认使用本机通信并按 `shm, udp, tcp` 优先级选择传输层。局域网多机启用 network mode 后，eCAL 默认按 `udp, tcp` 优先级通信，UDP multicast 默认组为 `239.0.0.1`、端口为 `14002`。请确认所有机器位于同一 multicast 可达网络，防火墙允许 eCAL 的 UDP multicast/registration 流量。

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

只验证 `fss_time` 和 eCAL transport：

```bash
scripts/verify_fss_time_ecal.sh
```

`fss_time` 的 eCAL transport 细节见：

```bash
docs/fss_time_ecal.md
```

完整 PX4/MAVROS dynamics 栈的迁移边界见：

```bash
src/fss_px4_sim/MIGRATION.md
```

eCAL 参考文档：

- 安装：<https://eclipse-ecal.github.io/ecal/stable/getting_started/setup.html>
- Transport layers：<https://eclipse-ecal.github.io/ecal/stable/advanced/transport_layers.html>
