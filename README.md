# FastSwarmSim

FastSwarmSim 是 `sim_px4_drone` 的 ROS 2 版本原型，用于轻量级多无人机仿真、MAVROS-like 接口测试、局部点云感知仿真，以及分布式逻辑时间同步实验。

当前版本已经包含：

- `fastswarm_time`: 分布式保守时间同步核心、`/clock` bridge、倍速/暂停控制接口。
- `fastswarm_px4_sim`: ROS 2 版理想 MAVROS 无人机节点，保留 `mavros/...` 话题和服务命名。
- `fastswarm_sensing`: ROS 2 版局部点云渲染节点。
- `marsim_render`: 点云渲染库和示例地图资源。
- `fastswarm_bringup`: 常用 launch 文件。

完整 trimmed PX4 控制器和动力学栈仍在迁移中；当前 `fastswarm_px4_sim` 先提供 perfect MAVROS drone 作为上层算法和时间同步的验证基线。

## 安装

推荐环境：Ubuntu 24.04 + ROS 2 Jazzy。

安装 ROS 2 后，安装构建工具和常用依赖：

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init 2>/dev/null || true
rosdep update
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
ros2 launch fastswarm_bringup distributed_clock.launch.py max_speed_ratio:=1.0
```

`max_speed_ratio` 含义：

- `1.0`: 按真实时间 1x 运行。
- `10.0`: 最高 10x 倍速运行。
- `0.0`: as-fast-as-safe，不按真实时间限速。

启动单架理想 MAVROS 无人机：

```bash
ros2 launch fastswarm_bringup perfect_drone.launch.py namespace:=uav1 init_z:=1.0
```

启动多架理想无人机：

```bash
ros2 launch fastswarm_bringup perfect_swarm.launch.py num_drones:=5
```

启动局部点云感知节点：

```bash
ros2 launch fastswarm_bringup sensing.launch.py
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
ros2 service call /fastswarm/clock_control fastswarm_time_interfaces/srv/SimClockControl "{proceed: false, max_sim_speed: 0.0}"
ros2 service call /fastswarm/clock_control fastswarm_time_interfaces/srv/SimClockControl "{proceed: true, max_sim_speed: 10.0}"
```

## 开发备注

构建测试：

```bash
colcon test
colcon test-result --verbose
```

完整 PX4/MAVROS dynamics 栈的迁移边界见：

```bash
src/fastswarm_px4_sim/MIGRATION.md
```
