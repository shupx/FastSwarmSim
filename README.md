# FastSwarmSim

FastSwarmSim is the ROS 2 successor workspace for `sim_px4_drone`.

The first migrated slice contains:

- `fastswarm_time_interfaces`: ROS 2 messages and services for distributed logical time.
- `fastswarm_time`: a conservative distributed simulated-time participant and `/clock` bridge.
- `fastswarm_sensing`: ROS 2 wrapper for local point cloud rendering with `marsim_render`.
- `marsim_render`: ROS-independent renderer copied from the original project and built with `ament_cmake`.
- `fastswarm_bringup`: launch files for the time bridge and sensing node.
- `fastswarm_px4_sim`: migration boundary package for the PX4/MAVROS simulator.

Build:

```bash
cd FastSwarmSim
colcon build --symlink-install
source install/setup.bash
```

Run the distributed clock bridge:

```bash
ros2 launch fastswarm_bringup distributed_clock.launch.py
```

Run the sensing wrapper:

```bash
ros2 launch fastswarm_bringup sensing.launch.py
```
