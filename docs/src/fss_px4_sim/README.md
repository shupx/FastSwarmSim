# fss_px4_sim

This package provides a ROS 2 PX4 simulator runtime. 

```bash
# Launch a single drone simulation with PX4 SITL and MAVROS Lite bridge.
ros2 launch fss_px4_sim px4_rotor_sim_single.launch.py

# Launch a multi-drone simulation.
ros2 launch fss_px4_sim px4_rotor_sim_multi.launch.py num_drones:=5

# Launch perfect MAVROS-compatible drones (No PX4 controller and dynamics, perfect tracking MAVROS commands).
ros2 launch fss_px4_sim perfect_mavros_drone_single.launch.py
ros2 launch fss_px4_sim perfect_mavros_drone_multi.launch.py num_drones:=5

# Only launch the drone visualizer node, useful for real experiments.
ros2 launch fss_px4_sim drone_visualizer.launch.py
ros2 launch fss_px4_sim drone_visualizer.launch.py num_drones:=5

# Terminal keyboard control (start PX4/MAVROS Lite first)
ros2 run fss_px4_sim keyboard_control_ros2 --ros-args -r __ns:=/mavros

```

## What it contains:

1. A tailored PX4 v1.13.3 flight controll stack. The position controller, attitude controller, state machine, and MAVLink interface are included.  Although it is based on the PX4 v1.13.3 source code, the core logic remains the same across different PX4 versions, and it is expected to be compatible with most recent PX4 versions (v1.15+).

2. A MAVROS Lite bridge to expose the PX4 MAVLink interface as ROS 2 topics and services. This bridge is embedded in the PX4 simulator process by default so that no extra mavros process is needed. The bridge can also be run as a separate process, and the PX4 simulator and the MAVROS Lite bridge communicate through a loopback UDP port.

3. AN ideal dynamic model for the quadrotor, which accepts the desired angular rates and thrust from the PX4 simulator and outputs the drone's position, velocity, attitude, and angular rates. The dynamics is integrated using odeint. The integration step size is 0.01s (100Hz).

4. (Optional) A drone visualizer node process receives MAVROS topics and publishes the drone's URDF, history trajectory, and name markers for visualization in RViz2. 

In conclusion, the conventional PX4 SITL process + MAVROS process + drone dynamics process is simplified to a single PX4 simulator process, which is more efficient. A step of simulation (10ms) only costs about 0.1ms in real time, so the simulation can run at most 100x real time speed on a normal laptop.

## Terminal keyboard control

`keyboard_control_ros2` is a standard-terminal `rclpy` node; it does not use ROS 1 or PyQt5. Run it in a separate terminal after starting a simulator with MAVROS Lite:

```bash
ros2 run fss_px4_sim keyboard_control_ros2 --ros-args -r __ns:=/mavros

# For the first vehicle in a multi-drone launch, use its vehicle namespace.
ros2 run fss_px4_sim keyboard_control_ros2 --ros-args -r __ns:=/uav1/mavros
```

| Key | Action |
| --- | --- |
| `W/A/S/D` | forward/left/back/right velocity |
| `Q/E` | increase/decrease altitude setpoint |
| `Z/C` | yaw left/right |
| `Space` or `R` | zero all velocities (hold position) |
| `+` / `-` | adjust translation speed by 0.1 m/s |
| `T` | arm via `/mavros/cmd/arming` |
| `O` | request `OFFBOARD` via `/mavros/set_mode` after arming |
| `X` | stop and disarm |
| `Ctrl-C` | stop, request disarm, and restore terminal |

The node starts streaming neutral setpoints at 20 Hz, then requires the explicit safe sequence `T`, wait for `armed=True`, then `O`; it refuses an OFFBOARD request while state reports disarmed. On every normal exit it publishes a neutral setpoint and waits up to one second for a disarm request before restoring the terminal. It publishes `mavros_msgs/msg/PositionTarget` to `setpoint_raw/local`, subscribes to `mavros_msgs/msg/State` on `state`, and uses the MAVROS-compatible `mavros_msgs/srv/CommandBool` and `SetMode` services. Set `--speed`, `--yaw-speed`, or `--initial-altitude` to change defaults. Always verify the vehicle is in a safe simulation state before arming.

The keyboard controller is intended for an interactive terminal. The following
is a verified ROS 2 node terminal capture; it demonstrates node startup and the
keyboard status display, but is not a substitute for checking the simulated
vehicle state before arming:

![Keyboard control terminal](../../misc/fss_keyboard_control_terminal.png)

## What it does not simulate:

1. The attitude rate controller of the PX4 is not included on purpose, because simulating the attitude rate controller is not necessary for most applications, and it will slow down the simulation speed. 
The output of the PX4 simulator is thus the desired angular rates and thrust, assuming that the attitude rate is ideally responded by the drone dynamic model. 

2. The full PX4 modes such as MISSION, POSCTL, STABLIZE, etc. are not included. The PX4 simulator only supports the OFFBOARD mode, which is sufficient for most applications. 

3. The PX4 simulator does not simulate the sensors such as IMU, barometer, GPS, etc. Therefore, the EKF module is also not included. The pose of the drone is directly provided by the drone dynamics model. And we provide two local pose source modes: (1) mocap/vision mode (default), where the pose in /mavros/local_position/pose is relative to the global map, i.e., the world frame; (2) gps mode, where the pose in /mavros/local_position/pose is relative to the initial position of the drone.

4. The uXRCE DDS and zenoh transports are not included so far. The only transport support is the MAVLINK over mavros_lite or UDP. Since mavlink is a common protocol not only for PX4 and ROS2, but also for Ardupilot and ROS1, while DDS and zenoh are only compatible with PX4 and ROS2 now. Adding the support for DDS and zenoh is possible in the future, but it is not a priority now.

## Accelerate multi-drone simulation using fss_time time coordinator

The simulation loop uses the `fss_time::Rate` instead of `rclcpp::Rate`, so it is compatible with the `fss_time` time coordinator, which controls the simulation clock for multiple drones using a synchronous approach (lock step). The simulation can thus run at a maximum speed of 100x real time for one drone, 20x real time for 10 drones, and 5x real time for 100 drones on a normal laptop. 

- To enable fss sim time, set the `use_fss_sim_time` launch argument to `true` when launching the simulation. The `use_sim_time` parameter should also be set to `true` automatically by the launch file.

- If `use_fss_sim_time` is set to `false`, the simulation will use the default ROS2 time, i.e., the /clock time if `use_sim_time = true` and wall time if `use_sim_time = false`. The /clock topic can be published by other processes, but the lock step simulation will not be supported. 

Since fss_time time coordinator supports cascade clock synchronization across multiple machines, **the simulation can also run on multiple machines**, with each machine simulating a subset of drones. The simulation clock is synchronized across all machines, and the maximum speed of the simulation is limited by the slowest machine.

## Trouble shooting

### 100+ drone simulation

Simulating 100+ drones is possible, but it requires slight modification as the default ROS2 FastRTPS transport has a limit of 100 participants. Solutions:

- Set the `mutation_tries` parameter of the FastRTPS transport to a value larger than the number of nodes, e.g., 10000. Create `large_scale_configuration.xml` according to [ros2 tutorials](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#large-number-of-participants):

  ```xml
  <?xml version="1.0" encoding="UTF-8"?>
  <dds xmlns="http://www.eprosima.com">
    <profiles>
      <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
          <builtin>
            <mutation_tries>10000</mutation_tries>
          </builtin>
        </rtps>
      </participant>
    </profiles>
  </dds>
  ```

  Then enable the profile before starting any ROS 2 nodes:

  ```bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/large_scale_configuration.xml # old ROS2 humble
  export FASTDDS_DEFAULT_PROFILES_FILE=/path/to/large_scale_configuration.xml
  ros2 daemon stop # latest ROS2 version
  ```

- Use the `rmw_cyclonedds_cpp` transport instead of the default `rmw_fastrtps_cpp`. CycloneDDS has no participant limit, and launches much more faster:

  ```bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ros2 daemon stop
  ```
