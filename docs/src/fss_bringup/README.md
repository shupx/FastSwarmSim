# fss_bringup

This package provides integrated FastSwarmSim launch files. Each launch combines the PX4 or perfect-drone simulator, optional local LiDAR point-cloud sensing, the `fss_time` coordinator, vehicle visualization, and RViz.

```bash
# One PX4 vehicle with LiDAR sensing and RViz.
ros2 launch fss_bringup sim_px4_drone_lidar_single.launch.py

# A three-vehicle PX4 swarm with LiDAR sensing and RViz.
ros2 launch fss_bringup sim_px4_drone_lidar_multi.launch.py num_drones:=3

# Use MAVROS-compatible perfect drones instead of PX4 simulation.
ros2 launch fss_bringup sim_px4_drone_lidar_multi.launch.py \
  num_drones:=5 use_perfect_drone:=true
```

Both launches use the FastSwarmSim coordinated clock by default. Set `use_fss_sim_time:=false` to disable lock-step time coordination, `sensing_enable:=false` to omit local LiDAR in the multi-drone launch, or `enable_sensing:=false` in the single-drone launch. Use `rviz_enable:=false` for multi-drone runs or `enable_rviz:=false` for single-drone runs to launch without RViz.

The multi-drone launch assigns the namespaces `uav1`, `uav2`, and so on. It accepts `drones_per_row`, `startup_batch_size`, and `startup_batch_delay_ms` to control formation layout and staged startup.
