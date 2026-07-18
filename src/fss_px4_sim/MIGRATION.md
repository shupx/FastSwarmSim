# PX4/MAVROS Migration Boundary

`fss_px4_sim` currently provides a ROS 2 port of the original perfect MAVROS drone.

The full trimmed PX4 v1.13.3 dynamics stack from `sim_px4_drone/src/px4_rotor_sim` should be migrated in these next slices:

1. Move ROS-independent dynamics and PX4 module libraries unchanged, then compile them without ROS includes.
2. Replace the MAVROS shim plugin layer with ROS 2 publishers/subscribers/services while preserving relative topic names under `mavros/...`.
3. Replace the original global `hrt_absolute_time_us_sim` with an indexed time context owned by each simulated agent.
4. Use `fss_time::TimeParticipant` in the 100 Hz agent loop to announce each agent's next safe integration time.
5. Register the final simulator as an `rclcpp_components` component after the standalone executable works.
