# PX4/MAVROS Migration Boundary

This package is the ROS 2 destination for the ROS 1
`mavros_px4_quadrotor_sim` runtime.  The PX4 v1.13.3-derived modules and
quadrotor dynamics live under `src/px4_core`; the executable feeds them through
the same MAVLink receive/stream queues used by the original `mavros_sim`.

`MavrosSim` owns the six legacy plugin surfaces under the `mavros/...`
namespace: `setpoint_raw`, `local_position`, `imu`, `sys_status`, `command`,
and `global_position`. Their ROS 2 publishers, subscriptions, and services use
`mavros_msgs`. Callbacks only encode MAVLink frames through the bridge; the
PX4/dynamics state is mutated exclusively by the 100 Hz `fss_time::Rate` loop.

`px4_rotor_visualizer_node` is intentionally a separate executable.  It only
subscribes to MAVROS output and uses a ROS-time timer; it is not linked into the
PX4 simulation loop.

## PX4 Context Limitation

The copied PX4 v1.13.3 code still stores uORB, parameters, and MAVLink queues
in agent-indexed static containers. The provided launch files run one complete
simulator executable per UAV, which gives each vehicle an independent process
and therefore independent storage. A composed, multi-vehicle process is not a
supported execution mode until those PX4 internals are converted to a real
`Px4Context` instance type.
