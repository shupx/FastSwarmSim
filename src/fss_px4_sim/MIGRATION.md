# PX4/MAVROS Migration Boundary

This package is the ROS 2 destination for the ROS 1
`mavros_px4_quadrotor_sim` runtime.  The PX4 v1.13.3-derived modules and
quadrotor dynamics live under `src/px4_core`; the executable feeds them through
the PX4-core MAVLink UDP receiver and streamer.

The official ROS 2 `mavros_node` owns the six plugins directly; launch files allowlist precisely the six
surfaces implemented by the ROS 1 simulator:

- `setpoint_raw.cpp`: `SET_POSITION_TARGET_{LOCAL_NED,GLOBAL_INT}` and
  `SET_ATTITUDE_TARGET`, plus the three target telemetry topics.
- `local_position.cpp`: `LOCAL_POSITION_NED` and `LOCAL_POSITION_NED_COV`.
- `imu.cpp`: `ATTITUDE`, `ATTITUDE_QUATERNION`, `HIGHRES_IMU`, `RAW_IMU`,
  `SCALED_IMU`, and `SCALED_PRESSURE`.
- `system_status.cpp`: `HEARTBEAT`, `EXTENDED_SYS_STATE`, `SYS_STATUS`,
  `STATUSTEXT`, and `ESTIMATOR_STATUS`.
- `command.cpp`: the command services implemented by the original simulator.
- `global_position.cpp`: `GPS_RAW_INT`, `GLOBAL_POSITION_INT`,
  `GPS_GLOBAL_ORIGIN`, and `LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET`.

The runtime uses the installed/overlay MAVROS plugin library. `MAVLINK` owns
the loopback UDP FCU endpoint, receiver, streamer, and polling receive thread;
each stream packet is sent directly through that endpoint. PX4/dynamics state
is updated by the 100 Hz `fss_time::Rate` loop.

The original command plugin explicitly skipped `COMMAND_ACK` waiting because
the copied PX4 stream set has no `COMMAND_ACK`. The official ROS 2 command
plugin keeps its normal ACK behavior by default; the FSS launch enables its
documented `fss_simulate_no_command_ack` compatibility option so command
services retain the original simulator's immediate transport-accepted result.

The full-simulator launch entries start an official MAVROS node named
`mavros`; its public API is therefore `/mavros/...` for no-prefix launch and
`/<uav namespace>/mavros/...` for namespaced launch. Each vehicle has an
isolated loopback UDP port pair, which is an internal transport detail rather
than a public simulator API.

`px4_rotor_visualizer_node` is intentionally a separate executable.  It only
subscribes to MAVROS output and uses a ROS-time timer; it is not linked into the
PX4 simulation loop.

## PX4 Context Limitation

The copied PX4 v1.13.3 code still stores uORB and parameters
in agent-indexed static containers. The provided launch files run one complete
simulator executable per UAV, which gives each vehicle an independent process
and therefore independent storage. A composed, multi-vehicle process is not a
supported execution mode until those PX4 internals are converted to a real
`Px4Context` instance type.
