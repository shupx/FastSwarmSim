# PX4/MAVROS Migration Boundary

This package is the ROS 2 destination for the ROS 1
`mavros_px4_quadrotor_sim` runtime.  The PX4 v1.13.3-derived modules and
quadrotor dynamics live under `src/px4_core`; the executable feeds them through
the same MAVLink receive/stream queues used by the original `mavros_sim`.

`MavrosSim` is now only a plugin owner and MAVLink dispatcher. The six legacy
surfaces are independent ROS 2 plugin classes under `src/mavros_sim/`:

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

Each class follows the endpoint names, message types, QoS direction, and frame
conversion conventions of its MAVROS ROS 2 counterpart, while retaining the
original simulator's MAVLink queue transport. Callbacks only encode MAVLink
frames through that bridge; the PX4/dynamics state is mutated exclusively by
the 100 Hz `fss_time::Rate` loop. As in the original `mavros_sim`, command
services do not wait for `COMMAND_ACK`: the copied PX4 stream set does not
provide that acknowledgement path, so service success means that the frame was
accepted by the simulator transport rather than accepted by PX4.

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
