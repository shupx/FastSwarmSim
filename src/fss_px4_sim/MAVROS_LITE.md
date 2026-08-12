# mavros_lite

`mavros_lite_node` is a small MAVROS-compatible ROS 2 bridge for the local PX4
simulator. It binds one loopback UDP port and learns the PX4 source endpoint
from the first MAVLink packet. It does not use the MAVROS UAS, router, or
pluginlib runtime.

The node exposes the standard topic and service names below its namespace for
the `setpoint_raw`, `local_position`, `imu`, `sys_status`, `command`, and
`global_position` modules. The existing PX4 launch starts it at `/mavros`.

Parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `udp.bind_port` | `24540` | Loopback port receiving PX4 MAVLink packets |
| `udp.remote_host` | `127.0.0.1` | Fixed remote address; loopback only |
| `udp.remote_port` | `0` | Fixed PX4 port, or `0` to learn it |
| `target_system` | `1` | Accepted PX4 MAVLink system id |
| `target_component` | `1` | Target component for outgoing messages |
| `system_id` | `1` | MAVLink system id used by the bridge |
| `component_id` | `191` | MAVLink component id used by the bridge |
| `command.ack_timeout` | `5.0` | Seconds to wait for `COMMAND_ACK` |

Standalone example:

```bash
ros2 run fss_px4_sim mavros_lite_node --ros-args \
  -r __ns:=/mavros -p udp.bind_port:=24540 -p target_system:=1
```
