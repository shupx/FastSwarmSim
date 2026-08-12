# mavros_lite

`mavros_lite_node` is a small MAVROS-compatible ROS 2 bridge for the local PX4
simulator. It binds one loopback UDP port and learns the PX4 source endpoint
from the first MAVLink packet. It does not use the MAVROS UAS, router, or
pluginlib runtime.

The implementation is exported as the `fss_mavros_lite` shared library.
`MavrosLite` is a transport-free ROS/MAVLink adapter that is constructed with
an existing `rclcpp::Node`. It adds its publishers, subscriptions, services,
and timers to that host node; constructing it never opens a socket, starts a
receiver thread, or creates a second ROS node. It can be fed already-decoded
`mavlink_message_t` values by another program.
`MavlinkUdpTransport` is a separate optional class in the
`fss_mavros_lite_udp` library, used by the standalone node. PX4 direct ROS
transport links only `fss_mavros_lite`.

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

PX4 simulator launch transport:

```bash
# Existing behavior: PX4 and a separate mavros_lite process communicate by UDP.
ros2 launch fss_px4_sim px4_rotor_sim_single.launch.py px4.mavlink_transport:=udp

# In-process behavior: PX4 owns mavros_lite and exchanges decoded messages.
ros2 launch fss_px4_sim px4_rotor_sim_single.launch.py \
  px4.mavlink_transport:=direct_ros
```

PX4 accepts `px4.mavlink_transport` values `udp`, `direct_ros`, and `empty`.
In `direct_ros` mode the UDP port parameters are unused and MAVROS Lite is
embedded in the PX4 process. In `empty` mode PX4 does not create its MAVLink
module. For UDP, `mavros_node_type:=official` selects the installed MAVROS node
while `mavros_node_type:=lite` selects `mavros_lite_node`. The launch-level
`enable_mavros_node` flag defaults to false and only requests starting that
external node. Neither argument modifies PX4 transport parameters. Enabling an
external node with `direct_ros` is a configuration error because that transport
already embeds MAVROS Lite. With `empty`, launch warns that no external node
can be started.

The official MAVROS branch is allowlisted to the same six plugins. Its
`sys_status` plugin requests `AUTOPILOT_VERSION`; the reduced PX4 simulator does
not currently implement MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, so official
MAVROS logs a timeout and falls back to default capabilities.

PX4 uses the same separation internally: `MAVLINK` owns the receiver, streamer,
transport selection, and direct ROS bridge, while `Px4MavlinkUdpTransport`
exclusively owns the PX4 UDP socket, byte encoding/parsing, and receive thread.

Embedding example:

```cpp
#include <fss_px4_sim/mavros_lite/core.hpp>

fss_px4_sim::mavros_lite::MavrosLite::Config config;
auto lite = std::make_shared<fss_px4_sim::mavros_lite::MavrosLite>(*node, config);

lite->set_send_callback([](const mavlink_message_t &message) {
  // Deliver an outgoing ROS-generated MAVLink message to the flight stack.
});
lite->receive_message(message_from_flight_stack);
```

Add `lite` to the application's executor. Calls to `receive_message()` dispatch
to the six Lite modules, while their outgoing messages are delivered through
the callback installed by `set_send_callback()`.
