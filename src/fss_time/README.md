# fss_time ZeroMQ Time Coordination

`fss_time` now uses a local ZeroMQ ROUTER/DEALER channel instead of HELICS.

- `time_coordinator` binds a ROUTER socket to `fss_time_coordinator_endpoint` and publishes ROS `/clock`.
- `time_coordinator` also binds a PUB socket to `fss_time_coordinator_pub_endpoint` and sends
  ZeroMQ `GRANT <time_ns>` messages whenever it publishes `/clock`.
- A child coordinator can set `fss_time_parent_coordinator_endpoint` to a parent coordinator's
  ROUTER endpoint. It requests the parent's PUB endpoint with `GET_PUB_ENDPOINT`, subscribes to
  parent grants, and announces its local next safe time to the parent before publishing local
  `/clock`.
- `thread_time_participant` creates one DEALER participant per thread.
- Participant construction waits until the coordinator acknowledges registration.
- `announce_next_safe_time()` sends a time request and waits for coordinator acknowledgement.
- `unregister_participant()` unregisters explicitly; thread-local destruction also unregisters.
- `auto_start` controls whether the coordinator starts in running or paused state.
- `speed_regulator_step_ns` is the fixed simulation-time step emitted by the speed regulator.
- `follows_real_time` controls whether the coordinator applies the real-time catch-up floor to
  participant requests. It defaults to `true`.
- A separate real-time progress timer also runs every `speed_regulator_step_ns` wall-time
  nanoseconds and advances by the measured wall-time elapsed between ticks.
- `speed_regulator_step_ns` and the regulator wall period are clamped to at least
  `min_operation_walltime` (`100000` ns).
- For positive RTF, regulator requests are emitted every
  `max(speed_regulator_step_ns / max_real_time_factor, min_operation_walltime)` wall-time
  nanoseconds.
- `max_real_time_factor <= 0` disables periodic regulator requests instead of running unbounded.

Default endpoint:

```bash
ipc:///tmp/fss_time_coordinator.ipc
```

Default PUB endpoint:

```bash
ipc:///tmp/fss_time_coordinator_pub.ipc
```

Launch:

```bash
ros2 launch fss_time time_coordinator.launch.py
```
