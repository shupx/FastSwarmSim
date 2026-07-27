# fss_time ZeroMQ Time Coordination

`fss_time` now uses a local ZeroMQ ROUTER/DEALER channel instead of HELICS.

- `sim_time_broker` binds a ROUTER socket to `sim_time_broker_endpoint` and publishes ROS `/clock`.
- `thread_time_participant` creates one DEALER participant per thread.
- Participant construction waits until the broker acknowledges registration.
- `announce_next_safe_time()` sends a time request and waits for broker acknowledgement.
- `unregister_participant()` unregisters explicitly; thread-local destruction also unregisters.
- `auto_start` controls whether the broker starts in running or paused state.
- `speed_regulator_step_ns` is the fixed simulation-time step emitted by the speed regulator.
- For bounded RTF, regulator requests are emitted every `speed_regulator_step_ns / max_real_time_factor` wall-time nanoseconds.

Default endpoint:

```bash
ipc:///tmp/fss_time_broker.ipc
```

Launch:

```bash
ros2 launch fss_time sim_time_broker.launch.py
```
