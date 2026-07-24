# fss_time HELICS Time Coordination

`fss_time` uses HELICS as the distributed simulation-time authority. Each
`thread_time_participant` joins the same HELICS federation, asynchronously
requests its next safe simulation time, and updates local granted time when
HELICS completes the request.

The public ROS-facing surface is now:

- `/clock` is published by `ros_clock_publisher_node`.
- `/fss/clock_control` is served by `sim_time_broker_node`.
- `/fss/sim_clock_status` publishes broker state.

## Dependencies

`fss_time` vendors HELICS source at `src/fss_time/third_party/HELICS` and builds
it by default. It does not use a system HELICS install, but it does use system
packages for `fmt`, `spdlog`, and ZeroMQ to reduce build time. Install those
packages before building:

```bash
sudo apt update
sudo apt install -y cmake libfmt-dev libspdlog-dev libzmq3-dev
cmake --version  # requires 3.22 or newer
```

Ubuntu 22.04 packages for `toml11` and `nlohmann_json` are not API-compatible
with HELICS 3.6.1, so those remain vendored with the HELICS source.

## Local Host

The distributed clock launch starts the broker and clock publisher by default:

```bash
ros2 launch fss_bringup distributed_clock.launch.py
ros2 launch fss_bringup perfect_swarm.launch.py num_drones:=5
```

Default HELICS settings:

```bash
helics_core_type:=zmq
helics_broker_address:=127.0.0.1
helics_broker_port:=23404
helics_time_delta_ns:=1000000
start_helics_broker:=true
```

`max_real_time_factor` keeps the same meaning:

- `1.0`: run at real-time speed.
- `10.0`: cap simulation time at 10x wall time.
- `0.0`: request time as fast as HELICS can safely grant it.

## External or Multi-Host Broker

Start a broker yourself, then disable launch-managed broker startup and point
all participants at the same broker:

```bash
helics_broker -t zmq --port 23404 --terminate_on_disconnect
ros2 launch fss_bringup distributed_clock.launch.py \
  start_helics_broker:=false \
  helics_broker_address:=<broker-host> \
  helics_broker_port:=23404
ros2 launch fss_bringup perfect_drone.launch.py \
  helics_broker_address:=<broker-host> \
  helics_broker_port:=23404
```

## Verification

```bash
scripts/verify_fss_time_helics.sh
```

The script checks that the vendored HELICS source is present, builds
`fss_time`, and runs the HELICS time-coordination tests.
