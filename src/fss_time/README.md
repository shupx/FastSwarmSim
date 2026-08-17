# fss_time

This package provides conservative lock-step simulation time for ROS 2 applications in FastSwarmSim. A ZeroMQ-based time coordinator grants the shared simulation clock only after every participating thread has announced the next time at which it is safe for the clock to advance. This makes simulation results deterministic with respect to simulation time while allowing real-time or accelerated execution.

```bash
# Launch a local coordinator, simulation clock, and control UI.
ros2 launch fss_time time_coordinator.launch.py

# Limit the simulation to 2x real time.
ros2 launch fss_time time_coordinator.launch.py max_real_time_factor:=2.0

# Run with a high speed limit (bounded by available CPU and participant work).
ros2 launch fss_time time_coordinator.launch.py max_real_time_factor:=1000.0

# Start without the UI.
ros2 launch fss_time time_coordinator.launch.py start_coordinator_ui:=false
```

## What it contains:

1. A time coordinator that publishes `/clock`, receives participant safe-time announcements over ZeroMQ, and grants a shared simulation time. The coordinator also provides `fss/clock_control` to pause/resume the clock and set `max_real_time_factor` at runtime.

2. Thread time participants. Each participating thread announces the earliest future simulation time that it can safely permit; its announcement automatically registers the thread and is removed when the thread exits.

3. `fss_time::Rate`, `fss_time::sleep_for`, and `fss_time::sleep_until` helpers. They use the node clock and automatically announce the next safe time when `use_fss_sim_time` is enabled.

4. `fss_time::executors::SingleThreadedExecutor` and `fss_time::executors::MultiThreadedExecutor`, plus the convenience `fss_time::spin(node)`. These executors make ROS callbacks participate in lock-step execution.

5. Cascaded coordinators for multi-machine simulation. A child coordinator announces its local safe time to a parent coordinator and republishes grants as its local `/clock`.

## Launch and clock parameters

`time_coordinator.launch.py` starts a root coordinator. By default it uses a local IPC endpoint:

```text
ipc:///tmp/fss_time_coordinator.ipc
```

Important launch arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `max_real_time_factor` | `1.0` | Maximum simulation-time to wall-time ratio. `2.0` allows up to 2x; use a large positive value for the highest practical speed. |
| `speed_regulator_step_ns` | `10000000` | Regulator step (10 ms). Set it no greater than the smallest desired simulation step; larger values reduce coordinator overhead. |
| `auto_start` | `true` | Start advancing the clock immediately; set `false` to start paused. |
| `follows_real_time` | `true` | Enable real-time catch-up behavior; see Advanced usage. |
| `publish_clock` | `true` | Publish `/clock`; parent coordinators normally set this to `false`. |
| `fss_time_coordinator_endpoint` | namespace-derived IPC endpoint | ZeroMQ ROUTER endpoint used by local participants. |

To use `fss_time` with application nodes, set both parameters in their launch scope:

```python
from launch_ros.actions import SetParameter

SetParameter(name="use_fss_sim_time", value=True)
SetParameter(name="use_sim_time", value=True)
```

`use_sim_time` makes the node clock follow the coordinator's `/clock`. `use_fss_sim_time` enables the `fss_time` announce-and-request behavior in its helpers and executors. They are intentionally separate: setting only `use_sim_time` uses `/clock` but does not make the application a lock-step participant; setting only `use_fss_sim_time` leaves ROS time inactive and causes the helpers to wait. The package launch files set both together, and `use_fss_sim_time` is normally enabled by default.

Set the endpoint too when it differs from the default or when running on another machine:

```python
SetParameter(
    name="fss_time_coordinator_endpoint",
    value="tcp://coordinator-host:5545",
)
```

## How lock-step time works

```text
  ROS timer / loop / callback                     ROS timer / loop / callback
  +--------------------------+                    +--------------------------+
  | thread participant A     |                    | thread participant B     |
  | announce next safe time  |                    | announce next safe time  |
  +------------+-------------+                    +-------------+------------+
               | ZeroMQ ANNOUNCE tA                             | ZeroMQ ANNOUNCE tB
               '---------------------------. .------------------'
                                            v v
                              +-----------------------------------+
                              |         Time coordinator          |
                              | target = earliest safe request    |
                              | + speed/real-time constraints     |
                              +----------------+------------------+
                                               |
                                               | grant t, publish /clock
                        . . . . . . . . . . .  v  . . . . . . . . . . .
                                               |
                       +-----------------------+-----------------------+
                       | Nodes use their ROS clock at the same time t  |
                       +------------------------------------------------+
```

For a normal finite step, the coordinator waits until every participating thread has made a new announcement, then advances to the minimum requested safe time, subject to the configured speed regulator. A thread that is waiting for ROS work does not constrain progress: an `fss_time` executor announces infinite safe time while it waits. Before executing a ready callback, the executor announces the current simulation time and therefore holds the clock during that callback.

`fss_time::Rate` and `fss_time::sleep_*` announce the time at which their calling thread will resume before sleeping on the node's ROS clock. The coordinator can consequently request the next grant proactively instead of waiting for wall time to elapse.

## Converting a ROS 2 node

Include the public header:

```cpp
#include "fss_time/fss_time.hpp"
```

Use the following replacements wherever simulated time controls the behavior. The original wall-time APIs still work, but those call sites do **not** announce a next safe time and therefore do not participate in lock-step coordination.

| Wall-time or generic code | Lock-step `fss_time` code |
| --- | --- |
| `rclcpp::Clock(RCL_SYSTEM_TIME).now()` or a standalone wall clock | `this->now()` or `node->now()` |
| `create_wall_timer(period, cb)` | `rclcpp::create_timer(node, node->get_clock(), period, cb)` |
| `std::this_thread::sleep_for(duration)` | `fss_time::sleep_for(*node, rclcpp::Duration::from_nanoseconds(...))` |
| `std::this_thread::sleep_until(...)` | `fss_time::sleep_until(*node, target_time)` |
| `rclcpp::Rate(rate_hz)` | `fss_time::Rate rate(*node, rate_hz)` |
| `rclcpp::spin(node)` | `fss_time::spin(node)` |
| `rclcpp::executors::SingleThreadedExecutor` | `fss_time::executors::SingleThreadedExecutor` |
| `rclcpp::executors::MultiThreadedExecutor` | `fss_time::executors::MultiThreadedExecutor` |

### Timer and `now()`

The timer must use the node clock, not a wall timer. `this->now()` uses the node's ROS clock and therefore follows `/clock`; a separately constructed system clock does not.

```cpp
timer_ = rclcpp::create_timer(
  this,
  this->get_clock(),
  rclcpp::Duration::from_seconds(0.01),
  [this]() {
    const auto sim_time = this->now();
    publish_state(sim_time);
  });
```

On ROS 2 Jazzy and newer, the equivalent node API is also available:

```cpp
timer_ = this->create_timer(
  rclcpp::Duration::from_seconds(0.01),
  [this]() { publish_state(this->now()); });
```

### Sleep and periodic loops

Replace wall sleeps and generic `rclcpp::Rate` with the node-aware helpers. They announce the wake-up time before blocking.

```cpp
fss_time::Rate rate(*this, 100.0);
while (rclcpp::ok()) {
  update_dynamics(this->now());
  fss_time::sleep_for(*this, rclcpp::Duration::from_seconds(0.001));
  rate.sleep();
}
```

Use either `Rate::sleep()` or an explicit `sleep_for` when appropriate. If both are used, both calls are valid time constraints and the last wake-up target determines the next progress opportunity for that thread.

### Executor

Replace the executor so timer, subscription, service, and other callbacks are pinned to the current simulation time while they run:

```cpp
auto node = std::make_shared<MyNode>();
fss_time::spin(node);
```

For multi-threaded nodes, create the matching executor. Each worker thread becomes an independent participant:

```cpp
fss_time::executors::MultiThreadedExecutor executor(
  rclcpp::ExecutorOptions(), 4);
executor.add_node(node);
executor.spin();
executor.remove_node(node);
```

With `use_fss_sim_time:=false`, all `fss_time` helpers and executors retain normal ROS 2 behavior: they use the node clock but make no coordinator announcement. This allows one codebase to run both wall-time and lock-step simulation modes.

## Thread participants

Use a `thread_time_participant` directly when the application owns a worker thread or wants an explicit safe-time protocol instead of the ROS helpers. A participant is thread-local: call `for_current_thread()` once inside each worker thread.

```cpp
void worker_loop(rclcpp::Node & node)
{
  auto & participant = fss_time::thread_time_participant::for_current_thread(node);
  participant.set_follows_real_time(false);

  while (rclcpp::ok()) {
    const auto next_time = node.now() + rclcpp::Duration::from_seconds(0.01);
    participant.announce_next_safe_time(next_time);
    node.get_clock()->sleep_until(next_time);
    step_simulation(node.now());
  }
}
```

`announce_next_safe_time()` registers the participant automatically. It announces a conservative promise: the thread will not require the simulation time before the announced time. The participant unregisters on thread-local destruction; call `unregister_participant()` when an earlier explicit removal is required.

Direct participants are also suitable for non-ROS worker loops. They still need an `rclcpp::Node` to obtain the ROS clock, coordinator endpoint, and parameters. Do not combine manual announcements with `Rate`/`sleep_*` on the same thread unless the intended sequence of safe-time constraints is clear.

## Advanced usage

### Real-time following

`follows_real_time:=true` is the default because it models a real system: a slow task should not cause the global clock to fall behind wall time indefinitely when the configured `max_real_time_factor` permits real-time progress. The coordinator therefore has a wall-time catch-up floor in addition to participant requests.

Physics and control simulation often need stricter behavior: the next time step must not be granted until that specific participant has completed its current computation. Disable real-time following for such a participant:

```cpp
auto & participant = fss_time::thread_time_participant::for_current_thread(*this);
participant.set_follows_real_time(false);
```

When any non-following participant has not issued its next request, the coordinator waits for it before applying the real-time catch-up floor. To disable this policy globally, launch the coordinator with:

```bash
ros2 launch fss_time time_coordinator.launch.py follows_real_time:=false
```

Set `max_real_time_factor` to a large positive value, such as `1000.0`, for the highest practical speed. The regulator period is clamped internally, so execution remains bounded by available CPU and participant work. `max_real_time_factor:=0.0` disables periodic regulator requests; it is not an unconstrained free-run mode and is intended for externally driven/cascaded configurations.

### Multi-machine hierarchy

Use cascaded coordinators to synchronize ROS 2 nodes across multiple machines without requiring every node to connect to a remote coordinator. The parent coordinator is the global time authority. Each machine runs one child coordinator: it aggregates its local participants, announces that machine's earliest safe time to the parent, waits for the parent's grant, and then publishes the granted `/clock` to local ROS nodes.

```text
  Machine A                                      Machine B
  local ROS nodes                                local ROS nodes
       |                                               |
       v                                               v
  child coordinator A . . . local safe time . . . child coordinator B
       |                                               |
       '-----------------. . . . . . . . . . .-------'
                             parent coordinator
                             global time grant
       .-----------------. . . . . . . . . . .-------.
       v                                               v
  local /clock A                                local /clock B
```

As a result, nodes on all machines observe the same granted simulation time, while normal ROS topics and services can remain on their local machine. The parent does not need to publish `/clock`; its responsibility is only to combine child safe-time requests and issue the global grants.

1. Start the parent coordinator on a reachable host with TCP endpoints and no local `/clock` publication:

```bash
ros2 launch fss_time parent_time_coordinator_example.launch.py \
  fss_time_coordinator_endpoint:=tcp://*:5545 \
  fss_time_coordinator_pub_endpoint:=tcp://*:0
```

2. On every child machine, launch one coordinator that points to the parent. The child discovers the parent PUB endpoint, forwards its local safe-time constraint to the parent, and publishes the granted clock locally:

```bash
ros2 launch fss_time time_coordinator.launch.py \
  namespace:=child1 \
  fss_time_parent_coordinator_endpoint:=tcp://parent-host:5545
```

3. Launch local application nodes with `use_fss_sim_time:=true` and `use_sim_time:=true`. Give each node the child coordinator endpoint explicitly, or place the nodes in the same namespace so the namespace-derived default is identical. Every child coordinator must use a distinct namespace or endpoint when several run on the same machine.

All machines then advance at the pace of the slowest relevant participant while ROS topic traffic can remain local to each child coordinator.

## Trouble shooting

### Node waits for ROS time to become active

`use_fss_sim_time` was enabled without active ROS simulation time. Ensure that a coordinator with `publish_clock:=true` is running and that the application launch sets both parameters before the node is created:

```python
SetParameter(name="use_fss_sim_time", value=True)
SetParameter(name="use_sim_time", value=True)
```

### Clock does not advance

The coordinator advances only after all active finite participants announce their next safe time. Check stalled participants and, after a crashed process, remove stale registrations:

```bash
ros2 service call /fss/clear_zombie_participants std_srvs/srv/Trigger '{}'
```

The coordinator status is available on `fss/sim_clock_status`; use the launch UI for live state, pause/resume, and real-time-factor control.

### CPU usage increases after enabling `use_sim_time`

Each ROS node using simulation time independently subscribes to `/clock` and processes clock-update callbacks. At high simulation rates, this per-node work can become a significant CPU cost in addition to the simulation itself. The cost grows with both the `/clock` frequency and the number of nodes.

When multiple components run in one process, prefer combining them into one `rclcpp::Node` when their lifecycle and interfaces allow it, instead of creating many nodes in the same process. Reusing one node shares its `/clock` handling and avoids unnecessary node/executor overhead. Keep separate nodes only where isolation, namespaces, or independent callback scheduling are needed.
