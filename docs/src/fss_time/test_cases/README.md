# fss_time Test Cases

This directory contains integration test scenarios for `fss_time`. They primarily verify two things:

- whether the coordinator starts with the launch file and publishes `/clock`
- whether simulation time actually drives application loops with multiple nodes, multiple threads, and different loop periods

## Scenario Overview

`launch/multi_node_sim_time_test_case.launch.py` starts:

- `time_coordinator`
- multiple `sim_time_test_load_node` instances

Each `sim_time_test_load_node` starts multiple threads. Every thread:

- connects to the same ZeroMQ IPC coordinator
- requests the next safe simulation time with `thread_time_participant`
- advances its own while loop at a different period
- periodically publishes `std_msgs/msg/UInt64MultiArray`

The message fields are, in order:

1. Thread ID
2. Sequence number
3. Simulation time at publication, in ns
4. Current thread period, in ns

The main parameters are controlled directly by launch arguments and no longer depend on an additional test YAML configuration file.

## Start the Test

```bash
ros2 launch fss_time multi_node_sim_time_test_case.launch.py
```

Common parameters:

```bash
ros2 launch fss_time multi_node_sim_time_test_case.launch.py \
  test.node_count:=5 \
  node.thread_count:=6 \
  node.base_period_ms:=10 \
  node.period_step_ms:=10 \
  coordinator.max_real_time_factor:=1.0
```

## Check Whether Simulation Time Is Working

- `sim_time` should keep increasing in the terminal logs rather than remaining at `0`.
- With `max_real_time_factor:=1.0`, simulation time should advance at approximately real-time speed.
- With `max_real_time_factor:=0.0`, the coordinator stops periodically advancing regulator requests.
- Threads publish at different frequencies, but all publication timestamps should increase monotonically with the time granted by the coordinator.

You can observe the topics directly:

```bash
ros2 topic echo /clock
ros2 topic echo /sim_time_test/node_0/load/thread_0
```

## Executor Test Scenario

`launch/executor_time_test_case.launch.py` starts:

- `time_coordinator`
- a node using `fss_time::spin()`
- a node using `fss_time::executors::SingleThreadedExecutor`
- a node using `fss_time::executors::MultiThreadedExecutor`

Each node creates two ROS-time timers by default. Use `node.timer_count` to change the number of timers. Each timer publishes `std_msgs/msg/UInt64MultiArray` at the simulation-time period specified by `node.timer_period_ms`. The message fields are, in order:

1. Executor type ID: `1=fss_spin`, `2=single_threaded`, `3=multi_threaded`
2. Sequence number
3. Simulation time in the current timer callback, in ns
4. Timer period, in ns
5. Timer ID

Start the scenario:

```bash
ros2 launch fss_time executor_time_test_case.launch.py
```

Common parameters:

```bash
ros2 launch fss_time executor_time_test_case.launch.py \
  spin.node_count:=2 \
  single.node_count:=2 \
  multi.node_count:=2 \
  multi.thread_count:=4 \
  node.timer_period_ms:=20 \
  node.timer_count:=4 \
  coordinator.max_real_time_factor:=1.0
```

You can observe the topics directly:

```bash
ros2 topic echo /fss_spin_0/executor_time/fss_spin/timer_0
ros2 topic echo /single_threaded_0/executor_time/single_threaded/timer_0
ros2 topic echo /multi_threaded_0/executor_time/multi_threaded/timer_0
```

## Additional Configuration for More Than 100 Simulated Nodes

Fast DDS has a default participant limit of 100. When this limit is exceeded, participant creation may fail. Set the `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable to point to a custom XML configuration file to change the participant limit.

### Solution 1: Increase the Fast DDS UDP port range (recommended for up to 100 nodes)

`config/fastdds_large_scale_configuration.xml` already sets `mutation_tries` to 10000 (at least the number of nodes; a larger value is harmless) to avoid participant-creation failures. See the [ROS 2 large-number-of-participants guide](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#large-number-of-participants).

Add the following to `~/.bashrc` or `~/.zshrc`:

```bash
export FASTDDS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix fss_time)/share/fss_time/config/fastdds_large_scale_configuration.xml
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix fss_time)/share/fss_time/config/fastdds_large_scale_configuration.xml # for older versions (ROS Humble)
```

Then run:

```bash
source ~/.bashrc

# Apply the settings to ROS 2 CLI tools
ros2 daemon stop
ros2 daemon start
```

Advantages:

- Tests have shown that simulations with approximately 100 nodes can start successfully.
- No RMW implementation changes are required, so other Fast DDS nodes remain compatible.

Disadvantages:

- Above 100 nodes, port conflicts can still occur, startup becomes significantly slower, and packet loss may appear. This is because Simple Discovery uses UDP broadcasts in which every node participates; too many nodes can congest the network. Fast DDS can use [Discovery Server mode](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#setup-discovery-server), but tests have still shown startup problems, and running an additional Discovery Server is inconvenient.

### Solution 2: Switch the RMW implementation to Cyclone DDS (recommended for 100–200 nodes)

Unlike Fast DDS Simple Discovery, Cyclone DDS [shares discovery information](https://cyclonedds.io/docs/cyclonedds/latest/config/discovery-behavior.html) instead of requiring each participant to complete all discovery independently, making it more suitable for a large number of participants.

```bash
sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp -y
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # Add this to ~/.bashrc or ~/.zshrc
```

Advantages:

- Tests have shown that simulations with approximately 200 nodes can start successfully without an additional Discovery Server, and startup is faster than with Fast DDS.

Disadvantages:

- With fewer nodes, Cyclone DDS has a lower performance ceiling than Fast DDS, so the maximum RTF may be lower. The difference is less noticeable with more nodes.
- Above 200 nodes, ROS 2 startup and runtime overhead still increases significantly and can saturate the CPU. At that point, discovery is no longer the main limiting factor.

### Solution 3: Switch the RMW implementation to Zenoh (for 200+ nodes)

In ROS 2 Lyrical (the 2026 release), Zenoh is supported as a first-class RMW implementation. Its advantages include:

- More efficient node discovery, with the potential to support a large number of nodes. Zenoh's router-gossip discovery mechanism is similar to the ROS 1 master: nodes discover one another through the router gossip mechanism, which theoretically supports more participants. Zenoh also supports UDP multicast discovery, similar to Fast DDS Simple Discovery, but with lower overhead and better performance.
- Shared-memory transport for faster inter-process topic communication. This is particularly beneficial for the `/clock` topic published and subscribed by the `fss_time` coordinator.

Disadvantages:

- It is not compatible with the DDS-based ROS 2 ecosystem. Zenoh is not yet widely used, and many ROS 2 packages do not support the Zenoh RMW implementation.
- It does not solve the ROS 2 limitation that each node is a separate process with its own ROS context. With many processes, ROS 2 startup and runtime overhead can still saturate the CPU.

### Solution 4: Start multiple nodes in one process (recommended for 1000+ nodes)

Like ROS 1 nodelets, ROS 2 can start multiple nodes in a single process and use a multithreaded executor. This avoids ROS 2 multi-process overhead. For example, the coordinator and all dynamics simulation nodes can run in one process.

Advantages:

- Avoids ROS 2 multi-process overhead and significantly reduces startup and runtime costs. This is the most promising approach for simulations with more than 1000 nodes.
- Nodes within one process can use zero-copy communication, potentially speeding up `/clock` publication and subscription and allowing a higher RTF from the `fss_time` coordinator.

Disadvantages:

- Less flexible: user algorithms are difficult to place in the same process and normally need to run in separate processes.
- When multiple nodes share a process, the ROS 2 multithreaded executor cannot completely isolate callbacks from different nodes. A callback that was originally single-threaded may run on multiple threads, especially when multiple callback groups are used. Callback-group configuration is usually sufficient to avoid problems, but behavior may not be identical to a single-threaded process.
