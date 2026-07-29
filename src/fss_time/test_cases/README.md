# fss_time Test Cases

这个目录放 `fss_time` 的联调用测试场景，重点验证两件事：

- broker 能否随 launch 一起启动并发布 `/clock`
- 多节点、多线程、不同循环周期下，仿真时间是否真的驱动业务循环推进

## 场景说明

`launch/multi_node_sim_time_test_case.launch.py` 会启动：

- `sim_time_broker`
- 多个 `sim_time_test_load_node`

每个 `sim_time_test_load_node` 会启动多个线程。每个线程都会：

- 连接到同一个 ZeroMQ IPC broker
- 使用 `thread_time_participant` 申请下一个安全仿真时间
- 在自己的 while 循环里按不同周期推进
- 周期性发布 `std_msgs/msg/UInt64MultiArray`

消息字段依次为：

1. 线程编号
2. 序号
3. 当前发布时刻的仿真时间，单位 ns
4. 当前线程周期，单位 ns

主要参数直接由 launch argument 控制，不再额外依赖测试 YAML 配置文件。

## 启动

```bash
ros2 launch fss_time multi_node_sim_time_test_case.launch.py
```

常用参数：

```bash
ros2 launch fss_time multi_node_sim_time_test_case.launch.py \
  test.node_count:=5 \
  node.thread_count:=6 \
  node.base_period_ms:=10 \
  node.period_step_ms:=10 \
  broker.max_real_time_factor:=1.0
```

## 判断仿真时间是否生效

- 终端日志里 `sim_time` 应持续增长，而不是一直停在 `0`
- `max_real_time_factor:=1.0` 时，仿真时间推进速度应接近真实时间
- `max_real_time_factor:=0.0` 时，仿真时间通常会明显快于真实时间
- 不同线程发布频率不同，但发布时间戳都应随 broker 授时单调增加

可直接观察：

```bash
ros2 topic echo /clock
ros2 topic echo /sim_time_test/node_0/load/thread_0
```

## Executor 测试场景

`launch/executor_time_test_case.launch.py` 会启动：

- `sim_time_broker`
- 使用 `fss_time::spin()` 的节点
- 使用 `fss_time::executors::SingleThreadedExecutor` 的节点
- 使用 `fss_time::executors::MultiThreadedExecutor` 的节点

每个节点默认创建 2 个 ROS-time timer，可用 `node.timer_count` 调整数量；每个 timer
按 `node.timer_period_ms` 指定的仿真时间周期发布
`std_msgs/msg/UInt64MultiArray`。消息字段依次为：

1. executor 类型编号：`1=fss_spin`，`2=single_threaded`，`3=multi_threaded`
2. 序号
3. 当前 timer callback 的仿真时间，单位 ns
4. timer 周期，单位 ns
5. timer 编号

启动：

```bash
ros2 launch fss_time executor_time_test_case.launch.py
```

常用参数：

```bash
ros2 launch fss_time executor_time_test_case.launch.py \
  spin.node_count:=2 \
  single.node_count:=2 \
  multi.node_count:=2 \
  multi.thread_count:=4 \
  node.timer_period_ms:=20 \
  node.timer_count:=4 \
  broker.max_real_time_factor:=1.0
```

可直接观察：

```bash
ros2 topic echo /fss_spin_0/executor_time/fss_spin/timer_0
ros2 topic echo /single_threaded_0/executor_time/single_threaded/timer_0
ros2 topic echo /multi_threaded_0/executor_time/multi_threaded/timer_0
```
