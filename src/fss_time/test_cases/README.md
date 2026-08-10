# fss_time Test Cases

这个目录放 `fss_time` 的联调用测试场景，重点验证两件事：

- coordinator 能否随 launch 一起启动并发布 `/clock`
- 多节点、多线程、不同循环周期下，仿真时间是否真的驱动业务循环推进

## 场景说明

`launch/multi_node_sim_time_test_case.launch.py` 会启动：

- `time_coordinator`
- 多个 `sim_time_test_load_node`

每个 `sim_time_test_load_node` 会启动多个线程。每个线程都会：

- 连接到同一个 ZeroMQ IPC coordinator
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
  coordinator.max_real_time_factor:=1.0
```

## 判断仿真时间是否生效

- 终端日志里 `sim_time` 应持续增长，而不是一直停在 `0`
- `max_real_time_factor:=1.0` 时，仿真时间推进速度应接近真实时间
- `max_real_time_factor:=0.0` 时，coordinator 不再周期性推进 regulator request
- 不同线程发布频率不同，但发布时间戳都应随 coordinator 授时单调增加

可直接观察：

```bash
ros2 topic echo /clock
ros2 topic echo /sim_time_test/node_0/load/thread_0
```

## Executor 测试场景

`launch/executor_time_test_case.launch.py` 会启动：

- `time_coordinator`
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
  coordinator.max_real_time_factor:=1.0
```

可直接观察：

```bash
ros2 topic echo /fss_spin_0/executor_time/fss_spin/timer_0
ros2 topic echo /single_threaded_0/executor_time/single_threaded/timer_0
ros2 topic echo /multi_threaded_0/executor_time/multi_threaded/timer_0
```

## 超过100个节点仿真的额外配置

fastdds 默认的 participant 限制是 100 个，超过会报错。可通过设置环境变量 `FASTRTPS_DEFAULT_PROFILES_FILE` 指向自定义 XML 配置文件来修改 participant 限制。

#### 解决方法1： 增加fastdds的udp端口范围（100个以内节点首选）

`config/fastdds_large_scale_configuration.xml` 中已经设置了 `mutation_tries` 为 10000（至少超过节点数，多了无害），避免 participant 创建失败。参考 https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#large-number-of-participants

在 `~/.bashrc` 或 `~/.zshrc` 中添加：

```bash
export FASTDDS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix fss_time)/share/fss_time/config/fastdds_large_scale_configuration.xml
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix fss_time)/share/fss_time/config/fastdds_large_scale_configuration.xml # for old version (ROS humble)
```

Then run 

```bash
source ~/.bashrc

# make ros2 cli tools take effect
ros2 daemon stop
ros2 daemon start
```

优点：

- 实测启动100个节点左右的仿真是可行的。

- 不用改rmw实现，兼容现有fastdds的其他节点。

缺点：

- 超过100个后仍然容易出现端口占用问题，而且启动速度明显变慢，甚至出现丢包。
这是因为它采用的是Simple Discovery,每个节点都参与UDP广播的discovery机制，节点数过多时会导致网络拥塞。即使fastdds可以改用[Discovery Server](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#setup-discovery-server)模式来解决这个问题，但实测仍然会出现启动问题，而且额外启动Discovery Server并不方便。

#### 解决方法2： rmw改用cyclonedds（100-200个节点首选）

不同于fastdds的simple discovery机制，cyclonedds会[共享 discovery 信息](https://cyclonedds.io/docs/cyclonedds/latest/config/discovery-behavior.html?utm_source=chatgpt.com)，而不是简单让每个 participant 独立完成全部 discovery, 对大规模 participant 更友好。

```bash
sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp -y
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # 添加到 ~/.bashrc 或 ~/.zshrc
```

优点：

- 实测启动200个节点左右的仿真是可行的，不用额外启动Discovery Server，启动速度也比fastdds快。

缺点：

- 节点数量较少时，cyclonedds的性能上限低于fastdds，仿真时间最大RTF推进速度可能会慢一些。但是数量较多时，差异不明显。

- 多于200个节点后，ros启动和运行开销仍然会明显增加，容易打满cpu。但这已经和发现机制关系不大了。

#### 解决方案3： rmw改用zenoh（200+节点选择）

在2026版本的ros2 lyrical中，zenoh已经被支持为一等rmw实现。优势：

- 节点Discovery效率更高，有望支持大规模节点。zenoh的router gossip发现机制类似ROS1的master节点，所有节点都通过router gossip来发现其他节点，理论上可以支持更多的participant。当然zenoh也支持udp multicast discovery机制，类似fastdds的simple discovery机制，但是overhead更低，性能更好。

- 支持共享内存传输，多进程间话题通信可以更快。这对于fss_time中的/clock话题发布和订阅非常有利，有望提供fss_time coordinator的更高RTF推进速度。

缺点：

- 不兼容DDS版本的ros2生态，zenoh目前还没有被广泛使用，很多ros2包不支持zenoh rmw。

- 仍然未能解决ros2一个节点就是一个进程（分配一个ros context）的问题，进程多时，ros2启动和运行开销仍然会明显增加，容易打满cpu。

#### 解决方案4： 单进程启动多node（1000+节点首选）

类似ros1的nodelet，ros2也支持在一个进程中启动多个node，并用多线程executor。这样可以避免ros2多进程开销。比如coordinator以及所有的动力学仿真节点可以在一个进程中启动，避免ros2多进程开销。优点：

- 避免ros2多进程开销，启动和运行开销明显降低。是最有可能实现1000+节点仿真的方案。

- 进程内部可以使用零拷贝通信，/clock话题发布和订阅可以更快，有望提供fss_time coordinator的更高RTF推进速度。

缺点：

- 不灵活，难以把用户算法也放入一个进程中。用户算法通常需要在不同的进程中运行。

- 多个node在同一个进程中运行时，ros2的多线程executor调度机制不能完全隔离不同node的callback，可能会导致一个node的原本单线程的callback现在被分配到多线程上运行，和单线程行为不完全一致，尤其是用了多个callback group的时候。不过注意callback group的设置一般问题不大。