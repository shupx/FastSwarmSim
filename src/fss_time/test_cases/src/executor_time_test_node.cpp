#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "fss_time/executors.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"

namespace
{

template<typename T>
T declare_or_get_parameter(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<T>(name, default_value);
  }

  T value{};
  node.get_parameter(name, value);
  return value;
}

uint64_t executor_type_id(const std::string & executor_type)
{
  if (executor_type == "fss_spin") {
    return 1;
  }
  if (executor_type == "single_threaded") {
    return 2;
  }
  if (executor_type == "multi_threaded") {
    return 3;
  }
  return 0;
}

}  // namespace

class ExecutorTimeTestNode : public rclcpp::Node
{
public:
  explicit ExecutorTimeTestNode(const rclcpp::NodeOptions & options)
  : Node("executor_time_test_node", options)
  {
    executor_type_ =
      declare_or_get_parameter<std::string>(*this, "executor_type", "fss_spin");
    topic_prefix_ =
      declare_or_get_parameter<std::string>(*this, "topic_prefix", "executor_time");
    timer_period_ms_ =
      std::max<int64_t>(1, declare_or_get_parameter<int64_t>(*this, "timer_period_ms", 10));
    timer_count_ =
      std::max<int64_t>(1, declare_or_get_parameter<int64_t>(*this, "timer_count", 2));
    log_every_n_ =
      std::max<int64_t>(1, declare_or_get_parameter<int64_t>(*this, "log_every_n", 100));

    publishers_.reserve(static_cast<size_t>(timer_count_));
    timers_.reserve(static_cast<size_t>(timer_count_));
    for (int64_t timer_index = 0; timer_index < timer_count_; ++timer_index) {
      const auto topic_name =
        topic_prefix_ + "/" + executor_type_ + "/timer_" + std::to_string(timer_index);
      publishers_.push_back(
        create_publisher<std_msgs::msg::UInt64MultiArray>(topic_name, rclcpp::QoS(10)));
      timers_.push_back(rclcpp::create_timer(
          this,
          this->get_clock(),
          rclcpp::Duration::from_nanoseconds(timer_period_ms_ * 1000000LL),
          [this, timer_index]() { on_timer(timer_index); }));
      // timers_.push_back(this->create_timer(
      //     rclcpp::Duration::from_nanoseconds(timer_period_ms_ * 1000000LL),
      //     [this, timer_index]() { on_timer(timer_index); })); // jazzy+ API in node_impl.hpp
    }
  }

  const std::string & executor_type() const
  {
    return executor_type_;
  }

private:
  void on_timer(int64_t timer_index)
  {
    const auto sim_time_ns = now().nanoseconds();
    std_msgs::msg::UInt64MultiArray message;
    message.data = {
      executor_type_id(executor_type_),
      sequence_,
      static_cast<uint64_t>(std::max<int64_t>(0, sim_time_ns)),
      static_cast<uint64_t>(timer_period_ms_ * 1000000LL),
      static_cast<uint64_t>(timer_index)
    };
    publishers_.at(static_cast<size_t>(timer_index))->publish(message);

    if (sequence_ % static_cast<uint64_t>(log_every_n_) == 0) {
      RCLCPP_INFO(
        get_logger(),
        "executor_type=%s timer=%ld sequence=%lu sim_time=%ld ns period=%ld ms",
        executor_type_.c_str(),
        timer_index,
        sequence_,
        sim_time_ns,
        timer_period_ms_);
    }
    ++sequence_;
  }

  std::string executor_type_;
  std::string topic_prefix_;
  int64_t timer_period_ms_{10};
  int64_t timer_count_{2};
  int64_t log_every_n_{100};
  uint64_t sequence_{0};
  std::vector<rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr> publishers_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<ExecutorTimeTestNode>(options);

  const auto & executor_type = node->executor_type();
  if (executor_type == "fss_spin") {
    fss_time::spin(node);
  } else if (executor_type == "single_threaded") {
    fss_time::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } else if (executor_type == "multi_threaded") {
    const auto thread_count =
      std::max<int>(1, declare_or_get_parameter<int>(*node, "multi_thread_count", 2));
    fss_time::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(),
      static_cast<size_t>(thread_count));
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } else {
    throw std::invalid_argument("unknown executor_type: " + executor_type);
  }

  rclcpp::shutdown();
  return 0;
}
