#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "fss_time/fss_time.hpp"
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

}  // namespace

class TimerSleepTestNode : public rclcpp::Node
{
public:
  explicit TimerSleepTestNode(const rclcpp::NodeOptions & options)
  : Node("timer_sleep_test_node", options)
  {
    topic_name_ = declare_or_get_parameter<std::string>(*this, "topic_name", "timer_sleep");
    timer_period_ms_ =
      std::max<int64_t>(1, declare_or_get_parameter<int64_t>(*this, "timer_period_ms", 10));
    callback_sleep_for_ms_ =
      std::max<int64_t>(0, declare_or_get_parameter<int64_t>(*this, "callback_sleep_for_ms", 5));
    log_every_n_ =
      std::max<int64_t>(1, declare_or_get_parameter<int64_t>(*this, "log_every_n", 100));

    publisher_ = create_publisher<std_msgs::msg::UInt64MultiArray>(topic_name_, rclcpp::QoS(10));
    timer_ = rclcpp::create_timer(
      this,
      get_clock(),
      rclcpp::Duration::from_nanoseconds(timer_period_ms_ * 1000000LL),
      [this]() { on_timer(); });
  }

private:
  void on_timer()
  {
    const auto before_sleep_ns = now().nanoseconds();
    if (callback_sleep_for_ms_ > 0) {
      fss_time::sleep_for(*this, rclcpp::Duration::from_nanoseconds(callback_sleep_for_ms_ * 1000000LL));
    }
    const auto publish_time_ns = now().nanoseconds();

    std_msgs::msg::UInt64MultiArray message;
    message.data = {
      2,
      sequence_,
      static_cast<uint64_t>(std::max<int64_t>(0, publish_time_ns)),
      static_cast<uint64_t>(timer_period_ms_ * 1000000LL),
      static_cast<uint64_t>(callback_sleep_for_ms_ * 1000000LL),
      static_cast<uint64_t>(std::max<int64_t>(0, publish_time_ns - before_sleep_ns))
    };
    publisher_->publish(message);

    if (sequence_ % static_cast<uint64_t>(log_every_n_) == 0) {
      RCLCPP_INFO(
        get_logger(),
        "timer_sleep sequence=%lu before_sleep=%ld ns publish_time=%ld ns timer_period=%ld ms cb_sleep=%ld ms",
        sequence_,
        before_sleep_ns,
        publish_time_ns,
        timer_period_ms_,
        callback_sleep_for_ms_);
    }
    ++sequence_;
  }

  std::string topic_name_;
  int64_t timer_period_ms_{10};
  int64_t callback_sleep_for_ms_{5};
  int64_t log_every_n_{100};
  uint64_t sequence_{0};
  rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimerSleepTestNode>(rclcpp::NodeOptions());
  fss_time::spin(node);
  rclcpp::shutdown();
  return 0;
}
