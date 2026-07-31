#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "fss_time/fss_time.hpp"
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

class FssRateTestNode : public rclcpp::Node
{
public:
  explicit FssRateTestNode(const rclcpp::NodeOptions & options)
  : Node("fss_rate_test_node", options)
  {
    topic_name_ = declare_or_get_parameter<std::string>(*this, "topic_name", "fss_rate");
    rate_period_ms_ =
      std::max<int64_t>(1, declare_or_get_parameter<int64_t>(*this, "rate_period_ms", 10));
    enable_loop_sleep_for_ =
      declare_or_get_parameter<bool>(*this, "enable_loop_sleep_for", false);
    loop_sleep_for_ms_ =
      std::max<int64_t>(0, declare_or_get_parameter<int64_t>(*this, "loop_sleep_for_ms", 1));
    log_every_n_ =
      std::max<int64_t>(1, declare_or_get_parameter<int64_t>(*this, "log_every_n", 100));

    publisher_ = create_publisher<std_msgs::msg::UInt64MultiArray>(topic_name_, rclcpp::QoS(10));
  }

  void run()
  {
    fss_time::Rate rate(*this, rclcpp::Duration::from_nanoseconds(rate_period_ms_ * 1000000LL));
    while (rclcpp::ok() && !stop_.load()) {
      publish_sample();
      if (enable_loop_sleep_for_ && loop_sleep_for_ms_ > 0) {
        fss_time::sleep_for(*this, rclcpp::Duration::from_nanoseconds(loop_sleep_for_ms_ * 1000000LL));
      }
      rate.sleep();
    }
  }

  void stop()
  {
    stop_.store(true);
  }

private:
  void publish_sample()
  {
    const auto sim_time_ns = now().nanoseconds();
    std_msgs::msg::UInt64MultiArray message;
    message.data = {
      1,
      sequence_,
      static_cast<uint64_t>(std::max<int64_t>(0, sim_time_ns)),
      static_cast<uint64_t>(rate_period_ms_ * 1000000LL),
      static_cast<uint64_t>(enable_loop_sleep_for_ ? 1 : 0),
      static_cast<uint64_t>(loop_sleep_for_ms_ * 1000000LL)
    };
    publisher_->publish(message);

    if (sequence_ % static_cast<uint64_t>(log_every_n_) == 0) {
      RCLCPP_INFO(
        get_logger(),
        "fss_rate sequence=%lu sim_time=%ld ns rate_period=%ld ms loop_sleep_for=%s/%ld ms",
        sequence_,
        sim_time_ns,
        rate_period_ms_,
        enable_loop_sleep_for_ ? "true" : "false",
        loop_sleep_for_ms_);
    }
    ++sequence_;
  }

  std::string topic_name_;
  int64_t rate_period_ms_{10};
  bool enable_loop_sleep_for_{false};
  int64_t loop_sleep_for_ms_{1};
  int64_t log_every_n_{100};
  uint64_t sequence_{0};
  std::atomic<bool> stop_{false};
  rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<FssRateTestNode>(options);

  std::thread spin_thread([node]() {
    fss_time::spin(node);
  });

  node->run();
  node->stop();
  rclcpp::shutdown();

  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  return 0;
}
