#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fss_time/thread_time_participant.hpp"
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

class SimTimeTestLoadNode : public rclcpp::Node
{
public:
  SimTimeTestLoadNode()
  : Node("sim_time_test_load_node")
  {
    topic_prefix_ = declare_or_get_parameter<std::string>(*this, "topic_prefix", "load");
    thread_count_ = std::max<int>(1, declare_or_get_parameter<int>(*this, "thread_count", 4));
    base_period_ms_ = std::max<int64_t>(1, declare_or_get_parameter<int64_t>(*this, "base_period_ms", 10));
    period_step_ms_ = std::max<int64_t>(0, declare_or_get_parameter<int64_t>(*this, "period_step_ms", 5));
    wait_poll_ms_ = std::max<int64_t>(1, declare_or_get_parameter<int64_t>(*this, "wait_poll_ms", 1));
    enable_fss_time_ = declare_or_get_parameter<bool>(*this, "enable_fss_time", true);
    (void)declare_or_get_parameter<std::string>(*this, "helics_core_type", "zmq");
    (void)declare_or_get_parameter<std::string>(*this, "broker_address", "127.0.0.1");
    (void)declare_or_get_parameter<int>(*this, "broker_port", 23404);
    (void)declare_or_get_parameter<int64_t>(*this, "helics_time_delta_ns", 1000000);

    published_counts_.assign(static_cast<size_t>(thread_count_), 0);
    publishers_.reserve(static_cast<size_t>(thread_count_));
    worker_period_ns_.reserve(static_cast<size_t>(thread_count_));

    for (int i = 0; i < thread_count_; ++i) {
      const auto topic_name = topic_prefix_ + "/thread_" + std::to_string(i);
      publishers_.push_back(create_publisher<std_msgs::msg::UInt64MultiArray>(topic_name, rclcpp::QoS(10)));
      worker_period_ns_.push_back((base_period_ms_ + period_step_ms_ * i) * 1000000LL);
    }

    status_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { log_status(); });
    start_workers();
  }

  ~SimTimeTestLoadNode() override
  {
    stop_.store(true);
    for (auto & worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

private:
  void start_workers()
  {
    workers_.reserve(static_cast<size_t>(thread_count_));
    for (int i = 0; i < thread_count_; ++i) {
      workers_.emplace_back([this, i]() { worker_loop(i); });
    }
  }

  void worker_loop(int worker_index)
  {
    const auto wait_period = std::chrono::milliseconds(wait_poll_ms_);
    const auto period_ns = worker_period_ns_[static_cast<size_t>(worker_index)];
    const auto participant_id =
      std::string(get_fully_qualified_name()) + "_thread_" + std::to_string(worker_index);

    fss_time::thread_time_participant * participant = nullptr;
    if (enable_fss_time_) {
      std::lock_guard<std::mutex> lock(participant_init_mutex_);
      participant = &fss_time::thread_time_participant::for_current_thread(*this, participant_id);
    }

    uint64_t sequence = 0;
    while (rclcpp::ok() && !stop_.load()) {
      int64_t publish_time_ns = now().nanoseconds();
      if (participant != nullptr) {
        const auto current_time_ns = participant->get_sim_time().nanoseconds();
        const auto target_time_ns = std::max(current_time_ns, publish_time_ns) + period_ns;
        participant->announce_next_safe_time(rclcpp::Time(target_time_ns, RCL_ROS_TIME));
        wait_for_grant(*participant, target_time_ns, wait_period);
        if (stop_.load() || !rclcpp::ok()) {
          break;
        }
        publish_time_ns = participant->get_sim_time().nanoseconds();
      } else {
        std::this_thread::sleep_for(std::chrono::nanoseconds(period_ns));
        publish_time_ns = now().nanoseconds();
      }

      std_msgs::msg::UInt64MultiArray message;
      message.data = {
        static_cast<uint64_t>(worker_index),
        sequence,
        static_cast<uint64_t>(publish_time_ns),
        static_cast<uint64_t>(period_ns)
      };
      publishers_[static_cast<size_t>(worker_index)]->publish(message);
      {
        std::lock_guard<std::mutex> lock(counts_mutex_);
        ++published_counts_[static_cast<size_t>(worker_index)];
      }
      ++sequence;
    }
  }

  void wait_for_grant(
    fss_time::thread_time_participant & participant,
    int64_t target_time_ns,
    const std::chrono::milliseconds & wait_period) const
  {
    while (rclcpp::ok() && !stop_.load()) {
      if (participant.get_sim_time().nanoseconds() >= target_time_ns) {
        return;
      }
      std::this_thread::sleep_for(wait_period);
    }
  }

  void log_status()
  {
    std::string counts = "[";
    std::lock_guard<std::mutex> lock(counts_mutex_);
    for (size_t i = 0; i < published_counts_.size(); ++i) {
      counts += std::to_string(published_counts_[i]);
      if (i + 1 < published_counts_.size()) {
        counts += ", ";
      }
    }
    counts += "]";

    RCLCPP_INFO(
      get_logger(),
      "sim_time=%ld ns, threads=%d, periods_ms=[%ld..%ld], published=%s",
      now().nanoseconds(),
      thread_count_,
      base_period_ms_,
      base_period_ms_ + period_step_ms_ * (thread_count_ - 1),
      counts.c_str());
  }

  std::string topic_prefix_;
  int thread_count_{1};
  int64_t base_period_ms_{10};
  int64_t period_step_ms_{5};
  int64_t wait_poll_ms_{1};
  bool enable_fss_time_{true};
  std::atomic<bool> stop_{false};
  std::vector<std::thread> workers_;
  std::vector<int64_t> worker_period_ns_;
  std::mutex counts_mutex_;
  std::mutex participant_init_mutex_;
  std::vector<uint64_t> published_counts_;
  std::vector<rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr> publishers_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimTimeTestLoadNode>());
  rclcpp::shutdown();
  return 0;
}
