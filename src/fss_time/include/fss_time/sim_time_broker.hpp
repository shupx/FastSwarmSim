#ifndef FSS_TIME_SIM_TIME_BROKER_HPP_
#define FSS_TIME_SIM_TIME_BROKER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "fss_time_interfaces/msg/sim_clock_status.hpp"
#include "fss_time_interfaces/srv/sim_clock_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace fss_time
{

class SimTimeBroker
{
public:
  explicit SimTimeBroker(rclcpp::Node & node);
  ~SimTimeBroker();

  SimTimeBroker(const SimTimeBroker &) = delete;
  SimTimeBroker & operator=(const SimTimeBroker &) = delete;

  void start();
  void set_running(bool running);
  void set_max_real_time_factor(double max_real_time_factor);
  fss_time_interfaces::msg::SimClockStatus status_message() const;

private:
  struct ParticipantState
  {
    int64_t request_time_ns{0};
    bool has_new_request{false};
  };

  void receive_loop();
  std::string handle_message(const std::string & identity, const std::string & message);
  void on_regulator_tick();
  void try_update_clock_locked();
  void publish_clock_locked();
  void publish_status();
  int64_t compute_regulator_target_ns_locked() const;
  void reset_regulator_timer();
  std::chrono::nanoseconds regulator_wall_period() const;
  std::string normalize_ipc_endpoint(const std::string & endpoint) const;

  struct Impl;

  rclcpp::Node & node_;
  std::unique_ptr<Impl> impl_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<fss_time_interfaces::msg::SimClockStatus>::SharedPtr status_pub_;
  rclcpp::Service<fss_time_interfaces::srv::SimClockControl>::SharedPtr control_srv_;
  rclcpp::TimerBase::SharedPtr regulator_timer_;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, ParticipantState> participants_;
  int64_t sim_time_ns_{0};
  int64_t regulator_request_ns_{0};
  int64_t speed_regulator_step_ns_{10000000};
  double max_real_time_factor_{1.0};
  bool running_{true};
  std::string ipc_endpoint_;
  std::thread receive_thread_;
  std::atomic<bool> stop_receive_{false};
};

}  // namespace fss_time

#endif  // FSS_TIME_SIM_TIME_BROKER_HPP_
