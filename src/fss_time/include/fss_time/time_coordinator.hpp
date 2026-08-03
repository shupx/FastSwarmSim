#ifndef FSS_TIME_TIME_COORDINATOR_HPP_
#define FSS_TIME_TIME_COORDINATOR_HPP_

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

class TimeCoordinator
{
public:
  /**
   * @brief Construct a simulation time coordinator attached to a ROS node.
   * @param node Node used for parameters, publishers, services, and timers.
   */
  explicit TimeCoordinator(rclcpp::Node & node);

  /**
   * @brief Stop coordinator background work and release coordinator resources.
   */
  ~TimeCoordinator();

  /**
   * @brief Copy construction is disabled.
   */
  TimeCoordinator(const TimeCoordinator &) = delete;

  /**
   * @brief Copy assignment is disabled.
   */
  TimeCoordinator & operator=(const TimeCoordinator &) = delete;

  /**
   * @brief Start coordinator communication and timer-driven clock regulation.
   */
  void start();

  /**
   * @brief Enable or pause simulation clock advancement.
   * @param running true to advance simulated time, false to pause it.
   */
  void set_running(bool running);

  /**
   * @brief Set the maximum simulated-time to wall-time speed ratio.
   * @param max_real_time_factor Maximum allowed real time factor.
   */
  void set_max_real_time_factor(double max_real_time_factor);

  /**
   * @brief Build a status message describing coordinator state.
   * @return Current coordinator status message.
   */
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
  void on_real_time_tick();
  void on_clock_status_tick();
  void update_real_time_request_locked();
  void update_observed_real_time_factor_locked();
  void try_update_clock_locked();
  void publish_clock_locked();
  void publish_status();
  int64_t compute_regulator_target_ns_locked() const;
  void reset_regulator_timer();
  void reset_real_time_timer();
  std::chrono::nanoseconds regulator_wall_period() const;
  std::string normalize_zmq_endpoint(const std::string & endpoint) const;

  struct Impl;

  rclcpp::Node & node_;
  std::unique_ptr<Impl> impl_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<fss_time_interfaces::msg::SimClockStatus>::SharedPtr status_pub_;
  rclcpp::Service<fss_time_interfaces::srv::SimClockControl>::SharedPtr control_srv_;
  rclcpp::TimerBase::SharedPtr regulator_timer_;
  rclcpp::TimerBase::SharedPtr real_time_timer_;
  rclcpp::TimerBase::SharedPtr clock_status_timer_;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, ParticipantState> participants_;
  int64_t sim_time_ns_{0};
  int64_t regulator_request_ns_{0};
  int64_t real_time_request_ns_{0};
  int64_t speed_regulator_step_ns_{5000000};  // 5 ms
  int64_t min_operation_walltime_{100000};
  double max_real_time_factor_{1.0};
  double observed_real_time_factor_{0.0};
  int64_t observed_rtf_last_sim_time_ns_{0};
  std::chrono::steady_clock::time_point observed_rtf_last_wall_time_{};
  std::chrono::steady_clock::time_point real_time_last_wall_time_{};
  bool running_{true};
  bool follows_real_time_{true};
  std::string debug_msg_{"try_update_clock_locked has not run yet"};
  std::string endpoint_;
  std::thread receive_thread_;
  std::atomic<bool> stop_receive_{false};
};

}  // namespace fss_time

#endif  // FSS_TIME_TIME_COORDINATOR_HPP_
