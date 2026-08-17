#ifndef FSS_TIME_TIME_COORDINATOR_HPP_
#define FSS_TIME_TIME_COORDINATOR_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>

#include "fss_time/zeromq_time_participant_backend.hpp"
#include "fss_time_interfaces/msg/sim_clock_status.hpp"
#include "fss_time_interfaces/srv/sim_clock_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_srvs/srv/trigger.hpp"

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

  /**
   * @brief Remove participants that have not made a recent time request.
   * @return Number of participants removed.
   */
  std::size_t clear_zombie_participants();

private:
  enum class DebugState
  {
    Initializing,
    NoParticipants,
    WaitingForParticipant,
    InfiniteRequest,
    RequestNotAdvanced,
    Advanced,
    ParentGrantPublished
  };

  struct ParticipantState
  {
    int64_t request_time_ns{0};
    bool has_new_request{false};
    bool follows_real_time{true};
    std::chrono::steady_clock::time_point last_request_walltime{};
  };

  void receive_router_loop();
  void receive_parent_loop();
  std::string handle_message(const std::string & identity, const std::string & message);
  void on_regulator_tick();
  void on_real_time_tick();
  void on_clock_status_tick();
  void update_real_time_request_locked();
  void update_observed_real_time_factor_locked();
  void try_update_clock_locked();
  bool should_advance(int64_t & output_request_time_ns);
  void advance_time_locked(int64_t target_time_ns);
  void publish_clock_locked();
  void publish_clock(int64_t sim_time_ns);
  void publish_granted_time(int64_t sim_time_ns);
  void enqueue_publish_task(std::function<void()> task);
  void enqueue_announce_task(std::function<void()> task);
  void start_async_worker();
  void stop_async_worker();
  void publish_worker_loop();
  void announce_worker_loop();
  void receive_router_message();
  void receive_parent_grant();
  void publish_status();
  int64_t compute_regulator_target_ns_locked() const;
  void reset_regulator_timer();
  void reset_real_time_timer();
  std::chrono::nanoseconds regulator_wall_period() const;

  struct Impl;

  rclcpp::Node & node_;
  std::unique_ptr<Impl> impl_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<fss_time_interfaces::msg::SimClockStatus>::SharedPtr status_pub_;
  rclcpp::Service<fss_time_interfaces::srv::SimClockControl>::SharedPtr control_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_zombie_participants_srv_;
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
  std::chrono::milliseconds zombie_participant_timeout_{500};
  double max_real_time_factor_{1.0};
  double observed_real_time_factor_{0.0};
  int64_t observed_rtf_last_sim_time_ns_{0};
  std::chrono::steady_clock::time_point observed_rtf_last_wall_time_{};
  std::chrono::steady_clock::time_point real_time_last_wall_time_{};
  std::chrono::steady_clock::time_point update_clock_stats_start_{};
  uint64_t update_clock_call_count_{0};
  uint64_t update_clock_total_duration_ns_{0};
  bool running_{true};
  bool follows_real_time_{true};
  bool publish_clock_{true};
  DebugState debug_state_{DebugState::Initializing};
  int64_t debug_min_request_ns_{0};
  std::string endpoint_;
  std::string pub_endpoint_;
  std::string parent_endpoint_;
  std::string parent_pub_endpoint_;
  std::unique_ptr<ZeroMqTimeParticipantBackend> parent_participant_;
  bool has_parent_coordinator_{false};
  std::thread receive_router_thread_;
  std::thread receive_parent_thread_;
  std::atomic<bool> stop_receive_{false};
  std::thread publish_worker_thread_;
  std::thread announce_worker_thread_;
  std::mutex publish_mutex_;
  std::condition_variable publish_cv_;
  std::queue<std::function<void()>> publish_tasks_;
  std::mutex announce_mutex_;
  std::condition_variable announce_cv_;
  std::queue<std::function<void()>> announce_tasks_;
  std::atomic<bool> stop_async_worker_{false};
};

}  // namespace fss_time

#endif  // FSS_TIME_TIME_COORDINATOR_HPP_
