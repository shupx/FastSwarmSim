#ifndef FSS_TIME_SIM_TIME_BROKER_HPP_
#define FSS_TIME_SIM_TIME_BROKER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>

#include "fss_time/helics_broker_backend.hpp"
#include "fss_time/helics_thread_participant_backend.hpp"
#include "fss_time_interfaces/msg/sim_clock_status.hpp"
#include "fss_time_interfaces/srv/sim_clock_control.hpp"
#include "rclcpp/rclcpp.hpp"

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
  void on_regulator_tick();
  void publish_status();
  void refresh_participant_count();
  int64_t compute_regulator_target_ns() const;
  void reset_wall_anchor_locked(int64_t sim_time_ns);

  rclcpp::Node & node_;
  std::unique_ptr<HelicsBrokerBackend> broker_backend_;
  std::shared_ptr<HelicsThreadParticipantBackend> regulator_backend_;
  rclcpp::Publisher<fss_time_interfaces::msg::SimClockStatus>::SharedPtr status_pub_;
  rclcpp::Service<fss_time_interfaces::srv::SimClockControl>::SharedPtr control_srv_;
  rclcpp::TimerBase::SharedPtr regulator_timer_;

  mutable std::mutex mutex_;
  std::chrono::steady_clock::time_point wall_anchor_steady_;
  std::chrono::steady_clock::time_point last_participant_query_steady_;
  int64_t sim_anchor_ns_{0};
  int64_t helics_time_delta_ns_{1000000};
  int64_t speed_regulator_tick_ns_{1000000};
  int64_t participant_query_period_ns_{500000000};
  uint32_t cached_participant_count_{0};
  double max_real_time_factor_{1.0};
  bool running_{true};
};

}  // namespace fss_time

#endif  // FSS_TIME_SIM_TIME_BROKER_HPP_
