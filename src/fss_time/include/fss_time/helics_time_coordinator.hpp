#ifndef FSS_TIME_HELICS_TIME_COORDINATOR_HPP_
#define FSS_TIME_HELICS_TIME_COORDINATOR_HPP_

#include <cstdint>
#include <memory>
#include <string>

namespace fss_time
{

struct HelicsTimeOptions
{
  std::string participant_id;
  std::string core_type{"zmq"};
  std::string broker_address{"127.0.0.1"};
  int broker_port{23404};
  int64_t time_delta_ns{1000000};
  double max_speed_ratio{1.0};
};

struct HelicsGrantResult
{
  bool advanced{false};
  bool paused{false};
  int64_t grant_time_ns{0};
};

class HelicsTimeCoordinator
{
public:
  explicit HelicsTimeCoordinator(HelicsTimeOptions options);
  ~HelicsTimeCoordinator();

  HelicsTimeCoordinator(const HelicsTimeCoordinator &) = delete;
  HelicsTimeCoordinator & operator=(const HelicsTimeCoordinator &) = delete;

  void start();
  void finalize();

  void set_next_safe_time(int64_t next_safe_time_ns);
  void set_idle();
  void set_speed(double max_speed_ratio, int64_t local_steady_now_ns);
  void set_paused(bool paused);
  void reset(int64_t sim_time_ns, int64_t local_steady_now_ns);

  HelicsGrantResult request_grant(int64_t local_steady_now_ns);

  int64_t current_time_ns() const { return current_time_ns_; }
  bool paused() const { return paused_; }

private:
  struct Impl;

  int64_t speed_cap_ns(int64_t local_steady_now_ns) const;
  int64_t compute_request_time_ns(int64_t local_steady_now_ns) const;

  HelicsTimeOptions options_;
  std::unique_ptr<Impl> impl_;
  int64_t sim_start_ns_{0};
  int64_t wall_start_steady_ns_{0};
  int64_t current_time_ns_{0};
  int64_t next_safe_time_ns_{0};
  double max_speed_ratio_{1.0};
  bool paused_{false};
  bool active_{false};
  bool started_{false};
};

}  // namespace fss_time

#endif  // FSS_TIME_HELICS_TIME_COORDINATOR_HPP_
