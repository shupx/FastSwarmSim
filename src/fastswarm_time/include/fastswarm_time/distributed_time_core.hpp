#ifndef FASTSWARM_TIME_DISTRIBUTED_TIME_CORE_HPP_
#define FASTSWARM_TIME_DISTRIBUTED_TIME_CORE_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>

#include "fastswarm_time_interfaces/msg/time_control.hpp"
#include "fastswarm_time_interfaces/msg/time_intent.hpp"

namespace fastswarm_time
{

struct ParticipantState
{
  std::string participant_id;
  uint64_t epoch{0};
  int64_t current_time_ns{0};
  int64_t next_safe_time_ns{0};
  int64_t lookahead_ns{0};
  uint8_t state{fastswarm_time_interfaces::msg::TimeIntent::STATE_ACTIVE};
  int64_t local_expiry_steady_ns{0};
};

struct GrantResult
{
  bool advanced{false};
  bool paused{false};
  int64_t grant_time_ns{0};
  std::size_t active_participants{0};
};

class DistributedTimeCore
{
public:
  DistributedTimeCore(int64_t sim_start_ns, double max_speed_ratio, int64_t lease_timeout_ns);

  void set_speed(double max_speed_ratio, int64_t local_steady_now_ns);
  void set_paused(bool paused);
  void reset(int64_t sim_time_ns, int64_t local_steady_now_ns);
  void observe_intent(
    const fastswarm_time_interfaces::msg::TimeIntent & intent,
    int64_t local_steady_now_ns);
  bool apply_control(
    const fastswarm_time_interfaces::msg::TimeControl & control,
    int64_t local_steady_now_ns);

  GrantResult compute_grant(int64_t local_steady_now_ns);

  int64_t current_time_ns() const { return current_time_ns_; }
  double max_speed_ratio() const { return max_speed_ratio_; }
  bool paused() const { return paused_; }
  uint64_t control_epoch() const { return control_epoch_; }
  const std::map<std::string, ParticipantState> & participants() const { return participants_; }

private:
  int64_t speed_cap_ns(int64_t local_steady_now_ns) const;
  void remove_expired(int64_t local_steady_now_ns);

  int64_t sim_start_ns_;
  int64_t wall_start_steady_ns_;
  int64_t current_time_ns_;
  double max_speed_ratio_;
  int64_t lease_timeout_ns_;
  bool paused_{false};
  uint64_t control_epoch_{0};
  std::map<std::string, ParticipantState> participants_;
};

}  // namespace fastswarm_time

#endif  // FASTSWARM_TIME_DISTRIBUTED_TIME_CORE_HPP_
