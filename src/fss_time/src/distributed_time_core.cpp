#include "fss_time/distributed_time_core.hpp"

#include <algorithm>

#include "fss_time/time_types.hpp"

namespace fss_time
{

DistributedTimeCore::DistributedTimeCore(
  int64_t sim_start_ns,
  double max_speed_ratio,
  int64_t lease_timeout_ns)
: sim_start_ns_(sim_start_ns),
  wall_start_steady_ns_(0),
  current_time_ns_(sim_start_ns),
  max_speed_ratio_(max_speed_ratio),
  lease_timeout_ns_(lease_timeout_ns)
{
}

void DistributedTimeCore::set_speed(double max_speed_ratio, int64_t local_steady_now_ns)
{
  sim_start_ns_ = current_time_ns_;
  wall_start_steady_ns_ = local_steady_now_ns;
  max_speed_ratio_ = max_speed_ratio;
}

void DistributedTimeCore::set_paused(bool paused)
{
  paused_ = paused;
}

void DistributedTimeCore::reset(int64_t sim_time_ns, int64_t local_steady_now_ns)
{
  sim_start_ns_ = sim_time_ns;
  wall_start_steady_ns_ = local_steady_now_ns;
  current_time_ns_ = sim_time_ns;
  participants_.clear();
}

void DistributedTimeCore::observe_intent(
  const fss_time_interfaces::msg::TimeIntent & intent,
  int64_t local_steady_now_ns)
{
  if (intent.participant_id.empty()) {
    return;
  }

  auto & state = participants_[intent.participant_id];
  if (intent.epoch < state.epoch) {
    return;
  }

  state.participant_id = intent.participant_id;
  state.epoch = intent.epoch;
  state.current_time_ns = to_ns(intent.current_time);
  state.next_safe_time_ns = to_ns(intent.next_safe_time);
  state.lookahead_ns = duration_to_ns(intent.lookahead);
  state.state = intent.state;
  state.local_expiry_steady_ns = local_steady_now_ns + lease_timeout_ns_;
}

bool DistributedTimeCore::apply_control(
  const fss_time_interfaces::msg::TimeControl & control,
  int64_t local_steady_now_ns)
{
  if (control.epoch < control_epoch_) {
    return false;
  }

  control_epoch_ = control.epoch;
  switch (control.command) {
    case fss_time_interfaces::msg::TimeControl::COMMAND_PAUSE:
      paused_ = true;
      break;
    case fss_time_interfaces::msg::TimeControl::COMMAND_RESUME:
      paused_ = false;
      break;
    case fss_time_interfaces::msg::TimeControl::COMMAND_SET_SPEED:
      set_speed(control.max_speed_ratio, local_steady_now_ns);
      break;
    case fss_time_interfaces::msg::TimeControl::COMMAND_RESET:
      reset(to_ns(control.reset_time), local_steady_now_ns);
      break;
    default:
      return false;
  }
  return true;
}

GrantResult DistributedTimeCore::compute_grant(int64_t local_steady_now_ns)
{
  remove_expired(local_steady_now_ns);

  GrantResult result;
  result.paused = paused_;
  result.grant_time_ns = current_time_ns_;

  if (paused_) {
    return result;
  }

  int64_t grant_ns = kInfiniteTimeNs;
  for (const auto & item : participants_) {
    const auto & participant = item.second;
    if (participant.state == fss_time_interfaces::msg::TimeIntent::STATE_ACTIVE) {
      grant_ns = std::min(grant_ns, participant.next_safe_time_ns);
      ++result.active_participants;
    }
  }

  if (grant_ns == kInfiniteTimeNs) {
    return result;
  }

  grant_ns = std::min(grant_ns, speed_cap_ns(local_steady_now_ns));
  if (grant_ns > current_time_ns_) {
    current_time_ns_ = grant_ns;
    result.advanced = true;
  }

  result.grant_time_ns = current_time_ns_;
  return result;
}

int64_t DistributedTimeCore::speed_cap_ns(int64_t local_steady_now_ns) const
{
  if (max_speed_ratio_ <= 0.0) {
    return kInfiniteTimeNs;
  }

  const auto elapsed_ns = std::max<int64_t>(0, local_steady_now_ns - wall_start_steady_ns_);
  return sim_start_ns_ + static_cast<int64_t>(elapsed_ns * max_speed_ratio_);
}

void DistributedTimeCore::remove_expired(int64_t local_steady_now_ns)
{
  for (auto it = participants_.begin(); it != participants_.end();) {
    const auto & state = it->second;
    const bool expired = local_steady_now_ns > state.local_expiry_steady_ns;
    const bool leaving =
      state.state == fss_time_interfaces::msg::TimeIntent::STATE_LEAVING;
    if (expired || leaving) {
      it = participants_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace fss_time
