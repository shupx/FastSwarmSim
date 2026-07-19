#include "fss_time/time_participant.hpp"

#include <algorithm>
#include <utility>

#include "fss_time/time_types.hpp"

namespace fss_time
{

TimeParticipant::TimeParticipant(
  rclcpp::Node & node,
  std::string participant_id,
  double max_speed_ratio,
  std::chrono::milliseconds lease_timeout,
  bool publish_clock)
: node_(node),
  participant_id_(std::move(participant_id)),
  lease_timeout_(lease_timeout),
  publish_clock_(publish_clock),
  steady_start_(std::chrono::steady_clock::now()),
  core_(0, max_speed_ratio, std::chrono::duration_cast<std::chrono::nanoseconds>(lease_timeout).count())
{
  TimeTransportOptions options;
  options.participant_id = participant_id_;
  transport_ = create_time_transport("ecal", options);

  if (publish_clock_) {
    clock_pub_ = node_.create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
  }
}

void TimeParticipant::start()
{
  next_safe_time_ns_ = kInfiniteTimeNs;
  intent_state_ = fss_time_interfaces::msg::TimeIntent::STATE_IDLE;
  transport_->start(
    [this](const fss_time_interfaces::msg::TimeIntent & msg) { on_intent(msg); },
    [this](const fss_time_interfaces::msg::TimeControl & msg) { on_control(msg); });
  publish_intent(intent_state_, next_safe_time_ns_);
  tick_timer_ = node_.create_wall_timer(std::chrono::milliseconds(5), [this]() { tick(); });
  RCLCPP_INFO(
    node_.get_logger(),
    "fss_time participant '%s' uses %s transport",
    participant_id_.c_str(),
    transport_->name().c_str());
}

void TimeParticipant::announce_next_safe_time(const rclcpp::Time & next_safe_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  next_safe_time_ns_ = next_safe_time.nanoseconds();
  intent_state_ = fss_time_interfaces::msg::TimeIntent::STATE_ACTIVE;
}

void TimeParticipant::announce_idle()
{
  std::lock_guard<std::mutex> lock(mutex_);
  next_safe_time_ns_ = kInfiniteTimeNs;
  intent_state_ = fss_time_interfaces::msg::TimeIntent::STATE_IDLE;
}

void TimeParticipant::announce_leaving()
{
  int64_t current_time_ns = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_time_ns = core_.current_time_ns();
  }
  publish_intent(fss_time_interfaces::msg::TimeIntent::STATE_LEAVING, current_time_ns);
}

rclcpp::Time TimeParticipant::now() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rclcpp::Time(core_.current_time_ns(), RCL_ROS_TIME);
}

int64_t TimeParticipant::steady_now_ns() const
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now() - steady_start_).count();
}

void TimeParticipant::publish_intent(uint8_t state, int64_t next_safe_time_ns)
{
  fss_time_interfaces::msg::TimeIntent msg;
  int64_t steady_now = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    steady_now = steady_now_ns();
    msg.participant_id = participant_id_;
    msg.epoch = ++epoch_;
    msg.current_time = from_ns(core_.current_time_ns());
    msg.next_safe_time = from_ns(next_safe_time_ns);
    msg.lookahead = duration_from_ns(std::max<int64_t>(0, next_safe_time_ns - core_.current_time_ns()));
    msg.state = state;
    msg.lease_deadline_steady_ns =
      static_cast<uint64_t>(steady_now + std::chrono::duration_cast<std::chrono::nanoseconds>(lease_timeout_).count());
    core_.observe_intent(msg, steady_now);
  }
  transport_->publish_intent(msg);
}

void TimeParticipant::tick()
{
  uint8_t intent_state = fss_time_interfaces::msg::TimeIntent::STATE_IDLE;
  int64_t next_safe_time_ns = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    intent_state = intent_state_;
    next_safe_time_ns = next_safe_time_ns_;
  }
  publish_intent(intent_state, next_safe_time_ns);
  GrantResult result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result = core_.compute_grant(steady_now_ns());
  }
  if (publish_clock_ && result.advanced) {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = from_ns(result.grant_time_ns);
    clock_pub_->publish(clock);
  }
}

void TimeParticipant::publish_control(const fss_time_interfaces::msg::TimeControl & control)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    core_.apply_control(control, steady_now_ns());
  }
  transport_->publish_control(control);
}

void TimeParticipant::on_intent(const fss_time_interfaces::msg::TimeIntent & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  core_.observe_intent(msg, steady_now_ns());
}

void TimeParticipant::on_control(const fss_time_interfaces::msg::TimeControl & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  core_.apply_control(msg, steady_now_ns());
}

}  // namespace fss_time
