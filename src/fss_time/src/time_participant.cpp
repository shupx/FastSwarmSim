#include "fss_time/time_participant.hpp"

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
  auto intent_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local();
  intent_pub_ = node_.create_publisher<fss_time_interfaces::msg::TimeIntent>(
    "/fss/time_intent", intent_qos);
  intent_sub_ = node_.create_subscription<fss_time_interfaces::msg::TimeIntent>(
    "/fss/time_intent", intent_qos,
    [this](const fss_time_interfaces::msg::TimeIntent::SharedPtr msg) { on_intent(msg); });

  control_sub_ = node_.create_subscription<fss_time_interfaces::msg::TimeControl>(
    "/fss/time_control", rclcpp::QoS(rclcpp::KeepLast(20)).reliable().transient_local(),
    [this](const fss_time_interfaces::msg::TimeControl::SharedPtr msg) { on_control(msg); });

  if (publish_clock_) {
    clock_pub_ = node_.create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
  }
}

void TimeParticipant::start()
{
  next_safe_time_ns_ = kInfiniteTimeNs;
  intent_state_ = fss_time_interfaces::msg::TimeIntent::STATE_IDLE;
  publish_intent(intent_state_, next_safe_time_ns_);
  tick_timer_ = node_.create_wall_timer(std::chrono::milliseconds(5), [this]() { tick(); });
}

void TimeParticipant::announce_next_safe_time(const rclcpp::Time & next_safe_time)
{
  next_safe_time_ns_ = next_safe_time.nanoseconds();
  intent_state_ = fss_time_interfaces::msg::TimeIntent::STATE_ACTIVE;
  publish_intent(intent_state_, next_safe_time_ns_);
}

void TimeParticipant::announce_idle()
{
  next_safe_time_ns_ = kInfiniteTimeNs;
  intent_state_ = fss_time_interfaces::msg::TimeIntent::STATE_IDLE;
  publish_intent(intent_state_, next_safe_time_ns_);
}

void TimeParticipant::announce_leaving()
{
  publish_intent(fss_time_interfaces::msg::TimeIntent::STATE_LEAVING, core_.current_time_ns());
}

rclcpp::Time TimeParticipant::now() const
{
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
  msg.participant_id = participant_id_;
  msg.epoch = ++epoch_;
  msg.current_time = from_ns(core_.current_time_ns());
  msg.next_safe_time = from_ns(next_safe_time_ns);
  msg.lookahead = duration_from_ns(std::max<int64_t>(0, next_safe_time_ns - core_.current_time_ns()));
  msg.state = state;
  msg.lease_deadline_steady_ns =
    static_cast<uint64_t>(steady_now_ns() + std::chrono::duration_cast<std::chrono::nanoseconds>(lease_timeout_).count());
  intent_pub_->publish(msg);
  core_.observe_intent(msg, steady_now_ns());
}

void TimeParticipant::tick()
{
  publish_intent(intent_state_, next_safe_time_ns_);
  const auto result = core_.compute_grant(steady_now_ns());
  if (publish_clock_ && result.advanced) {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = from_ns(result.grant_time_ns);
    clock_pub_->publish(clock);
  }
}

void TimeParticipant::on_intent(const fss_time_interfaces::msg::TimeIntent::SharedPtr msg)
{
  core_.observe_intent(*msg, steady_now_ns());
}

void TimeParticipant::on_control(const fss_time_interfaces::msg::TimeControl::SharedPtr msg)
{
  core_.apply_control(*msg, steady_now_ns());
}

}  // namespace fss_time
