#include <gtest/gtest.h>

#include <string>

#include "fss_time/time_transport_serialization.hpp"
#include "fss_time/time_types.hpp"

namespace
{

TEST(TimeTransportSerialization, IntentRoundTripPreservesAllFields)
{
  fss_time_interfaces::msg::TimeIntent intent;
  intent.participant_id = "machine_a/uav_01";
  intent.epoch = 42;
  intent.current_time = fss_time::from_ns(123456789);
  intent.next_safe_time = fss_time::from_ns(223456789);
  intent.lookahead = fss_time::duration_from_ns(100000000);
  intent.state = fss_time_interfaces::msg::TimeIntent::STATE_ACTIVE;
  intent.lease_deadline_steady_ns = 987654321;

  const auto payload = fss_time::serialize_time_intent(intent);

  fss_time_interfaces::msg::TimeIntent decoded;
  ASSERT_TRUE(fss_time::deserialize_time_intent(payload.data(), payload.size(), decoded));

  EXPECT_EQ(decoded.participant_id, intent.participant_id);
  EXPECT_EQ(decoded.epoch, intent.epoch);
  EXPECT_EQ(fss_time::to_ns(decoded.current_time), fss_time::to_ns(intent.current_time));
  EXPECT_EQ(fss_time::to_ns(decoded.next_safe_time), fss_time::to_ns(intent.next_safe_time));
  EXPECT_EQ(fss_time::duration_to_ns(decoded.lookahead), fss_time::duration_to_ns(intent.lookahead));
  EXPECT_EQ(decoded.state, intent.state);
  EXPECT_EQ(decoded.lease_deadline_steady_ns, intent.lease_deadline_steady_ns);
}

TEST(TimeTransportSerialization, IntentRoundTripPreservesDelimiterInParticipantId)
{
  fss_time_interfaces::msg::TimeIntent intent;
  intent.participant_id = "host_a\nuav|with spaces";
  intent.epoch = 7;
  intent.current_time = fss_time::from_ns(1);
  intent.next_safe_time = fss_time::from_ns(2);
  intent.lookahead = fss_time::duration_from_ns(1);
  intent.state = fss_time_interfaces::msg::TimeIntent::STATE_IDLE;
  intent.lease_deadline_steady_ns = 3;

  const auto payload = fss_time::serialize_time_intent(intent);

  fss_time_interfaces::msg::TimeIntent decoded;
  ASSERT_TRUE(fss_time::deserialize_time_intent(payload.data(), payload.size(), decoded));
  EXPECT_EQ(decoded.participant_id, intent.participant_id);
}

TEST(TimeTransportSerialization, ControlRoundTripPreservesAllFields)
{
  fss_time_interfaces::msg::TimeControl control;
  control.epoch = 11;
  control.command = fss_time_interfaces::msg::TimeControl::COMMAND_SET_SPEED;
  control.max_speed_ratio = 20.5;
  control.reset_time = fss_time::from_ns(33445566);

  const auto payload = fss_time::serialize_time_control(control);

  fss_time_interfaces::msg::TimeControl decoded;
  ASSERT_TRUE(fss_time::deserialize_time_control(payload.data(), payload.size(), decoded));

  EXPECT_EQ(decoded.epoch, control.epoch);
  EXPECT_EQ(decoded.command, control.command);
  EXPECT_DOUBLE_EQ(decoded.max_speed_ratio, control.max_speed_ratio);
  EXPECT_EQ(fss_time::to_ns(decoded.reset_time), fss_time::to_ns(control.reset_time));
}

TEST(TimeTransportSerialization, RejectsInvalidPayloads)
{
  fss_time_interfaces::msg::TimeIntent intent;
  EXPECT_FALSE(fss_time::deserialize_time_intent(nullptr, 0, intent));

  const std::string bad_intent = "not-an-intent\n";
  EXPECT_FALSE(fss_time::deserialize_time_intent(bad_intent.data(), bad_intent.size(), intent));

  fss_time_interfaces::msg::TimeControl control;
  const std::string bad_control = "not-a-control\n";
  EXPECT_FALSE(fss_time::deserialize_time_control(bad_control.data(), bad_control.size(), control));
}

}  // namespace
