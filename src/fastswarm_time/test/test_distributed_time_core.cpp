#include "fastswarm_time/distributed_time_core.hpp"

#include <gtest/gtest.h>

#include "fastswarm_time/time_types.hpp"

namespace
{

fastswarm_time_interfaces::msg::TimeIntent intent(
  const std::string & id,
  uint64_t epoch,
  int64_t next_safe_time_ns)
{
  fastswarm_time_interfaces::msg::TimeIntent msg;
  msg.participant_id = id;
  msg.epoch = epoch;
  msg.current_time = fastswarm_time::from_ns(0);
  msg.next_safe_time = fastswarm_time::from_ns(next_safe_time_ns);
  msg.lookahead = fastswarm_time::duration_from_ns(next_safe_time_ns);
  msg.state = fastswarm_time_interfaces::msg::TimeIntent::STATE_ACTIVE;
  return msg;
}

}  // namespace

TEST(DistributedTimeCore, GrantsMinimumActiveNextSafeTime)
{
  fastswarm_time::DistributedTimeCore core(0, 0.0, 1000000000);
  core.observe_intent(intent("uav2", 1, 20000000), 0);
  core.observe_intent(intent("uav1", 1, 10000000), 0);

  const auto result = core.compute_grant(1);
  EXPECT_TRUE(result.advanced);
  EXPECT_EQ(result.grant_time_ns, 10000000);
  EXPECT_EQ(result.active_participants, 2u);
}

TEST(DistributedTimeCore, LeaseExpiryUnblocksGrant)
{
  fastswarm_time::DistributedTimeCore core(0, 0.0, 10);
  core.observe_intent(intent("alive", 1, 50000000), 5);
  core.observe_intent(intent("expired", 1, 10000000), 0);

  const auto result = core.compute_grant(11);
  EXPECT_TRUE(result.advanced);
  EXPECT_EQ(result.grant_time_ns, 50000000);
  EXPECT_EQ(result.active_participants, 1u);
}

TEST(DistributedTimeCore, SpeedCapLimitsGrant)
{
  fastswarm_time::DistributedTimeCore core(0, 2.0, 1000000000);
  core.observe_intent(intent("uav1", 1, 100000000), 0);

  const auto result = core.compute_grant(10000000);
  EXPECT_TRUE(result.advanced);
  EXPECT_EQ(result.grant_time_ns, 20000000);
}

TEST(DistributedTimeCore, StaleControlEpochIgnored)
{
  fastswarm_time::DistributedTimeCore core(0, 1.0, 1000000000);

  fastswarm_time_interfaces::msg::TimeControl pause;
  pause.epoch = 2;
  pause.command = fastswarm_time_interfaces::msg::TimeControl::COMMAND_PAUSE;
  EXPECT_TRUE(core.apply_control(pause, 0));
  EXPECT_TRUE(core.paused());

  fastswarm_time_interfaces::msg::TimeControl resume;
  resume.epoch = 1;
  resume.command = fastswarm_time_interfaces::msg::TimeControl::COMMAND_RESUME;
  EXPECT_FALSE(core.apply_control(resume, 0));
  EXPECT_TRUE(core.paused());
}
