#include "fss_time/executors.hpp"
#include "fss_time/rate.hpp"
#include "fss_time/time_coordinator.hpp"
#include "fss_time/thread_time_participant.hpp"
#include "fss_time/sleep.hpp"
#include "fss_time/time_types.hpp"

#include <atomic>
#include <chrono>
#include <future>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"

namespace
{

std::atomic<int> g_next_test_endpoint{0};

std::string reserve_test_endpoint()
{
  return "ipc:///tmp/fss_time_test_" + std::to_string(g_next_test_endpoint.fetch_add(1)) + ".ipc";
}

std::string reserve_tcp_test_endpoint()
{
  return "tcp://127.0.0.1:" + std::to_string(28000 + g_next_test_endpoint.fetch_add(1));
}

void wait_until(const std::function<bool()> & predicate, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

}  // namespace

::testing::Environment * const g_rclcpp_environment =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

TEST(ThreadTimeParticipant, SameThreadReturnsSameParticipant)
{
  const auto endpoint = reserve_test_endpoint();
  auto coordinator_node = std::make_shared<rclcpp::Node>("thread_time_same_thread_coordinator_test");
  coordinator_node->declare_parameter("fss_time_coordinator_endpoint", endpoint);
  fss_time::TimeCoordinator coordinator(*coordinator_node);
  coordinator.start();

  rclcpp::Node node("thread_time_same_thread_test");
  node.declare_parameter("fss_time_coordinator_endpoint", endpoint);
  fss_time::thread_time_participant::reset_current_thread_for_testing();
  auto & first = fss_time::thread_time_participant::for_current_thread(node, "same_thread");
  auto & second = fss_time::thread_time_participant::for_current_thread(node, "same_thread");
  EXPECT_EQ(&first, &second);
  fss_time::thread_time_participant::reset_current_thread_for_testing();
}

TEST(ThreadTimeParticipant, DifferentThreadsReturnDifferentParticipants)
{
  const auto endpoint = reserve_test_endpoint();
  auto coordinator_node = std::make_shared<rclcpp::Node>("thread_time_different_thread_coordinator_test");
  coordinator_node->declare_parameter("fss_time_coordinator_endpoint", endpoint);
  fss_time::TimeCoordinator coordinator(*coordinator_node);
  coordinator.start();

  rclcpp::Node node("thread_time_different_thread_test");
  node.declare_parameter("fss_time_coordinator_endpoint", endpoint);
  fss_time::thread_time_participant::reset_current_thread_for_testing();
  auto & first = fss_time::thread_time_participant::for_current_thread(node, "thread_one");

  std::promise<const void *> other_promise;
  auto other_future = other_promise.get_future();
  std::thread other_thread([&node, &other_promise]() {
    fss_time::thread_time_participant::reset_current_thread_for_testing();
    auto & second = fss_time::thread_time_participant::for_current_thread(node, "thread_two");
    other_promise.set_value(&second);
    fss_time::thread_time_participant::reset_current_thread_for_testing();
  });
  other_thread.join();

  EXPECT_NE(&first, other_future.get());
  fss_time::thread_time_participant::reset_current_thread_for_testing();
}

TEST(ThreadTimeParticipant, WaitsForCoordinatorBeforeReturning)
{
  const auto endpoint = reserve_test_endpoint();
  std::promise<void> participant_entered_promise;
  auto participant_entered = participant_entered_promise.get_future();
  std::promise<void> participant_registered_promise;
  auto participant_registered = participant_registered_promise.get_future();

  std::thread participant_thread([endpoint, &participant_entered_promise, &participant_registered_promise]() {
    rclcpp::Node node("thread_time_waits_for_coordinator_test");
    node.declare_parameter("fss_time_coordinator_endpoint", endpoint);
    fss_time::thread_time_participant::reset_current_thread_for_testing();
    participant_entered_promise.set_value();
    auto & participant = fss_time::thread_time_participant::for_current_thread(node, "wait_for_coordinator");
    participant_registered_promise.set_value();
    participant.unregister_participant();
    fss_time::thread_time_participant::reset_current_thread_for_testing();
  });

  ASSERT_EQ(participant_entered.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(participant_registered.wait_for(std::chrono::milliseconds(150)), std::future_status::timeout);

  auto coordinator_node = std::make_shared<rclcpp::Node>("thread_time_delayed_coordinator_test");
  coordinator_node->declare_parameter("fss_time_coordinator_endpoint", endpoint);
  fss_time::TimeCoordinator coordinator(*coordinator_node);
  coordinator.start();

  EXPECT_EQ(participant_registered.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  participant_thread.join();
}

TEST(TimeCoordinator, SupportsTcpEndpoint)
{
  const auto endpoint = reserve_tcp_test_endpoint();
  auto coordinator_node = std::make_shared<rclcpp::Node>("time_coordinator_tcp_endpoint_test");
  coordinator_node->declare_parameter("fss_time_coordinator_endpoint", endpoint);
  fss_time::TimeCoordinator coordinator(*coordinator_node);
  coordinator.start();

  rclcpp::Node participant_node("sim_time_participant_tcp_endpoint_test");
  participant_node.declare_parameter("fss_time_coordinator_endpoint", endpoint);

  fss_time::thread_time_participant::reset_current_thread_for_testing();
  auto & participant =
    fss_time::thread_time_participant::for_current_thread(participant_node, "tcp_endpoint");

  EXPECT_EQ(coordinator.status_message().participant_count, 1u);

  participant.unregister_participant();
  wait_until([&coordinator]() {
    return coordinator.status_message().participant_count == 0;
  }, std::chrono::seconds(1));
  fss_time::thread_time_participant::reset_current_thread_for_testing();
}

TEST(TimeCoordinator, ChildCoordinatorAdvancesFromParentGrant)
{
  const auto parent_endpoint = reserve_test_endpoint();
  const auto parent_pub_endpoint = reserve_test_endpoint();
  const auto child_endpoint = reserve_test_endpoint();
  const auto child_pub_endpoint = reserve_test_endpoint();

  auto parent_node = std::make_shared<rclcpp::Node>("time_coordinator_cascade_parent_test");
  parent_node->declare_parameter("fss_time_coordinator_endpoint", parent_endpoint);
  parent_node->declare_parameter("fss_time_coordinator_pub_endpoint", parent_pub_endpoint);
  parent_node->declare_parameter("speed_regulator_step_ns", 1000000);
  parent_node->declare_parameter("auto_start", true);
  parent_node->declare_parameter("follows_real_time", false);
  fss_time::TimeCoordinator parent_coordinator(*parent_node);
  parent_coordinator.start();

  auto child_node = std::make_shared<rclcpp::Node>("time_coordinator_cascade_child_test");
  child_node->declare_parameter("fss_time_coordinator_endpoint", child_endpoint);
  child_node->declare_parameter("fss_time_coordinator_pub_endpoint", child_pub_endpoint);
  child_node->declare_parameter("fss_time_parent_coordinator_endpoint", parent_endpoint);
  child_node->declare_parameter("speed_regulator_step_ns", 1000000);
  child_node->declare_parameter("auto_start", true);
  child_node->declare_parameter("follows_real_time", false);
  fss_time::TimeCoordinator child_coordinator(*child_node);
  child_coordinator.start();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(parent_node);
  executor.add_node(child_node);
  std::atomic<bool> stop_executor{false};
  std::thread spin_thread([&executor, &stop_executor]() {
    while (!stop_executor.load()) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  rclcpp::Node participant_node("time_coordinator_cascade_participant_test");
  participant_node.declare_parameter("fss_time_coordinator_endpoint", child_endpoint);

  fss_time::thread_time_participant::reset_current_thread_for_testing();
  auto & participant =
    fss_time::thread_time_participant::for_current_thread(participant_node, "cascade_child");
  wait_until([&child_coordinator]() {
    return child_coordinator.status_message().participant_count == 1;
  }, std::chrono::seconds(1));

  participant.announce_next_safe_time(rclcpp::Time(5000000LL, RCL_ROS_TIME));
  wait_until([&child_coordinator]() {
    return fss_time::to_ns(child_coordinator.status_message().sim_time) > 0;
  }, std::chrono::seconds(2));

  EXPECT_GT(fss_time::to_ns(child_coordinator.status_message().sim_time), 0);
  EXPECT_LE(fss_time::to_ns(child_coordinator.status_message().sim_time), 5000000LL);
  EXPECT_EQ(parent_coordinator.status_message().participant_count, 1u);

  participant.unregister_participant();
  wait_until([&child_coordinator]() {
    return child_coordinator.status_message().participant_count == 0;
  }, std::chrono::seconds(1));
  fss_time::thread_time_participant::reset_current_thread_for_testing();

  stop_executor = true;
  spin_thread.join();
  executor.remove_node(child_node);
  executor.remove_node(parent_node);
}

TEST(TimeCoordinator, ParticipantAnnouncementAdvancesClockAndStatus)
{
  const auto endpoint = reserve_test_endpoint();
  auto coordinator_node = std::make_shared<rclcpp::Node>("time_coordinator_status_test");
  coordinator_node->declare_parameter("fss_time_coordinator_endpoint", endpoint);
  coordinator_node->declare_parameter("speed_regulator_step_ns", 5000000);
  coordinator_node->declare_parameter("max_real_time_factor", 1.0);
  coordinator_node->declare_parameter("auto_start", true);
  fss_time::TimeCoordinator coordinator(*coordinator_node);
  coordinator.start();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(coordinator_node);
  std::atomic<bool> stop_executor{false};
  std::thread spin_thread([&executor, &stop_executor]() {
    while (!stop_executor.load()) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  rclcpp::Node participant_node("sim_time_participant_test");
  participant_node.declare_parameter("fss_time_coordinator_endpoint", endpoint);

  fss_time::thread_time_participant::reset_current_thread_for_testing();
  auto & participant =
    fss_time::thread_time_participant::for_current_thread(participant_node, "coordinator_test");
  EXPECT_EQ(participant.get_last_safe_time().nanoseconds(), 0);
  participant.announce_next_safe_time(rclcpp::Time(10000000LL, RCL_ROS_TIME));
  EXPECT_EQ(participant.get_last_safe_time().nanoseconds(), 10000000LL);
  participant.announce_next_safe_time(rclcpp::Time(5000000LL, RCL_ROS_TIME));
  EXPECT_EQ(participant.get_last_safe_time().nanoseconds(), 5000000LL);
  participant.announce_next_safe_time(rclcpp::Time(fss_time::kInfiniteTimeNs, RCL_ROS_TIME));
  EXPECT_EQ(participant.get_last_safe_time().nanoseconds(), fss_time::kInfiniteTimeNs);
  participant.announce_next_safe_time(rclcpp::Time(20000000LL, RCL_ROS_TIME));
  EXPECT_EQ(participant.get_last_safe_time().nanoseconds(), 20000000LL);

  wait_until([&coordinator]() {
    return coordinator.status_message().participant_count == 1 &&
           coordinator.status_message().sim_time.nanosec > 0;
  }, std::chrono::seconds(1));

  const auto status = coordinator.status_message();
  EXPECT_TRUE(status.running);
  EXPECT_EQ(status.participant_count, 1u);
  EXPECT_LE(status.new_request_participant_count, status.participant_count);
  EXPECT_GT(status.sim_time.nanosec, 0u);
  EXPECT_EQ(status.speed_regulator_step_ns, 5000000);
  EXPECT_EQ(status.min_operation_walltime, 100000);

  participant.unregister_participant();
  wait_until([&coordinator]() {
    return coordinator.status_message().participant_count == 0;
  }, std::chrono::seconds(1));
  EXPECT_EQ(coordinator.status_message().participant_count, 0u);
  EXPECT_EQ(coordinator.status_message().new_request_participant_count, 0u);
  EXPECT_NE(
    coordinator.status_message().debug_msg.find("publish_clock_locked skipped"),
    std::string::npos);
  fss_time::thread_time_participant::reset_current_thread_for_testing();

  stop_executor = true;
  spin_thread.join();
  executor.remove_node(coordinator_node);
}

TEST(TimeCoordinator, RealTimeParticipantRequestCatchesUpToWallTime)
{
  const auto endpoint = reserve_test_endpoint();
  auto coordinator_node = std::make_shared<rclcpp::Node>("time_coordinator_real_time_catchup_test");
  coordinator_node->declare_parameter("fss_time_coordinator_endpoint", endpoint);
  coordinator_node->declare_parameter("speed_regulator_step_ns", 5000000);
  coordinator_node->declare_parameter("max_real_time_factor", 1.0);
  coordinator_node->declare_parameter("auto_start", true);
  fss_time::TimeCoordinator coordinator(*coordinator_node);
  coordinator.start();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(coordinator_node);
  std::atomic<bool> stop_executor{false};
  std::thread spin_thread([&executor, &stop_executor]() {
    while (!stop_executor.load()) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  rclcpp::Node participant_node("sim_time_real_time_catchup_participant_test");
  participant_node.declare_parameter("fss_time_coordinator_endpoint", endpoint);

  fss_time::thread_time_participant::reset_current_thread_for_testing();
  auto & participant =
    fss_time::thread_time_participant::for_current_thread(participant_node, "real_time_catchup");
  wait_until([&coordinator]() {
    return coordinator.status_message().participant_count == 1;
  }, std::chrono::seconds(1));

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  participant.announce_next_safe_time(rclcpp::Time(1000000LL, RCL_ROS_TIME));
  wait_until([&coordinator]() {
    return fss_time::to_ns(coordinator.status_message().sim_time) > 20000000LL;
  }, std::chrono::seconds(1));

  EXPECT_GT(fss_time::to_ns(coordinator.status_message().sim_time), 20000000LL);

  participant.unregister_participant();
  wait_until([&coordinator]() {
    return coordinator.status_message().participant_count == 0;
  }, std::chrono::seconds(1));
  fss_time::thread_time_participant::reset_current_thread_for_testing();

  stop_executor = true;
  spin_thread.join();
  executor.remove_node(coordinator_node);
}

TEST(TimeCoordinator, ParticipantCanDisableRealTimeFollowing)
{
  const auto endpoint = reserve_test_endpoint();
  auto coordinator_node = std::make_shared<rclcpp::Node>(
    "time_coordinator_participant_real_time_setting_test");
  coordinator_node->declare_parameter("fss_time_coordinator_endpoint", endpoint);
  coordinator_node->declare_parameter("speed_regulator_step_ns", 5000000);
  coordinator_node->declare_parameter("max_real_time_factor", 1.0);
  coordinator_node->declare_parameter("auto_start", true);
  fss_time::TimeCoordinator coordinator(*coordinator_node);
  coordinator.start();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(coordinator_node);
  std::atomic<bool> stop_executor{false};
  std::thread spin_thread([&executor, &stop_executor]() {
    while (!stop_executor.load()) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  rclcpp::Node participant_node("participant_real_time_setting_test");
  participant_node.declare_parameter("fss_time_coordinator_endpoint", endpoint);
  fss_time::thread_time_participant::reset_current_thread_for_testing();
  auto & participant =
    fss_time::thread_time_participant::for_current_thread(participant_node, "no_real_time_following");
  wait_until([&coordinator]() {
    return coordinator.status_message().participant_count == 1;
  }, std::chrono::seconds(1));

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  participant.set_follows_real_time(false);
  participant.announce_next_safe_time(rclcpp::Time(1000000LL, RCL_ROS_TIME));
  wait_until([&coordinator]() {
    return fss_time::to_ns(coordinator.status_message().sim_time) >= 1000000LL;
  }, std::chrono::seconds(1));
  EXPECT_EQ(fss_time::to_ns(coordinator.status_message().sim_time), 1000000LL);

  participant.unregister_participant();
  fss_time::thread_time_participant::reset_current_thread_for_testing();
  stop_executor = true;
  spin_thread.join();
  executor.remove_node(coordinator_node);
}

TEST(TimeCoordinator, RejectsInvalidParticipantRealTimeSetting)
{
  const auto endpoint = reserve_test_endpoint();
  auto coordinator_node = std::make_shared<rclcpp::Node>(
    "time_coordinator_invalid_participant_real_time_setting_test");
  coordinator_node->declare_parameter("fss_time_coordinator_endpoint", endpoint);
  fss_time::TimeCoordinator coordinator(*coordinator_node);
  coordinator.start();

  fss_time::ZeroMqTimeParticipantOptions options;
  options.participant_id = "invalid_participant_real_time_setting";
  options.coordinator_endpoint = endpoint;
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  fss_time::ZeroMqTimeParticipantBackend participant(options, clock);
  participant.register_participant();

  EXPECT_EQ(
    participant.request_coordinator("SET_FOLLOWS_REAL_TIME 2", true),
    "ERROR invalid follows_real_time setting");
  participant.set_follows_real_time(false);
  participant.unregister_participant();
}

TEST(TimeCoordinator, FollowsRealTimeParameterDisablesWallTimeCatchUp)
{
  const auto endpoint = reserve_test_endpoint();
  auto coordinator_node = std::make_shared<rclcpp::Node>("time_coordinator_no_real_time_catchup_test");
  coordinator_node->declare_parameter("fss_time_coordinator_endpoint", endpoint);
  coordinator_node->declare_parameter("speed_regulator_step_ns", 5000000);
  coordinator_node->declare_parameter("max_real_time_factor", 1.0);
  coordinator_node->declare_parameter("auto_start", true);
  coordinator_node->declare_parameter("follows_real_time", false);
  fss_time::TimeCoordinator coordinator(*coordinator_node);
  coordinator.start();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(coordinator_node);
  std::atomic<bool> stop_executor{false};
  std::thread spin_thread([&executor, &stop_executor]() {
    while (!stop_executor.load()) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  rclcpp::Node participant_node("sim_time_no_real_time_catchup_participant_test");
  participant_node.declare_parameter("fss_time_coordinator_endpoint", endpoint);

  fss_time::thread_time_participant::reset_current_thread_for_testing();
  auto & participant =
    fss_time::thread_time_participant::for_current_thread(participant_node, "no_real_time_catchup");
  wait_until([&coordinator]() {
    return coordinator.status_message().participant_count == 1;
  }, std::chrono::seconds(1));

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  participant.announce_next_safe_time(rclcpp::Time(1000000LL, RCL_ROS_TIME));
  wait_until([&coordinator]() {
    return fss_time::to_ns(coordinator.status_message().sim_time) >= 1000000LL;
  }, std::chrono::seconds(1));

  EXPECT_LT(fss_time::to_ns(coordinator.status_message().sim_time), 20000000LL);

  participant.unregister_participant();
  wait_until([&coordinator]() {
    return coordinator.status_message().participant_count == 0;
  }, std::chrono::seconds(1));
  fss_time::thread_time_participant::reset_current_thread_for_testing();

  stop_executor = true;
  spin_thread.join();
  executor.remove_node(coordinator_node);
}

TEST(TimeSleep, SleepForWithoutFssSimTimeBehavesLikeClock)
{
  auto node = std::make_shared<rclcpp::Node>("fss_sleep_without_sim_time_node");
  node->declare_parameter("use_fss_sim_time", false);

  EXPECT_TRUE(fss_time::sleep_for(*node, rclcpp::Duration(0, 1000000)));
}

TEST(TimeSleep, SleepUntilWithFssSimTimeAnnouncesEndTime)
{
  const auto endpoint = reserve_test_endpoint();
  auto coordinator_node = std::make_shared<rclcpp::Node>("fss_sleep_until_coordinator_node");
  coordinator_node->declare_parameter("fss_time_coordinator_endpoint", endpoint);
  coordinator_node->declare_parameter("auto_start", true);
  fss_time::TimeCoordinator coordinator(*coordinator_node);
  coordinator.start();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(coordinator_node);
  auto node = std::make_shared<rclcpp::Node>("fss_sleep_until_sim_time_node");
  node->declare_parameter("fss_time_coordinator_endpoint", endpoint);
  node->declare_parameter("use_fss_sim_time", true);
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  executor.add_node(node);
  std::atomic<bool> stop_executor{false};
  std::thread spin_thread([&executor, &stop_executor]() {
    while (!stop_executor.load()) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  const rclcpp::Time until(5000000LL, RCL_ROS_TIME);

  fss_time::thread_time_participant::reset_current_thread_for_testing();
  EXPECT_TRUE(fss_time::sleep_until(*node, until));
  auto & participant = fss_time::thread_time_participant::for_current_thread(*node);
  EXPECT_EQ(participant.get_last_safe_time().nanoseconds(), until.nanoseconds());
  fss_time::thread_time_participant::reset_current_thread_for_testing();

  stop_executor = true;
  spin_thread.join();
  executor.remove_node(node);
  executor.remove_node(coordinator_node);
}

TEST(TimeSleep, RateSleepUsesFssSleepFor)
{
  auto node = std::make_shared<rclcpp::Node>("fss_rate_without_sim_time_node");
  node->declare_parameter("use_fss_sim_time", false);
  fss_time::Rate rate(*node, rclcpp::Duration(0, 1000000));

  EXPECT_TRUE(rate.sleep());
  EXPECT_EQ(rate.get_type(), node->get_clock()->get_clock_type());
  EXPECT_EQ(rate.period(), std::chrono::nanoseconds(1000000));
}

TEST(Executors, PublicIncludeConstructsBothExecutorTypes)
{
  fss_time::executors::SingleThreadedExecutor single_executor;
  fss_time::executors::MultiThreadedExecutor multi_executor(
    rclcpp::ExecutorOptions(),
    2);

  EXPECT_EQ(multi_executor.get_number_of_threads(), 2u);
}

TEST(Executors, SingleThreadedExecutorSpinsWithoutFssSimTime)
{
  auto node = std::make_shared<rclcpp::Node>("fss_single_executor_spin_node");
  node->declare_parameter("use_fss_sim_time", false);

  std::atomic<int> calls{0};
  fss_time::executors::SingleThreadedExecutor executor;
  rclcpp::TimerBase::SharedPtr timer;
  timer = node->create_wall_timer(std::chrono::milliseconds(1), [&executor, &calls]() {
    calls.fetch_add(1);
    executor.cancel();
  });

  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);

  EXPECT_GE(calls.load(), 1);
}

TEST(Executors, NamespaceSpinUsesFssSingleThreadedExecutor)
{
  auto context = std::make_shared<rclcpp::Context>();
  int argc = 0;
  context->init(argc, nullptr);
  rclcpp::NodeOptions node_options;
  node_options.context(context);
  auto node = std::make_shared<rclcpp::Node>("fss_namespace_spin_node", node_options);
  node->declare_parameter("use_fss_sim_time", false);

  std::atomic<int> calls{0};
  rclcpp::TimerBase::SharedPtr timer;
  timer = node->create_wall_timer(std::chrono::milliseconds(1), [context, &calls]() {
    calls.fetch_add(1);
    rclcpp::shutdown(context);
  });

  fss_time::spin(node);

  EXPECT_GE(calls.load(), 1);
}

TEST(Executors, MultiThreadedExecutorSpinsWithoutFssSimTime)
{
  auto node = std::make_shared<rclcpp::Node>("fss_multi_executor_spin_node");
  node->declare_parameter("use_fss_sim_time", false);

  std::atomic<int> calls{0};
  fss_time::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(),
    2);
  rclcpp::TimerBase::SharedPtr timer;
  timer = node->create_wall_timer(std::chrono::milliseconds(1), [&executor, &calls]() {
    calls.fetch_add(1);
    executor.cancel();
  });

  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);

  EXPECT_GE(calls.load(), 1);
}

TEST(Executors, FssSimTimeEnablesRosSimTime)
{
  auto node = std::make_shared<rclcpp::Node>("fss_executor_enables_ros_sim_time_node");
  node->declare_parameter("use_fss_sim_time", true);
  node->set_parameter(rclcpp::Parameter("use_sim_time", false));

  fss_time::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  bool use_sim_time = false;
  ASSERT_TRUE(node->get_parameter("use_sim_time", use_sim_time));
  EXPECT_TRUE(use_sim_time);

  executor.remove_node(node);
}

TEST(Executors, SingleThreadedExecutorSpinsWithFssSimTime)
{
  const auto endpoint = reserve_test_endpoint();
  auto coordinator_node = std::make_shared<rclcpp::Node>("fss_single_executor_coordinator_node");
  coordinator_node->declare_parameter("fss_time_coordinator_endpoint", endpoint);
  coordinator_node->declare_parameter("auto_start", true);
  fss_time::TimeCoordinator coordinator(*coordinator_node);
  coordinator.start();

  auto node = std::make_shared<rclcpp::Node>("fss_single_executor_sim_time_node");
  node->declare_parameter("fss_time_coordinator_endpoint", endpoint);
  node->declare_parameter("use_fss_sim_time", true);

  std::atomic<int> calls{0};
  fss_time::thread_time_participant::reset_current_thread_for_testing();
  fss_time::executors::SingleThreadedExecutor executor;
  rclcpp::TimerBase::SharedPtr timer;
  timer = node->create_wall_timer(std::chrono::milliseconds(1), [&executor, &calls]() {
    calls.fetch_add(1);
    executor.cancel();
  });

  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);

  EXPECT_GE(calls.load(), 1);
  fss_time::thread_time_participant::reset_current_thread_for_testing();
}
