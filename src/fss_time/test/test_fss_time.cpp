#include "fss_time/executors.hpp"
#include "fss_time/sim_time_broker.hpp"
#include "fss_time/thread_time_participant.hpp"
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
  auto broker_node = std::make_shared<rclcpp::Node>("thread_time_same_thread_broker_test");
  broker_node->declare_parameter("sim_time_broker_endpoint", endpoint);
  fss_time::SimTimeBroker broker(*broker_node);
  broker.start();

  rclcpp::Node node("thread_time_same_thread_test");
  node.declare_parameter("sim_time_broker_endpoint", endpoint);
  fss_time::thread_time_participant::reset_current_thread_for_testing();
  auto & first = fss_time::thread_time_participant::for_current_thread(node, "same_thread");
  auto & second = fss_time::thread_time_participant::for_current_thread(node, "same_thread");
  EXPECT_EQ(&first, &second);
  fss_time::thread_time_participant::reset_current_thread_for_testing();
}

TEST(ThreadTimeParticipant, DifferentThreadsReturnDifferentParticipants)
{
  const auto endpoint = reserve_test_endpoint();
  auto broker_node = std::make_shared<rclcpp::Node>("thread_time_different_thread_broker_test");
  broker_node->declare_parameter("sim_time_broker_endpoint", endpoint);
  fss_time::SimTimeBroker broker(*broker_node);
  broker.start();

  rclcpp::Node node("thread_time_different_thread_test");
  node.declare_parameter("sim_time_broker_endpoint", endpoint);
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

TEST(ThreadTimeParticipant, WaitsForBrokerBeforeReturning)
{
  const auto endpoint = reserve_test_endpoint();
  std::promise<void> participant_entered_promise;
  auto participant_entered = participant_entered_promise.get_future();
  std::promise<void> participant_registered_promise;
  auto participant_registered = participant_registered_promise.get_future();

  std::thread participant_thread([endpoint, &participant_entered_promise, &participant_registered_promise]() {
    rclcpp::Node node("thread_time_waits_for_broker_test");
    node.declare_parameter("sim_time_broker_endpoint", endpoint);
    fss_time::thread_time_participant::reset_current_thread_for_testing();
    participant_entered_promise.set_value();
    auto & participant = fss_time::thread_time_participant::for_current_thread(node, "wait_for_broker");
    participant_registered_promise.set_value();
    participant.unregister_participant();
    fss_time::thread_time_participant::reset_current_thread_for_testing();
  });

  ASSERT_EQ(participant_entered.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(participant_registered.wait_for(std::chrono::milliseconds(150)), std::future_status::timeout);

  auto broker_node = std::make_shared<rclcpp::Node>("thread_time_delayed_broker_test");
  broker_node->declare_parameter("sim_time_broker_endpoint", endpoint);
  fss_time::SimTimeBroker broker(*broker_node);
  broker.start();

  EXPECT_EQ(participant_registered.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  participant_thread.join();
}

TEST(SimTimeBroker, ParticipantAnnouncementAdvancesClockAndStatus)
{
  const auto endpoint = reserve_test_endpoint();
  auto broker_node = std::make_shared<rclcpp::Node>("sim_time_broker_status_test");
  broker_node->declare_parameter("sim_time_broker_endpoint", endpoint);
  broker_node->declare_parameter("speed_regulator_step_ns", 10000000);
  broker_node->declare_parameter("max_real_time_factor", 1.0);
  broker_node->declare_parameter("auto_start", true);
  fss_time::SimTimeBroker broker(*broker_node);
  broker.start();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(broker_node);
  std::atomic<bool> stop_executor{false};
  std::thread spin_thread([&executor, &stop_executor]() {
    while (!stop_executor.load()) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  rclcpp::Node participant_node("sim_time_participant_test");
  participant_node.declare_parameter("sim_time_broker_endpoint", endpoint);

  fss_time::thread_time_participant::reset_current_thread_for_testing();
  auto & participant =
    fss_time::thread_time_participant::for_current_thread(participant_node, "broker_test");
  EXPECT_EQ(participant.get_last_safe_time().nanoseconds(), 0);
  participant.announce_next_safe_time(rclcpp::Time(10000000LL, RCL_ROS_TIME));
  EXPECT_EQ(participant.get_last_safe_time().nanoseconds(), 10000000LL);
  participant.announce_next_safe_time(rclcpp::Time(5000000LL, RCL_ROS_TIME));
  EXPECT_EQ(participant.get_last_safe_time().nanoseconds(), 10000000LL);
  participant.announce_next_safe_time(rclcpp::Time(fss_time::kInfiniteTimeNs, RCL_ROS_TIME));
  EXPECT_EQ(participant.get_last_safe_time().nanoseconds(), fss_time::kInfiniteTimeNs);
  participant.announce_next_safe_time(rclcpp::Time(20000000LL, RCL_ROS_TIME));
  EXPECT_EQ(participant.get_last_safe_time().nanoseconds(), 20000000LL);

  wait_until([&broker]() {
    return broker.status_message().participant_count == 1 &&
           broker.status_message().sim_time.nanosec > 0;
  }, std::chrono::seconds(1));

  const auto status = broker.status_message();
  EXPECT_TRUE(status.running);
  EXPECT_EQ(status.participant_count, 1u);
  EXPECT_LE(status.new_request_participant_count, status.participant_count);
  EXPECT_GT(status.sim_time.nanosec, 0u);

  participant.unregister_participant();
  wait_until([&broker]() {
    return broker.status_message().participant_count == 0;
  }, std::chrono::seconds(1));
  EXPECT_EQ(broker.status_message().participant_count, 0u);
  EXPECT_EQ(broker.status_message().new_request_participant_count, 0u);
  fss_time::thread_time_participant::reset_current_thread_for_testing();

  stop_executor = true;
  spin_thread.join();
  executor.remove_node(broker_node);
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
  auto broker_node = std::make_shared<rclcpp::Node>("fss_single_executor_broker_node");
  broker_node->declare_parameter("sim_time_broker_endpoint", endpoint);
  broker_node->declare_parameter("auto_start", true);
  fss_time::SimTimeBroker broker(*broker_node);
  broker.start();

  auto node = std::make_shared<rclcpp::Node>("fss_single_executor_sim_time_node");
  node->declare_parameter("sim_time_broker_endpoint", endpoint);
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
