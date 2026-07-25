#include "fss_time/helics_broker_backend.hpp"
#include "fss_time/helics_thread_participant_backend.hpp"
#include "fss_time/sim_time_broker.hpp"
#include "fss_time/thread_time_participant.hpp"

#include <atomic>
#include <chrono>
#include <future>
#include <functional>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr int64_t kMs = 1000000;
std::atomic<int> g_next_test_port{24500};

int reserve_test_port()
{
  return g_next_test_port.fetch_add(1);
}

fss_time::HelicsThreadParticipantOptions make_participant_options(
  const std::string & id,
  int port,
  bool count_for_metrics = true)
{
  fss_time::HelicsThreadParticipantOptions options;
  options.participant_id = id;
  options.broker_port = port;
  options.time_delta_ns = kMs;
  options.count_for_participant_metrics = count_for_metrics;
  return options;
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
  rclcpp::Node node("thread_time_same_thread_test");
  fss_time::thread_time_participant::reset_current_thread_for_testing();
  auto & first = fss_time::thread_time_participant::for_current_thread(node, "same_thread");
  auto & second = fss_time::thread_time_participant::for_current_thread(node, "same_thread");
  EXPECT_EQ(&first, &second);
  fss_time::thread_time_participant::reset_current_thread_for_testing();
}

TEST(ThreadTimeParticipant, DifferentThreadsReturnDifferentParticipants)
{
  rclcpp::Node node("thread_time_different_thread_test");
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

TEST(HelicsThreadParticipantBackend, ReportsInitialState)
{
  const int port = reserve_test_port();
  auto participant = std::make_shared<fss_time::HelicsThreadParticipantBackend>(
    make_participant_options("first_" + std::to_string(port), port, false));
  EXPECT_EQ(participant->current_time_ns(), 0);
  EXPECT_EQ(participant->last_requested_time_ns(), 0);
  EXPECT_FALSE(participant->is_request_in_flight());
  EXPECT_FALSE(participant->count_for_participant_metrics());
}

TEST(HelicsThreadParticipantBackend, RequestsAreRoundedUpToTimeDelta)
{
  const int port = reserve_test_port();
  auto broker = std::make_unique<fss_time::HelicsBrokerBackend>(
    fss_time::HelicsBrokerOptions{"zmq", "127.0.0.1", port, true, 0});
  broker->start();

  auto participant = std::make_shared<fss_time::HelicsThreadParticipantBackend>(
    make_participant_options("rounded_" + std::to_string(port), port, false));
  participant->announce_next_safe_time(1500000);

  wait_until([&participant]() {
    return participant->last_requested_time_ns() > 0;
  }, std::chrono::seconds(1));

  EXPECT_EQ(participant->last_requested_time_ns(), 2000000);

  participant->finalize();
  broker->finalize();
}

TEST(SimTimeBroker, PauseAndResumeAffectPublishedStatus)
{
  const int port = reserve_test_port();
  auto node = std::make_shared<rclcpp::Node>("sim_time_broker_status_test");
  node->declare_parameter("broker_port", port);
  node->declare_parameter("start_broker", true);
  node->declare_parameter("helics_time_delta_ns", 1000000);
  node->declare_parameter("speed_regulator_tick_ns", 1000000);
  node->declare_parameter("max_real_time_factor", 1.0);
  node->declare_parameter("running", false);
  fss_time::SimTimeBroker broker(*node);
  broker.start();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::atomic<bool> stop_executor{false};
  std::thread spin_thread([&executor, &stop_executor]() {
    while (!stop_executor.load()) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  auto paused_status = broker.status_message();
  EXPECT_FALSE(paused_status.running);

  broker.set_running(true);
  broker.set_max_real_time_factor(2.0);
  wait_until([&broker]() {
    return broker.status_message().sim_time.sec > 0 || broker.status_message().sim_time.nanosec > 0;
  }, std::chrono::seconds(2));

  const auto running_status = broker.status_message();
  EXPECT_TRUE(running_status.running);
  EXPECT_DOUBLE_EQ(running_status.max_real_time_factor, 2.0);

  stop_executor = true;
  spin_thread.join();
  executor.remove_node(node);
}
