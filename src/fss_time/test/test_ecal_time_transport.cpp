#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <sys/wait.h>
#include <unistd.h>

#include "fss_time/time_transport.hpp"
#include "fss_time/time_types.hpp"

namespace
{

fss_time::TimeTransportOptions make_options(const std::string & participant_id, const std::string & suffix)
{
  fss_time::TimeTransportOptions options;
  options.participant_id = participant_id;
  options.intent_topic = "/fss/test/" + suffix + "/time_intent";
  options.control_topic = "/fss/test/" + suffix + "/time_control";
  return options;
}

fss_time_interfaces::msg::TimeIntent make_intent(uint64_t epoch)
{
  fss_time_interfaces::msg::TimeIntent intent;
  intent.participant_id = "tx";
  intent.epoch = epoch;
  intent.current_time = fss_time::from_ns(1000);
  intent.next_safe_time = fss_time::from_ns(2000);
  intent.lookahead = fss_time::duration_from_ns(1000);
  intent.state = fss_time_interfaces::msg::TimeIntent::STATE_ACTIVE;
  intent.lease_deadline_steady_ns = 3000;
  return intent;
}

fss_time_interfaces::msg::TimeControl make_control(uint64_t epoch)
{
  fss_time_interfaces::msg::TimeControl control;
  control.epoch = epoch;
  control.command = fss_time_interfaces::msg::TimeControl::COMMAND_SET_SPEED;
  control.max_speed_ratio = 4.0;
  return control;
}

void publish_until_deadline(
  fss_time::TimeTransport & transport,
  const fss_time_interfaces::msg::TimeIntent & intent,
  const fss_time_interfaces::msg::TimeControl & control,
  std::chrono::steady_clock::time_point deadline)
{
  while (std::chrono::steady_clock::now() < deadline) {
    transport.publish_intent(intent);
    transport.publish_control(control);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

TEST(EcalTimeTransport, InterprocessLocalTransportDeliversIntentAndControl)
{
  const auto topic_suffix = std::to_string(::getpid()) + "_interprocess";

  const auto pid = ::fork();
  ASSERT_GE(pid, 0);

  if (pid == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    auto tx = fss_time::create_time_transport("ecal", make_options("tx", topic_suffix));
    tx->start(nullptr, nullptr);
    publish_until_deadline(
      *tx,
      make_intent(21),
      make_control(22),
      std::chrono::steady_clock::now() + std::chrono::seconds(2));
    ::_exit(0);
  }

  auto rx = fss_time::create_time_transport("ecal", make_options("rx", topic_suffix));

  std::atomic<bool> got_intent{false};
  std::atomic<bool> got_control{false};

  rx->start(
    [&got_intent](const fss_time_interfaces::msg::TimeIntent & intent) {
      if (intent.participant_id == "tx" && intent.epoch == 21) {
        got_intent = true;
      }
    },
    [&got_control](const fss_time_interfaces::msg::TimeControl & control) {
      if (
        control.epoch == 22 &&
        control.command == fss_time_interfaces::msg::TimeControl::COMMAND_SET_SPEED &&
        control.max_speed_ratio == 4.0)
      {
        got_control = true;
      }
    });

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (std::chrono::steady_clock::now() < deadline && (!got_intent || !got_control)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  int child_status = 0;
  ASSERT_EQ(::waitpid(pid, &child_status, 0), pid);
  EXPECT_TRUE(WIFEXITED(child_status));
  EXPECT_EQ(WEXITSTATUS(child_status), 0);
  EXPECT_TRUE(got_intent);
  EXPECT_TRUE(got_control);
}

TEST(EcalTimeTransport, SameProcessLoopbackDeliversIntentAndControl)
{
  const auto topic_suffix = std::to_string(::getpid());

  const auto tx_options = make_options("tx", topic_suffix);
  const auto rx_options = make_options("rx", topic_suffix);

  auto tx = fss_time::create_time_transport("ecal", tx_options);
  auto rx = fss_time::create_time_transport("ecal", rx_options);

  std::atomic<bool> got_intent{false};
  std::atomic<bool> got_control{false};

  tx->start(nullptr, nullptr);
  rx->start(
    [&got_intent](const fss_time_interfaces::msg::TimeIntent & intent) {
      if (intent.participant_id == "tx" && intent.epoch == 9) {
        got_intent = true;
      }
    },
    [&got_control](const fss_time_interfaces::msg::TimeControl & control) {
      if (
        control.epoch == 12 &&
        control.command == fss_time_interfaces::msg::TimeControl::COMMAND_SET_SPEED &&
        control.max_speed_ratio == 4.0)
      {
        got_control = true;
      }
    });

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline && (!got_intent || !got_control)) {
    tx->publish_intent(make_intent(9));
    tx->publish_control(make_control(12));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_TRUE(got_intent);
  EXPECT_TRUE(got_control);
}

}  // namespace
