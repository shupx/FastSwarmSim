#include "fss_time/helics_time_coordinator.hpp"

#include <chrono>
#include <future>
#include <stdexcept>
#include <sstream>
#include <string>

#include <gtest/gtest.h>
#include <helics/helics.h>

namespace
{

constexpr int64_t kMs = 1000000;

struct BrokerFixture
{
  BrokerFixture(int federates, int port)
  : port_(port)
  {
    HelicsError error = helicsErrorInitialize();
    std::ostringstream init;
    init << "--federates=" << federates << " --port=" << port_;
    broker_ = helicsCreateBroker("zmq", broker_name().c_str(), init.str().c_str(), &error);
    if (error.error_code != 0 || broker_ == nullptr) {
      throw std::runtime_error(error.message == nullptr ? "failed to create HELICS broker" : error.message);
    }
  }

  ~BrokerFixture()
  {
    if (broker_ != nullptr) {
      HelicsError error = helicsErrorInitialize();
      helicsBrokerDisconnect(broker_, &error);
      helicsBrokerFree(broker_);
      broker_ = nullptr;
    }
  }

  std::string broker_name() const
  {
    std::ostringstream name;
    name << "fss_time_test_broker_" << port_;
    return name.str();
  }

  int port_;
  HelicsBroker broker_{nullptr};
};

fss_time::HelicsTimeOptions make_options(const std::string & id, int port, double max_speed_ratio)
{
  fss_time::HelicsTimeOptions options;
  options.participant_id = id;
  options.broker_port = port;
  options.time_delta_ns = kMs;
  options.max_speed_ratio = max_speed_ratio;
  return options;
}

template<typename T>
T wait_future(std::future<T> & future)
{
  const auto status = future.wait_for(std::chrono::seconds(10));
  EXPECT_EQ(status, std::future_status::ready);
  return future.get();
}

}  // namespace

TEST(HelicsTimeCoordinator, FederatesGrantMonotonicTimes)
{
  BrokerFixture broker(2, 23441);
  fss_time::HelicsTimeCoordinator first(make_options("first", broker.port_, 0.0));
  fss_time::HelicsTimeCoordinator second(make_options("second", broker.port_, 0.0));

  auto first_start = std::async(std::launch::async, [&first]() { first.start(); });
  auto second_start = std::async(std::launch::async, [&second]() { second.start(); });
  wait_future(first_start);
  wait_future(second_start);

  first.set_next_safe_time(10 * kMs);
  second.set_next_safe_time(20 * kMs);

  auto first_grant_future = std::async(
    std::launch::async, [&first]() { return first.request_grant(0); });
  auto second_grant_future = std::async(
    std::launch::async, [&second]() { return second.request_grant(0); });

  const auto first_grant = wait_future(first_grant_future);
  const auto second_grant = wait_future(second_grant_future);

  EXPECT_TRUE(first_grant.advanced);
  EXPECT_TRUE(second_grant.advanced);
  EXPECT_GE(first_grant.grant_time_ns, 0);
  EXPECT_GE(second_grant.grant_time_ns, 0);
  EXPECT_LE(first_grant.grant_time_ns, 10 * kMs);
  EXPECT_LE(second_grant.grant_time_ns, 20 * kMs);
  EXPECT_GE(first.current_time_ns(), first_grant.grant_time_ns);
  EXPECT_GE(second.current_time_ns(), second_grant.grant_time_ns);
}

TEST(HelicsTimeCoordinator, PauseResumeAndSpeedCap)
{
  BrokerFixture broker(1, 23442);
  fss_time::HelicsTimeCoordinator coordinator(make_options("speed_limited", broker.port_, 1.0));
  coordinator.start();
  coordinator.set_next_safe_time(50 * kMs);

  coordinator.set_paused(true);
  const auto paused = coordinator.request_grant(100 * kMs);
  EXPECT_FALSE(paused.advanced);
  EXPECT_TRUE(paused.paused);
  EXPECT_EQ(paused.grant_time_ns, 0);

  coordinator.set_paused(false);
  const auto not_ready = coordinator.request_grant(0);
  EXPECT_FALSE(not_ready.advanced);
  EXPECT_EQ(not_ready.grant_time_ns, 0);

  const auto speed_limited = coordinator.request_grant(20 * kMs);
  EXPECT_TRUE(speed_limited.advanced);
  EXPECT_EQ(speed_limited.grant_time_ns, 20 * kMs);

  coordinator.set_speed(0.0, 20 * kMs);
  const auto unlimited = coordinator.request_grant(50 * kMs);
  EXPECT_TRUE(unlimited.advanced);
  EXPECT_EQ(unlimited.grant_time_ns, 50 * kMs);
}
