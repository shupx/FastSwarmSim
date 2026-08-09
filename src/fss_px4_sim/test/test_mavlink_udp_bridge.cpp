/**
 * @file test_mavlink_udp_bridge.cpp
 * @brief Tests the simulated MAVLink UDP bridge and instance-local PX4 state.
 * Added by Peixuan Shu (2026-08-09).
 */

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <poll.h>
#include <thread>

#include <gtest/gtest.h>

#include "parameters/px4_parameters.hpp"
#include "fss_px4_sim/px4_instance_context.hpp"
#include "px4_modules/mavlink/mavlink_main.hpp"
#include "uORB/uORB_sim.hpp"
#include <uORB/topics/vehicle_command_ack.h>

namespace
{
using namespace std::chrono_literals;

// Added by Peixuan Shu: cover isolation and generation updates across domains.
TEST(UorbDomain, IsolatesInstancesAndTracksGeneration)
{
  uORB_sim::Domain first;
  uORB_sim::Domain second;
  uORB_sim::Publication<vehicle_attitude_s> first_pub{first, ORB_ID(vehicle_attitude)};
  uORB_sim::Publication<vehicle_attitude_s> second_pub{second, ORB_ID(vehicle_attitude)};
  uORB_sim::Subscription<vehicle_attitude_s> first_sub{first, ORB_ID(vehicle_attitude)};
  uORB_sim::Subscription<vehicle_attitude_s> second_sub{second, ORB_ID(vehicle_attitude)};

  vehicle_attitude_s sample{};
  sample.timestamp = 123;
  sample.q[0] = 1.0F;
  first_pub.publish(sample);

  vehicle_attitude_s received{};
  ASSERT_TRUE(first_sub.updated());
  ASSERT_TRUE(first_sub.update(&received));
  EXPECT_EQ(received.timestamp, 123U);
  EXPECT_FLOAT_EQ(received.q[0], 1.0F);
  EXPECT_FALSE(second_sub.updated());

  // Generation, rather than a byte comparison, detects an identical sample.
  first_pub.publish(sample);
  EXPECT_TRUE(first_sub.updated());
  ASSERT_TRUE(first_sub.update(&received));

  vehicle_attitude_s second_sample{};
  second_sample.timestamp = 456;
  second_pub.publish(second_sample);
  EXPECT_TRUE(second_sub.updated());
  EXPECT_FALSE(first_sub.updated());
}

// Added by Peixuan Shu: verify parameter writes remain local to each PX4 instance.
TEST(ParameterStore, IsolatesInstances)
{
  px4::ParameterStore first;
  px4::ParameterStore second;
  constexpr param_t parameter = static_cast<param_t>(1);
  ASSERT_EQ(px4::parameters_type[parameter], PARAM_TYPE_FLOAT);

  float first_value = 0.0F;
  float second_value = 0.0F;
  ASSERT_EQ(first.get(parameter, &first_value), PX4_OK);
  ASSERT_EQ(second.get(parameter, &second_value), PX4_OK);
  EXPECT_FLOAT_EQ(first_value, second_value);

  const float override_value = 17.25F;
  ASSERT_EQ(first.set(parameter, &override_value), PX4_OK);
  ASSERT_EQ(first.get(parameter, &first_value), PX4_OK);
  ASSERT_EQ(second.get(parameter, &second_value), PX4_OK);
  EXPECT_FLOAT_EQ(first_value, override_value);
  EXPECT_FLOAT_EQ(second_value, px4::parameters[parameter].val.f);
}

// Added by Peixuan Shu: verify nested clock scopes restore the owner correctly.
TEST(SimClock, ScopeSelectsTheOwningInstance)
{
  px4::SimClock first;
  px4::SimClock second;
  first.set(100);
  second.set(200);

  {
    px4::SimClock::Scope first_scope(first);
    EXPECT_EQ(hrt_absolute_time(), 100U);
    {
      px4::SimClock::Scope second_scope(second);
      EXPECT_EQ(hrt_absolute_time(), 200U);
    }
    EXPECT_EQ(hrt_absolute_time(), 100U);
  }
}

// Added by Peixuan Shu: verify the aggregate context selects all PX4 state.
TEST(Px4InstanceContext, ScopeSelectsAllInstanceState)
{
  MavrosQuadSimulator::Px4InstanceContext context;
  context.clock.set(300);
  uORB_sim::Publication<vehicle_attitude_s> publication{context.uorb_domain, ORB_ID(vehicle_attitude)};

  {
    MavrosQuadSimulator::Px4InstanceContext::Scope scope(context);
    EXPECT_EQ(&uORB_sim::Domain::current(), &context.uorb_domain);
    EXPECT_EQ(&px4::ParameterStore::current(), &context.parameter_store);
    EXPECT_EQ(hrt_absolute_time(), 300U);
  }

  vehicle_attitude_s attitude{};
  attitude.timestamp = 300;
  publication.publish(attitude);
}

// Modified by Peixuan Shu: MAVLINK now receives one aggregate instance context.
TEST(MavlinkUdpReceiver, PollThreadHandlesSetMode)
{
  MavrosQuadSimulator::Px4InstanceContext context;
  MavrosQuadSimulator::Px4InstanceContext::Scope scope(context);
  const int receiver_port = 23000 + (getpid() % 1000) * 4;
  const int sender_port = receiver_port + 1;
  MAVLINK mavlink(context, receiver_port, sender_port);
  mavlink.start();
  uORB_sim::Subscription<vehicle_command_s> command_sub{context.uorb_domain, ORB_ID(vehicle_command)};

  const int sender_fd = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(sender_fd, 0);
  sockaddr_in sender{};
  sender.sin_family = AF_INET;
  sender.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  sender.sin_port = htons(static_cast<uint16_t>(sender_port));
  ASSERT_EQ(bind(sender_fd, reinterpret_cast<const sockaddr *>(&sender), sizeof(sender)), 0);

  mavlink_message_t set_mode{};
  mavlink_msg_set_mode_pack(1, MAV_COMP_ID_ONBOARD_COMPUTER, &set_mode, 1,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0x00060000U);
  uint8_t packet[MAVLINK_MAX_PACKET_LEN];
  const auto packet_length = mavlink_msg_to_send_buffer(packet, &set_mode);

  sockaddr_in receiver_address{};
  receiver_address.sin_family = AF_INET;
  receiver_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  receiver_address.sin_port = htons(static_cast<uint16_t>(receiver_port));
  ASSERT_EQ(sendto(sender_fd, packet, packet_length, 0,
    reinterpret_cast<const sockaddr *>(&receiver_address), sizeof(receiver_address)),
    static_cast<ssize_t>(packet_length));

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  vehicle_command_s command{};
  while ((!command_sub.updated() || !command_sub.update(&command))
         && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_SET_MODE);
  EXPECT_EQ(command.target_system, 1);

  close(sender_fd);
}

TEST(MavlinkUdpSender, SendsEncodedPacketToRemote)
{
  MavrosQuadSimulator::Px4InstanceContext context;
  MavrosQuadSimulator::Px4InstanceContext::Scope scope(context);
  const int receiver_port = 23000 + (getpid() % 1000) * 4 + 2;
  const int remote_port = receiver_port + 1;

  const int remote_fd = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(remote_fd, 0);
  sockaddr_in remote{};
  remote.sin_family = AF_INET;
  remote.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  remote.sin_port = htons(static_cast<uint16_t>(remote_port));
  ASSERT_EQ(bind(remote_fd, reinterpret_cast<const sockaddr *>(&remote), sizeof(remote)), 0);

  MAVLINK mavlink(context, receiver_port, remote_port);
  mavlink.start();
  mavlink.Stream(100000);

  pollfd poll_fd{remote_fd, POLLIN, 0};
  ASSERT_EQ(poll(&poll_fd, 1, 2000), 1);
  uint8_t packet[MAVLINK_MAX_PACKET_LEN];
  const auto length = recv(remote_fd, packet, sizeof(packet), 0);
  ASSERT_GT(length, 0);

  mavlink_message_t received{};
  mavlink_status_t status{};
  bool parsed = false;
  for (ssize_t index = 0; index < length; ++index) {
    if (mavlink_parse_char(MAVLINK_COMM_1, packet[index], &received, &status)) {
      parsed = true;
      break;
    }
  }
  ASSERT_TRUE(parsed);
  EXPECT_EQ(received.msgid, MAVLINK_MSG_ID_HEARTBEAT);

  close(remote_fd);
}

TEST(MavlinkUdpSender, SendsCommandAcknowledgement)
{
  MavrosQuadSimulator::Px4InstanceContext context;
  MavrosQuadSimulator::Px4InstanceContext::Scope scope(context);
  const int local_port = 23000 + (getpid() % 1000) * 4 + 4;
  const int remote_port = local_port + 1;

  const int remote_fd = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(remote_fd, 0);
  sockaddr_in remote{};
  remote.sin_family = AF_INET;
  remote.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  remote.sin_port = htons(static_cast<uint16_t>(remote_port));
  ASSERT_EQ(bind(remote_fd, reinterpret_cast<const sockaddr *>(&remote), sizeof(remote)), 0);

  MAVLINK mavlink(context, local_port, remote_port);
  mavlink.start();

  uORB_sim::Publication<vehicle_command_ack_s> ack_pub{context.uorb_domain, ORB_ID(vehicle_command_ack)};
  vehicle_command_ack_s ack{};
  ack.timestamp = 1;
  ack.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
  ack.result = vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;
  ack.target_system = 42;
  ack.target_component = MAV_COMP_ID_ONBOARD_COMPUTER;
  ack_pub.publish(ack);

  mavlink.Stream(0);

  pollfd poll_fd{remote_fd, POLLIN, 0};
  ASSERT_EQ(poll(&poll_fd, 1, 2000), 1);
  uint8_t packet[MAVLINK_MAX_PACKET_LEN];
  const auto length = recv(remote_fd, packet, sizeof(packet), 0);
  ASSERT_GT(length, 0);

  mavlink_message_t received{};
  mavlink_status_t status{};
  bool parsed = false;
  for (ssize_t index = 0; index < length; ++index) {
    if (mavlink_parse_char(MAVLINK_COMM_2, packet[index], &received, &status)) {
      parsed = true;
      break;
    }
  }
  ASSERT_TRUE(parsed);
  EXPECT_EQ(received.msgid, MAVLINK_MSG_ID_COMMAND_ACK);

  mavlink_command_ack_t decoded{};
  mavlink_msg_command_ack_decode(&received, &decoded);
  EXPECT_EQ(decoded.command, vehicle_command_s::VEHICLE_CMD_DO_SET_MODE);
  EXPECT_EQ(decoded.result, MAV_RESULT_ACCEPTED);
  EXPECT_EQ(decoded.target_system, 42);
  EXPECT_EQ(decoded.target_component, MAV_COMP_ID_ONBOARD_COMPUTER);

  close(remote_fd);
}
}  // namespace
