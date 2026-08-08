#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <poll.h>
#include <thread>

#include <gtest/gtest.h>

#include "parameters/px4_parameters.hpp"
#include "px4_modules/mavlink/mavlink_main.hpp"
#include "uORB/uORB_sim.hpp"
#include <uORB/topics/vehicle_command_ack.h>

namespace
{
using namespace std::chrono_literals;

TEST(MavlinkUdpReceiver, PollThreadHandlesSetMode)
{
  px4::allocate_px4_params_storage(1);
  uORB_sim::allocate_uorb_message_storage(1);
  const int receiver_port = 23000 + (getpid() % 1000) * 4;
  const int sender_port = receiver_port + 1;
  MAVLINK mavlink(0, receiver_port, sender_port);
  mavlink.start();

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
  while (uORB_sim::vehicle_command.at(0).command != vehicle_command_s::VEHICLE_CMD_DO_SET_MODE
         && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(uORB_sim::vehicle_command.at(0).command, vehicle_command_s::VEHICLE_CMD_DO_SET_MODE);
  EXPECT_EQ(uORB_sim::vehicle_command.at(0).target_system, 1);

  close(sender_fd);
}

TEST(MavlinkUdpSender, SendsEncodedPacketToRemote)
{
  px4::allocate_px4_params_storage(1);
  uORB_sim::allocate_uorb_message_storage(1);
  const int receiver_port = 23000 + (getpid() % 1000) * 4 + 2;
  const int remote_port = receiver_port + 1;

  const int remote_fd = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(remote_fd, 0);
  sockaddr_in remote{};
  remote.sin_family = AF_INET;
  remote.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  remote.sin_port = htons(static_cast<uint16_t>(remote_port));
  ASSERT_EQ(bind(remote_fd, reinterpret_cast<const sockaddr *>(&remote), sizeof(remote)), 0);

  MAVLINK mavlink(0, receiver_port, remote_port);
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
  px4::allocate_px4_params_storage(1);
  uORB_sim::allocate_uorb_message_storage(1);
  const int local_port = 23000 + (getpid() % 1000) * 4 + 4;
  const int remote_port = local_port + 1;

  const int remote_fd = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(remote_fd, 0);
  sockaddr_in remote{};
  remote.sin_family = AF_INET;
  remote.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  remote.sin_port = htons(static_cast<uint16_t>(remote_port));
  ASSERT_EQ(bind(remote_fd, reinterpret_cast<const sockaddr *>(&remote), sizeof(remote)), 0);

  int agent_id_ = 0;
  MAVLINK mavlink(agent_id_, local_port, remote_port);
  mavlink.start();

  uORB_sim::Publication<vehicle_command_ack_s> ack_pub{agent_id_, uORB_sim::vehicle_command_ack};
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
