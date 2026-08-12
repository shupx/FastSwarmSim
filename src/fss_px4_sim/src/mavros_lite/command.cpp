#include "fss_px4_sim/mavros_lite/core.hpp"

#include <chrono>
#include <condition_variable>
#include <unordered_map>

#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_int.hpp>
#include <mavros_msgs/srv/command_long.hpp>

namespace fss_px4_sim::mavros_lite
{
using namespace std::chrono_literals;

class Command final : public Module
{
public:
  explicit Command(MavrosLite & core)
  : Module(core)
  {
    timeout_ = std::chrono::duration<double>(core.declare_parameter<double>("command.ack_timeout", 5.0));
    long_srv_ = core.create_service<mavros_msgs::srv::CommandLong>("cmd/command",
      [this](const std::shared_ptr<rmw_request_id_t>,
        mavros_msgs::srv::CommandLong::Request::SharedPtr request,
        mavros_msgs::srv::CommandLong::Response::SharedPtr response) {send_long(*request, *response);});
    int_srv_ = core.create_service<mavros_msgs::srv::CommandInt>("cmd/command_int",
      [this](const std::shared_ptr<rmw_request_id_t>,
        mavros_msgs::srv::CommandInt::Request::SharedPtr request,
        mavros_msgs::srv::CommandInt::Response::SharedPtr response) {send_int(*request, *response);});
    arm_srv_ = core.create_service<mavros_msgs::srv::CommandBool>("cmd/arming",
      [this](const std::shared_ptr<rmw_request_id_t>,
        mavros_msgs::srv::CommandBool::Request::SharedPtr request,
        mavros_msgs::srv::CommandBool::Response::SharedPtr response) {
        mavros_msgs::srv::CommandLong::Request command;
        command.command = MAV_CMD_COMPONENT_ARM_DISARM; command.confirmation = 1;
        command.param1 = request->value ? 1.0F : 0.0F;
        mavros_msgs::srv::CommandLong::Response result; send_long(command, result);
        response->success = result.success; response->result = result.result;
      });
  }

  void handle_message(const mavlink_message_t & message) override
  {
    if (message.msgid != MAVLINK_MSG_ID_COMMAND_ACK) return;
    mavlink_command_ack_t ack{}; mavlink_msg_command_ack_decode(&message, &ack);
    {std::lock_guard<std::mutex> lock(mutex_); acknowledgements_[ack.command] = ack.result;}
    condition_.notify_all();
  }

private:
  void send_long(const mavros_msgs::srv::CommandLong::Request & request,
    mavros_msgs::srv::CommandLong::Response & response)
  {
    const uint8_t target_system = request.broadcast ? 0 : core_.target_system();
    const uint8_t target_component = request.broadcast ? 0 : core_.target_component();
    {std::lock_guard<std::mutex> lock(mutex_); acknowledgements_.erase(request.command);}
    mavlink_message_t message{};
    mavlink_msg_command_long_pack(core_.system_id(), core_.component_id(), &message,
      target_system, target_component, request.command, request.broadcast ? 0 : request.confirmation,
      request.param1, request.param2, request.param3, request.param4, request.param5, request.param6,
      request.param7);
    core_.send_message(message);
    if (request.broadcast) {response.success = true; response.result = MAV_RESULT_ACCEPTED; return;}
    std::unique_lock<std::mutex> lock(mutex_);
    const bool received = condition_.wait_for(lock, timeout_, [this, &request] {
      return acknowledgements_.count(request.command) != 0;});
    response.result = received ? acknowledgements_[request.command] :
      static_cast<uint8_t>(MAV_RESULT_FAILED);
    response.success = received && response.result == MAV_RESULT_ACCEPTED;
    acknowledgements_.erase(request.command);
  }

  void send_int(const mavros_msgs::srv::CommandInt::Request & request,
    mavros_msgs::srv::CommandInt::Response & response)
  {
    mavlink_message_t message{};
    mavlink_msg_command_int_pack(core_.system_id(), core_.component_id(), &message,
      request.broadcast ? 0 : core_.target_system(), request.broadcast ? 0 : core_.target_component(),
      request.frame, request.command, request.current, request.autocontinue, request.param1,
      request.param2, request.param3, request.param4, request.x, request.y, request.z);
    core_.send_message(message); response.success = true;
  }

  std::chrono::duration<double> timeout_{};
  std::mutex mutex_; std::condition_variable condition_;
  std::unordered_map<uint16_t, uint8_t> acknowledgements_;
  rclcpp::Service<mavros_msgs::srv::CommandLong>::SharedPtr long_srv_;
  rclcpp::Service<mavros_msgs::srv::CommandInt>::SharedPtr int_srv_;
  rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr arm_srv_;
};
std::unique_ptr<Module> make_command(MavrosLite & core) {return std::make_unique<Command>(core);}
}  // namespace fss_px4_sim::mavros_lite
