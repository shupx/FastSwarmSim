#include "fss_px4_sim/mavros_sim/plugin.hpp"

#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_home.hpp"
#include "mavros_msgs/srv/command_int.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/command_trigger_control.hpp"
#include "mavros_msgs/srv/command_trigger_interval.hpp"
#include "mavros_msgs/srv/command_vtol_transition.hpp"

namespace fss_px4_sim::mavros_sim
{
namespace { constexpr uint8_t kSystemId=1,kComponentId=1; }
class CommandPlugin final : public Plugin
{
public:
  using Plugin::Plugin;
  CommandPlugin(rclcpp::Node & node,MavlinkIngress ingress,std::shared_ptr<SharedState> state):Plugin(node,std::move(ingress),std::move(state))
  {
    command_srv_=node_.create_service<mavros_msgs::srv::CommandLong>("mavros/cmd/command",[this](std::shared_ptr<mavros_msgs::srv::CommandLong::Request> req,std::shared_ptr<mavros_msgs::srv::CommandLong::Response> res){long_command(req->broadcast,req->command,req->confirmation,req->param1,req->param2,req->param3,req->param4,req->param5,req->param6,req->param7,res->success,res->result);});
    command_int_srv_=node_.create_service<mavros_msgs::srv::CommandInt>("mavros/cmd/command_int",[this](std::shared_ptr<mavros_msgs::srv::CommandInt::Request> req,std::shared_ptr<mavros_msgs::srv::CommandInt::Response> res){mavlink_message_t m{};mavlink_msg_command_int_pack(kSystemId,kComponentId,&m,req->broadcast?0:kSystemId,req->broadcast?0:kComponentId,req->frame,req->command,req->current,req->autocontinue,req->param1,req->param2,req->param3,req->param4,req->x,req->y,req->z);ingress_(px4::mavlink_receive_handle::COMMAND_INT,m);res->success=true;});
    arming_srv_=node_.create_service<mavros_msgs::srv::CommandBool>("mavros/cmd/arming",[this](std::shared_ptr<mavros_msgs::srv::CommandBool::Request> req,std::shared_ptr<mavros_msgs::srv::CommandBool::Response> res){long_command(false,MAV_CMD_COMPONENT_ARM_DISARM,1,req->value?1.0F:0.0F,0,0,0,0,0,0,res->success,res->result);});
    home_srv_=node_.create_service<mavros_msgs::srv::CommandHome>("mavros/cmd/set_home",[this](std::shared_ptr<mavros_msgs::srv::CommandHome::Request> req,std::shared_ptr<mavros_msgs::srv::CommandHome::Response> res){long_command(false,MAV_CMD_DO_SET_HOME,1,req->current_gps?1.0F:0.0F,0,0,req->yaw,req->latitude,req->longitude,req->altitude,res->success,res->result);});
    takeoff_srv_=node_.create_service<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff",[this](std::shared_ptr<mavros_msgs::srv::CommandTOL::Request> req,std::shared_ptr<mavros_msgs::srv::CommandTOL::Response> res){long_command(false,MAV_CMD_NAV_TAKEOFF,1,req->min_pitch,0,0,req->yaw,req->latitude,req->longitude,req->altitude,res->success,res->result);});
    land_srv_=node_.create_service<mavros_msgs::srv::CommandTOL>("mavros/cmd/land",[this](std::shared_ptr<mavros_msgs::srv::CommandTOL::Request> req,std::shared_ptr<mavros_msgs::srv::CommandTOL::Response> res){long_command(false,MAV_CMD_NAV_LAND,1,0,0,0,req->yaw,req->latitude,req->longitude,req->altitude,res->success,res->result);});
    trigger_control_srv_=node_.create_service<mavros_msgs::srv::CommandTriggerControl>("mavros/cmd/trigger_control",[this](std::shared_ptr<mavros_msgs::srv::CommandTriggerControl::Request> req,std::shared_ptr<mavros_msgs::srv::CommandTriggerControl::Response> res){long_command(false,MAV_CMD_DO_TRIGGER_CONTROL,1,req->trigger_enable,req->sequence_reset,req->trigger_pause,0,0,0,0,res->success,res->result);});
    trigger_interval_srv_=node_.create_service<mavros_msgs::srv::CommandTriggerInterval>("mavros/cmd/trigger_interval",[this](std::shared_ptr<mavros_msgs::srv::CommandTriggerInterval::Request> req,std::shared_ptr<mavros_msgs::srv::CommandTriggerInterval::Response> res){long_command(false,MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL,1,req->cycle_time,req->integration_time,0,0,0,0,0,res->success,res->result);});
    transition_srv_=node_.create_service<mavros_msgs::srv::CommandVtolTransition>("mavros/cmd/vtol_transition",[this](std::shared_ptr<mavros_msgs::srv::CommandVtolTransition::Request> req,std::shared_ptr<mavros_msgs::srv::CommandVtolTransition::Response> res){long_command(false,MAV_CMD_DO_VTOL_TRANSITION,0,req->state,0,0,0,0,0,0,res->success,res->result);});
  }
  bool handles(uint32_t) const override {return false;} // ROS 1 simulator deliberately did not wait for COMMAND_ACK.
  void handle_message(const mavlink_message_t &,const rclcpp::Time &) override {}
private:
  void long_command(bool broadcast,uint16_t command,uint8_t confirmation,float p1,float p2,float p3,float p4,float p5,float p6,float p7,bool & success,uint8_t & result)
  {mavlink_message_t m{};mavlink_msg_command_long_pack(kSystemId,kComponentId,&m,broadcast?0:kSystemId,broadcast?0:kComponentId,command,broadcast?0:confirmation,p1,p2,p3,p4,p5,p6,p7);ingress_(px4::mavlink_receive_handle::COMMAND_LONG,m);success=true;result=MAV_RESULT_ACCEPTED;}
  rclcpp::Service<mavros_msgs::srv::CommandLong>::SharedPtr command_srv_;rclcpp::Service<mavros_msgs::srv::CommandInt>::SharedPtr command_int_srv_;rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr arming_srv_;rclcpp::Service<mavros_msgs::srv::CommandHome>::SharedPtr home_srv_;rclcpp::Service<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_srv_,land_srv_;rclcpp::Service<mavros_msgs::srv::CommandTriggerControl>::SharedPtr trigger_control_srv_;rclcpp::Service<mavros_msgs::srv::CommandTriggerInterval>::SharedPtr trigger_interval_srv_;rclcpp::Service<mavros_msgs::srv::CommandVtolTransition>::SharedPtr transition_srv_;
};
std::unique_ptr<Plugin> make_command_plugin(rclcpp::Node & node,MavlinkIngress ingress,std::shared_ptr<SharedState> state){return std::make_unique<CommandPlugin>(node,std::move(ingress),std::move(state));}
}  // namespace fss_px4_sim::mavros_sim
