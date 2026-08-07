#include "fss_px4_sim/mavros_sim/plugin.hpp"

#include <array>
#include <cstring>
#include <limits>

#include "mavros_msgs/msg/estimator_status.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/status_text.hpp"
#include "mavros_msgs/msg/sys_status.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace fss_px4_sim::mavros_sim
{
namespace
{
constexpr uint8_t kSystemId=1,kComponentId=1;
std::string px4_mode(uint32_t mode) {const auto main=static_cast<uint8_t>((mode>>16)&0xff),sub=static_cast<uint8_t>((mode>>24)&0xff);if(main==1)return "MANUAL";if(main==2)return "ALTCTL";if(main==3)return "POSCTL";if(main==5)return "ACRO";if(main==6)return "OFFBOARD";if(main==7)return "STABILIZED";if(main!=4)return "";if(sub==2)return "AUTO.TAKEOFF";if(sub==3)return "AUTO.LOITER";if(sub==4)return "AUTO.MISSION";if(sub==5)return "AUTO.RTL";if(sub==6)return "AUTO.LAND";return "AUTO";}
}
class SystemStatusPlugin final : public Plugin
{
public:
  using Plugin::Plugin;
  SystemStatusPlugin(rclcpp::Node & node,MavlinkIngress ingress,std::shared_ptr<SharedState> state):Plugin(node,std::move(ingress),std::move(state))
  {
    const auto sensor=rclcpp::SensorDataQoS(); const auto latched=rclcpp::QoS(1).reliable().transient_local();
    state_pub_=node_.create_publisher<mavros_msgs::msg::State>("mavros/state",latched); extended_pub_=node_.create_publisher<mavros_msgs::msg::ExtendedState>("mavros/extended_state",latched);
    sys_pub_=node_.create_publisher<mavros_msgs::msg::SysStatus>("mavros/sys_status",latched); battery_pub_=node_.create_publisher<sensor_msgs::msg::BatteryState>("mavros/battery",sensor);
    text_recv_pub_=node_.create_publisher<mavros_msgs::msg::StatusText>("mavros/statustext/recv",sensor); estimator_pub_=node_.create_publisher<mavros_msgs::msg::EstimatorStatus>("mavros/estimator_status",latched);
    text_send_sub_=node_.create_subscription<mavros_msgs::msg::StatusText>("mavros/statustext/send",sensor,[](mavros_msgs::msg::StatusText::SharedPtr) {});
    set_mode_srv_=node_.create_service<mavros_msgs::srv::SetMode>("mavros/set_mode",[this](const std::shared_ptr<mavros_msgs::srv::SetMode::Request> req,const std::shared_ptr<mavros_msgs::srv::SetMode::Response> res){res->mode_sent=queue_mode(*req);});
  }
  bool handles(uint32_t id) const override {return id==MAVLINK_MSG_ID_HEARTBEAT || id==MAVLINK_MSG_ID_EXTENDED_SYS_STATE || id==MAVLINK_MSG_ID_SYS_STATUS || id==MAVLINK_MSG_ID_STATUSTEXT || id==MAVLINK_MSG_ID_ESTIMATOR_STATUS;}
  void handle_message(const mavlink_message_t & message,const rclcpp::Time & stamp) override
  {
    if(message.msgid==MAVLINK_MSG_ID_HEARTBEAT){mavlink_heartbeat_t d{};mavlink_msg_heartbeat_decode(&message,&d);mavros_msgs::msg::State out;out.header.stamp=stamp;out.connected=true;out.armed=(d.base_mode&MAV_MODE_FLAG_SAFETY_ARMED)!=0;out.guided=(d.base_mode&MAV_MODE_FLAG_GUIDED_ENABLED)!=0;out.manual_input=(d.base_mode&MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)!=0;out.mode=px4_mode(d.custom_mode);out.system_status=d.system_status;state_pub_->publish(out);}
    else if(message.msgid==MAVLINK_MSG_ID_EXTENDED_SYS_STATE){mavlink_extended_sys_state_t d{};mavlink_msg_extended_sys_state_decode(&message,&d);mavros_msgs::msg::ExtendedState out;out.header.stamp=stamp;out.vtol_state=d.vtol_state;out.landed_state=d.landed_state;extended_pub_->publish(out);}
    else if(message.msgid==MAVLINK_MSG_ID_SYS_STATUS){mavlink_sys_status_t d{};mavlink_msg_sys_status_decode(&message,&d);mavros_msgs::msg::SysStatus sys;sys.header.stamp=stamp;sys.sensors_present=d.onboard_control_sensors_present;sys.sensors_enabled=d.onboard_control_sensors_enabled;sys.sensors_health=d.onboard_control_sensors_health;sys.load=d.load;sys.voltage_battery=d.voltage_battery;sys.current_battery=d.current_battery;sys.battery_remaining=d.battery_remaining;sys.drop_rate_comm=d.drop_rate_comm;sys.errors_comm=d.errors_comm;sys.errors_count1=d.errors_count1;sys.errors_count2=d.errors_count2;sys.errors_count3=d.errors_count3;sys.errors_count4=d.errors_count4;sys_pub_->publish(sys);sensor_msgs::msg::BatteryState b;b.header.stamp=stamp;b.voltage=d.voltage_battery==UINT16_MAX?std::numeric_limits<float>::quiet_NaN():d.voltage_battery/1000.0F;b.current=d.current_battery<0?std::numeric_limits<float>::quiet_NaN():-d.current_battery/100.0F;b.percentage=d.battery_remaining<0?std::numeric_limits<float>::quiet_NaN():d.battery_remaining/100.0F;b.present=d.voltage_battery!=UINT16_MAX;battery_pub_->publish(b);}
    else if(message.msgid==MAVLINK_MSG_ID_STATUSTEXT){mavlink_statustext_t d{};mavlink_msg_statustext_decode(&message,&d);mavros_msgs::msg::StatusText out;out.header.stamp=stamp;out.severity=d.severity;out.text=std::string(d.text,strnlen(d.text,sizeof(d.text)));text_recv_pub_->publish(out);}
    else {mavlink_estimator_status_t d{};mavlink_msg_estimator_status_decode(&message,&d);mavros_msgs::msg::EstimatorStatus out;out.header.stamp=stamp;out.attitude_status_flag=d.flags&(1<<0);out.velocity_horiz_status_flag=d.flags&(1<<1);out.velocity_vert_status_flag=d.flags&(1<<2);out.pos_horiz_rel_status_flag=d.flags&(1<<3);out.pos_horiz_abs_status_flag=d.flags&(1<<4);out.pos_vert_abs_status_flag=d.flags&(1<<5);out.pos_vert_agl_status_flag=d.flags&(1<<6);out.const_pos_mode_status_flag=d.flags&(1<<7);out.pred_pos_horiz_rel_status_flag=d.flags&(1<<8);out.pred_pos_horiz_abs_status_flag=d.flags&(1<<9);out.gps_glitch_status_flag=d.flags&(1<<10);out.accel_error_status_flag=d.flags&(1<<11);estimator_pub_->publish(out);}
  }
private:
  bool queue_mode(const mavros_msgs::srv::SetMode::Request & req)
  {uint32_t mode=0;if(!req.custom_mode.empty()){const std::array<std::pair<const char *,uint32_t>,10> modes{{{"MANUAL",1U<<16},{"ALTCTL",2U<<16},{"POSCTL",3U<<16},{"AUTO.MISSION",(4U<<16)|(4U<<24)},{"AUTO.LOITER",(4U<<16)|(3U<<24)},{"AUTO.RTL",(4U<<16)|(5U<<24)},{"AUTO.LAND",(4U<<16)|(6U<<24)},{"AUTO.TAKEOFF",(4U<<16)|(2U<<24)},{"ACRO",5U<<16},{"OFFBOARD",6U<<16}}};bool found=false;for(const auto & item:modes)if(req.custom_mode==item.first){mode=item.second;found=true;break;}if(!found)return false;}mavlink_message_t message{};mavlink_msg_set_mode_pack(kSystemId,kComponentId,&message,kSystemId,static_cast<uint8_t>(req.base_mode|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),mode);ingress_(px4::mavlink_receive_handle::SET_MODE,message);return true;}
  rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub_;rclcpp::Publisher<mavros_msgs::msg::ExtendedState>::SharedPtr extended_pub_;rclcpp::Publisher<mavros_msgs::msg::SysStatus>::SharedPtr sys_pub_;rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr text_recv_pub_;rclcpp::Publisher<mavros_msgs::msg::EstimatorStatus>::SharedPtr estimator_pub_;rclcpp::Subscription<mavros_msgs::msg::StatusText>::SharedPtr text_send_sub_;rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr set_mode_srv_;
};
std::unique_ptr<Plugin> make_system_status_plugin(rclcpp::Node & node,MavlinkIngress ingress,std::shared_ptr<SharedState> state){return std::make_unique<SystemStatusPlugin>(node,std::move(ingress),std::move(state));}
}  // namespace fss_px4_sim::mavros_sim
