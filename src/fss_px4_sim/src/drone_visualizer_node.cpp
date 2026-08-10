#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_ack.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

#include "geo/geo.h" // from fss_px4_sim px4_core

class DroneVisualizer : public rclcpp::Node
{
public:
  DroneVisualizer()
  : Node("px4_rotor_visualizer_node")
  {
    max_frequency_ = declare_parameter<double>("visualize_max_freq", 20.0);
    history_seconds_ = declare_parameter<double>("visualize_path_time", 30.0);
    history_enabled_ = declare_parameter<bool>("enable_history_path", true);
    frame_id_ = declare_parameter<std::string>("visualize_tf_frame", "map");
    child_frame_id_ = declare_parameter<std::string>("base_link_name", "base_link");
    const auto tf_prefix = declare_parameter<std::string>("base_link_tf_prefix", "");
    if (!tf_prefix.empty()) child_frame_id_ = tf_prefix + "/" + child_frame_id_;
    marker_name_ = declare_parameter<std::string>("visualize_marker_name", "uav");
    local_pos_source_ = declare_parameter<int>("local_pos_source", 0);
    origin_latitude_deg_ = declare_parameter<double>("world_origin_latitude_deg", 39.978861);
    origin_longitude_deg_ = declare_parameter<double>("world_origin_longitude_deg", 116.339803);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joint_names_[i] = declare_parameter<std::string>("rotor_" + std::to_string(i) + "_joint_name", "rotor_" + std::to_string(i) + "_joint");
    }
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "mavros/local_position/pose", rclcpp::SensorDataQoS(), [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        std::scoped_lock lock(mutex_);
        pose_ = *message;
        pose_.header.frame_id = frame_id_;
        have_pose_ = true;
      });
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "mavros/state", rclcpp::QoS(10), [this](const mavros_msgs::msg::State::SharedPtr message) {
        std::scoped_lock lock(mutex_);
        armed_ = message->armed;
      });
    global_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "mavros/global_position/global", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr message) {
        if (local_pos_source_ != 1) return;
        std::scoped_lock lock(mutex_);
        MapProjection global_local_proj_ref{origin_latitude_deg_, origin_longitude_deg_, 0};
        float east = 0.0F;
        float north = 0.0F;
        global_local_proj_ref.project(message->latitude, message->longitude, north, east);
        gps_x_east_ = east;
        gps_y_north_ = north;
        have_gps_ = true;
      });
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("visualizer/joint_states", rclcpp::QoS(1).transient_local());
    path_pub_ = create_publisher<nav_msgs::msg::Path>("visualizer/history_path", rclcpp::QoS(1).transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("visualizer/marker_name", rclcpp::QoS(1).transient_local());
    switch_source_srv_ = create_service<mavros_msgs::srv::CommandAck>(
      "visualizer/switch_visualize_pose_source",
      [this](const std::shared_ptr<mavros_msgs::srv::CommandAck::Request> request,
        std::shared_ptr<mavros_msgs::srv::CommandAck::Response> response) {
        if (request->command != 0 && request->command != 1) {
          response->success = false;
          response->result = 1;
          return;
        }
        std::scoped_lock lock(mutex_);
        local_pos_source_ = request->command;
        response->success = true;
        response->result = 0;
      });
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1.0, max_frequency_)));
    timer_ = rclcpp::create_timer(
      get_node_base_interface(), get_node_timers_interface(), get_clock(), period,
      std::bind(&DroneVisualizer::publish, this));
  }

private:
  void publish()
  {
    std::scoped_lock lock(mutex_);
    if (!have_pose_) return;
    const auto stamp = now();
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = frame_id_;
    transform.child_frame_id = child_frame_id_;
    transform.transform.translation.x = local_pos_source_ == 1 && have_gps_ ? gps_x_east_ : pose_.pose.position.x;
    transform.transform.translation.y = local_pos_source_ == 1 && have_gps_ ? gps_y_north_ : pose_.pose.position.y;
    transform.transform.translation.z = pose_.pose.position.z;
    transform.transform.rotation = pose_.pose.orientation;
    tf_broadcaster_->sendTransform(transform);

    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    joints.name.assign(joint_names_.begin(), joint_names_.end());
    joints.position.resize(joint_names_.size());
    const auto velocity = armed_ ? 200.0 * 2.0 * M_PI / 60.0 : 0.0;
    for (auto & position : joint_positions_) position = std::remainder(position + velocity / std::max(1.0, max_frequency_), 2.0 * M_PI);
    joints.position.assign(joint_positions_.begin(), joint_positions_.end());
    joints_pub_->publish(joints);

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    marker.header.frame_id = child_frame_id_;
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.27;
    marker.pose.position.y = 0.27;
    marker.pose.position.z = 0.27;
    marker.pose.orientation = pose_.pose.orientation;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 0.9F;
    marker.color.r = 0.0F;
    marker.color.g = 0.0F;
    marker.color.b = 0.0F;
    marker.frame_locked = true;
    marker.text = marker_name_;
    marker_pub_->publish(marker);

    if (!history_enabled_) return;
    if (path_pub_->get_subscription_count() == 0) return;
    if (stamp.seconds() - last_path_publish_seconds_ <= 1.0 / 5.0) return;
    auto path_pose = pose_;
    path_pose.pose.position.x = transform.transform.translation.x;
    path_pose.pose.position.y = transform.transform.translation.y;
    path_pose.header.stamp = stamp;
    path_.push_back(path_pose);
    const auto max_path_size = static_cast<size_t>(std::max(0.0, history_seconds_ * 5.0));
    while (path_.size() > max_path_size) path_.pop_front();
    nav_msgs::msg::Path path_message;
    path_message.header.stamp = stamp;
    path_message.header.frame_id = frame_id_;
    path_message.poses.assign(path_.begin(), path_.end());
    path_pub_->publish(path_message);
    last_path_publish_seconds_ = stamp.seconds();
  }

  std::mutex mutex_;
  double max_frequency_{20.0};
  double history_seconds_{30.0};
  bool history_enabled_{true};
  bool armed_{false};
  bool have_pose_{false};
  bool have_gps_{false};
  int local_pos_source_{0};
  double origin_latitude_deg_{39.978861};
  double origin_longitude_deg_{116.339803};
  double gps_x_east_{0.0};
  double gps_y_north_{0.0};
  std::string frame_id_, child_frame_id_, marker_name_;
  std::array<std::string, 4> joint_names_;
  std::array<double, 4> joint_positions_{{0.0, 0.5, 2.6, 1.4}};
  double last_path_publish_seconds_{0.0};
  geometry_msgs::msg::PoseStamped pose_;
  std::deque<geometry_msgs::msg::PoseStamped> path_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_fix_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Service<mavros_msgs::srv::CommandAck>::SharedPtr switch_source_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneVisualizer>());
  rclcpp::shutdown();
  return 0;
}
