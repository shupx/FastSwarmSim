#include <memory>
#include <algorithm>
#include <string>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "fastswarm_sensing/local_pointcloud_sim_config.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "marsim_render/marsim_render.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace fastswarm_sensing
{

using Vec3f = Eigen::Matrix<float, 3, 1>;

class LocalPointCloudSimulator : public rclcpp::Node
{
public:
  LocalPointCloudSimulator()
  : Node("local_pointcloud_sim")
  {
    const auto default_config_path =
      ament_index_cpp::get_package_share_directory("fastswarm_sensing") +
      "/config/local_pointcloud_sim.yaml";
    auto config_path = declare_parameter<std::string>("config_path", default_config_path);
    if (config_path.empty()) {
      config_path = default_config_path;
    }
    config_ = LocalPointCloudSimConfig(config_path);

    RCLCPP_INFO(get_logger(), "Loading local point cloud config: %s", config_path.c_str());
    render_ptr_ = std::make_shared<marsim::MarsimRender>(config_path);

    if (config_.use_odom) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        config_.odom_topic,
        rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });
    } else {
      pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        config_.pose_topic,
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_callback(msg); });
    }

    local_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      config_.local_pc_topic, rclcpp::QoS(10));
    global_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      config_.global_pc_topic, rclcpp::QoS(1).transient_local());

    t_start_ = now();
    const auto period = std::chrono::duration<double>(1.0 / std::max(1, config_.sensing_rate));
    local_pc_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish_local_pointcloud(); });
  }

private:
  void publish_local_pointcloud()
  {
    if (!pose_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for pose or odom input");
      return;
    }

    if (local_pc_pub_->get_subscription_count() == 0) {
      return;
    }

    try {
      pcl::PointCloud<marsim::PointType>::Ptr local_cloud(new pcl::PointCloud<marsim::PointType>);
      render_ptr_->renderOnceInWorld(
        current_position_,
        current_quaternion_,
        (last_pose_time_ - t_start_).seconds(),
        local_cloud);

      sensor_msgs::msg::PointCloud2 pc_msg;
      pcl::toROSMsg(*local_cloud, pc_msg);
      pc_msg.header.frame_id = config_.frame_id;
      pc_msg.header.stamp = last_pose_time_;
      local_pc_pub_->publish(pc_msg);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Error publishing local point cloud: %s", e.what());
    }
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_position_ = Vec3f(
      static_cast<float>(msg->pose.position.x),
      static_cast<float>(msg->pose.position.y),
      static_cast<float>(msg->pose.position.z));
    current_quaternion_ = Eigen::Quaternionf(
      static_cast<float>(msg->pose.orientation.w),
      static_cast<float>(msg->pose.orientation.x),
      static_cast<float>(msg->pose.orientation.y),
      static_cast<float>(msg->pose.orientation.z));
    last_pose_time_ = rclcpp::Time(msg->header.stamp);
    pose_received_ = true;
    publish_global_pointcloud();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_position_ = Vec3f(
      static_cast<float>(msg->pose.pose.position.x),
      static_cast<float>(msg->pose.pose.position.y),
      static_cast<float>(msg->pose.pose.position.z));
    current_quaternion_ = Eigen::Quaternionf(
      static_cast<float>(msg->pose.pose.orientation.w),
      static_cast<float>(msg->pose.pose.orientation.x),
      static_cast<float>(msg->pose.pose.orientation.y),
      static_cast<float>(msg->pose.pose.orientation.z));
    last_pose_time_ = rclcpp::Time(msg->header.stamp);
    pose_received_ = true;
    publish_global_pointcloud();
  }

  void publish_global_pointcloud()
  {
    const auto sub_count = global_pc_pub_->get_subscription_count();
    if (sub_count == 0 || sub_count == last_global_sub_count_) {
      last_global_sub_count_ = sub_count;
      return;
    }

    try {
      pcl::PointCloud<marsim::PointType>::Ptr global_cloud(new pcl::PointCloud<marsim::PointType>);
      render_ptr_->getGlobalMap(global_cloud);

      sensor_msgs::msg::PointCloud2 pc_msg;
      pcl::toROSMsg(*global_cloud, pc_msg);
      pc_msg.header.frame_id = config_.frame_id;
      pc_msg.header.stamp = now();
      global_pc_pub_->publish(pc_msg);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Error publishing global point cloud: %s", e.what());
    }
    last_global_sub_count_ = sub_count;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pc_pub_;
  rclcpp::TimerBase::SharedPtr local_pc_timer_;

  std::shared_ptr<marsim::MarsimRender> render_ptr_;
  LocalPointCloudSimConfig config_;
  Vec3f current_position_{0.0f, 0.0f, 0.0f};
  Eigen::Quaternionf current_quaternion_{Eigen::Quaternionf::Identity()};
  rclcpp::Time t_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  bool pose_received_{false};
  size_t last_global_sub_count_{0};
};

}  // namespace fastswarm_sensing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fastswarm_sensing::LocalPointCloudSimulator>());
  rclcpp::shutdown();
  return 0;
}
