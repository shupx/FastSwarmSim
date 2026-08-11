#include <cmath>
#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "fss_time/fss_time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class PerfectMavrosDrone : public rclcpp::Node
{
public:
  PerfectMavrosDrone()
  : Node("perfect_mavros_quadrotor_sim")
  {
    pose_publish_rate_ = declare_parameter<double>("pose_publish_rate", 30.0);
    velocity_publish_rate_ = declare_parameter<double>("velocity_publish_rate", 30.0);
    odom_publish_rate_ = declare_parameter<double>("odom_publish_rate", 30.0);
    state_publish_rate_ = declare_parameter<double>("state_publish_rate", 10.0);
    const auto init_x = declare_parameter<double>("init_x", 0.0);
    const auto init_y = declare_parameter<double>("init_y", 0.0);
    const auto init_z = declare_parameter<double>("init_z", 0.0);
    const auto init_yaw_deg = declare_parameter<double>("init_yaw", 0.0);

    setpoint_sub_ = create_subscription<mavros_msgs::msg::PositionTarget>(
      "mavros/setpoint_raw/local", rclcpp::QoS(10),
      [this](const mavros_msgs::msg::PositionTarget::SharedPtr msg) { setpoint_callback(msg); });
    set_mode_srv_ = create_service<mavros_msgs::srv::SetMode>(
      "mavros/set_mode",
      [this](
        const std::shared_ptr<mavros_msgs::srv::SetMode::Request> request,
        std::shared_ptr<mavros_msgs::srv::SetMode::Response> response) {
        {
          std::scoped_lock lock(state_mutex_);
          current_state_.mode = request->custom_mode;
        }
        response->mode_sent = true;
      });
    arming_srv_ = create_service<mavros_msgs::srv::CommandBool>(
      "mavros/cmd/arming",
      [this](
        const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> request,
        std::shared_ptr<mavros_msgs::srv::CommandBool::Response> response) {
        {
          std::scoped_lock lock(state_mutex_);
          current_state_.armed = request->value;
        }
        response->success = true;
        response->result = 0;
      });

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "mavros/local_position/pose", rclcpp::QoS(10));
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "mavros/local_position/velocity_local", rclcpp::QoS(10));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "mavros/local_position/odom", rclcpp::QoS(10));
    state_pub_ = create_publisher<mavros_msgs::msg::State>("mavros/state", rclcpp::QoS(10));

    initialize_state(init_x, init_y, init_z, init_yaw_deg * M_PI / 180.0);
    last_pose_publish_ = now();
    last_velocity_publish_ = now();
    last_odom_publish_ = now();
    last_state_publish_ = now();

    RCLCPP_INFO(get_logger(), "Perfect MAVROS drone ready on relative namespace '%s'", get_namespace());
  }

  void run()
  {
    fss_time::Rate rate(*this, 100.0);
    while (rclcpp::ok()) {
      const auto stamp = now();

      {
        std::scoped_lock lock(state_mutex_);
        publish_due(stamp);
      }

      rate.sleep();
    }
  }

private:
  void initialize_state(double x, double y, double z, double yaw)
  {
    current_pose_.header.frame_id = "map";
    current_pose_.pose.position.x = x;
    current_pose_.pose.position.y = y;
    current_pose_.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    current_pose_.pose.orientation = tf2::toMsg(q);

    current_velocity_.header.frame_id = "map";
    current_odom_.header.frame_id = "map";
    current_odom_.child_frame_id = "base_link";
    current_odom_.pose.pose = current_pose_.pose;
    current_odom_.twist.twist = current_velocity_.twist;

    current_state_.connected = true;
    current_state_.armed = true;
    current_state_.guided = true;
    current_state_.manual_input = false;
    current_state_.mode = "OFFBOARD";
    current_state_.system_status = 4;
  }

  void setpoint_callback(const mavros_msgs::msg::PositionTarget::SharedPtr msg)
  {
    std::scoped_lock lock(state_mutex_);
    const auto stamp = now();
    if (current_state_.mode != "OFFBOARD" || !current_state_.armed) {
      hold_position(stamp);
      return;
    }

    current_pose_.header.stamp = stamp;
    const auto dt = last_setpoint_time_.nanoseconds() == 0 ? 0.0 :
      std::max(0.0, (stamp - last_setpoint_time_).seconds());
    last_setpoint_time_ = stamp;
    if ((msg->type_mask & mavros_msgs::msg::PositionTarget::IGNORE_PX) == 0) {
      current_pose_.pose.position.x = msg->position.x;
    } else if ((msg->type_mask & mavros_msgs::msg::PositionTarget::IGNORE_VX) == 0) {
      current_pose_.pose.position.x += msg->velocity.x * dt;
    }
    if ((msg->type_mask & mavros_msgs::msg::PositionTarget::IGNORE_PY) == 0) {
      current_pose_.pose.position.y = msg->position.y;
    } else if ((msg->type_mask & mavros_msgs::msg::PositionTarget::IGNORE_VY) == 0) {
      current_pose_.pose.position.y += msg->velocity.y * dt;
    }
    if ((msg->type_mask & mavros_msgs::msg::PositionTarget::IGNORE_PZ) == 0) {
      current_pose_.pose.position.z = msg->position.z;
    } else if ((msg->type_mask & mavros_msgs::msg::PositionTarget::IGNORE_VZ) == 0) {
      current_pose_.pose.position.z += msg->velocity.z * dt;
    }

    if ((msg->type_mask & mavros_msgs::msg::PositionTarget::IGNORE_YAW) == 0) {
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, msg->yaw);
      current_pose_.pose.orientation = tf2::toMsg(q);
    } else if ((msg->type_mask & mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE) == 0) {
      tf2::Quaternion q_current;
      tf2::fromMsg(current_pose_.pose.orientation, q_current);
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw + msg->yaw_rate * dt);
      current_pose_.pose.orientation = tf2::toMsg(q);
    }

    current_velocity_.header.stamp = stamp;
    current_velocity_.twist.linear.x =
      ((msg->type_mask & mavros_msgs::msg::PositionTarget::IGNORE_VX) == 0) ? msg->velocity.x : 0.0;
    current_velocity_.twist.linear.y =
      ((msg->type_mask & mavros_msgs::msg::PositionTarget::IGNORE_VY) == 0) ? msg->velocity.y : 0.0;
    current_velocity_.twist.linear.z =
      ((msg->type_mask & mavros_msgs::msg::PositionTarget::IGNORE_VZ) == 0) ? msg->velocity.z : 0.0;
    current_velocity_.twist.angular.z =
      ((msg->type_mask & mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE) == 0) ?
      msg->yaw_rate : 0.0;

    sync_odom(stamp);
  }

  void hold_position(const rclcpp::Time & stamp)
  {
    tf2::Quaternion q_current;
    tf2::fromMsg(current_pose_.pose.orientation, q_current);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw);
    tf2::Quaternion q_hover;
    q_hover.setRPY(0.0, 0.0, yaw);
    current_pose_.pose.orientation = tf2::toMsg(q_hover);
    current_pose_.header.stamp = stamp;
    current_velocity_.header.stamp = stamp;
    current_velocity_.twist = geometry_msgs::msg::Twist();
    sync_odom(stamp);
  }

  void sync_odom(const rclcpp::Time & stamp)
  {
    current_odom_.header.stamp = stamp;
    current_odom_.pose.pose = current_pose_.pose;
    current_odom_.twist.twist = current_velocity_.twist;
  }

  void publish_pose()
  {
    current_pose_.header.stamp = now();
    pose_pub_->publish(current_pose_);
  }

  void publish_velocity()
  {
    current_velocity_.header.stamp = now();
    velocity_pub_->publish(current_velocity_);
  }

  void publish_odom()
  {
    sync_odom(now());
    odom_pub_->publish(current_odom_);
  }

  void publish_state()
  {
    current_state_.header.stamp = now();
    state_pub_->publish(current_state_);
  }

  void publish_due(const rclcpp::Time & stamp)
  {
    if ((stamp - last_pose_publish_).seconds() >= 1.0 / std::max(1.0, pose_publish_rate_)) {
      publish_pose();
      last_pose_publish_ = stamp;
    }
    if ((stamp - last_velocity_publish_).seconds() >= 1.0 / std::max(1.0, velocity_publish_rate_)) {
      publish_velocity();
      last_velocity_publish_ = stamp;
    }
    if ((stamp - last_odom_publish_).seconds() >= 1.0 / std::max(1.0, odom_publish_rate_)) {
      publish_odom();
      last_odom_publish_ = stamp;
    }
    if ((stamp - last_state_publish_).seconds() >= 1.0 / std::max(1.0, state_publish_rate_)) {
      publish_state();
      last_state_publish_ = stamp;
    }
  }

  rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_sub_;
  rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr set_mode_srv_;
  rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr arming_srv_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub_;
  std::mutex state_mutex_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::TwistStamped current_velocity_;
  nav_msgs::msg::Odometry current_odom_;
  mavros_msgs::msg::State current_state_;
  double pose_publish_rate_{30.0};
  double velocity_publish_rate_{30.0};
  double odom_publish_rate_{30.0};
  double state_publish_rate_{10.0};
  rclcpp::Time last_setpoint_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_velocity_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_state_publish_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PerfectMavrosDrone>();

  std::thread spin_thread([node]() {
    rclcpp::spin(node);
  });

  node->run();
  
  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  return 0;
}
