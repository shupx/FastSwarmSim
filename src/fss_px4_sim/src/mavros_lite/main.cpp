#include "fss_px4_sim/mavros_lite/core.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<fss_px4_sim::mavros_lite::Core>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mavros_lite"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
