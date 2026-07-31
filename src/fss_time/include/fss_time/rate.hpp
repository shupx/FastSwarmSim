#ifndef FSS_TIME_RATE_HPP_
#define FSS_TIME_RATE_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace fss_time
{

/**
 * @brief Fixed-period loop helper that uses fss_time::sleep_for().
 *
 * The rate uses the supplied node's clock for interval accounting. During
 * construction, if the node parameter `use_fss_sim_time` is true, the node's
 * `use_sim_time` parameter is enabled before the initial interval time is read.
 * Each sleep cycle uses a cached fss_time setting and avoids repeated
 * parameter lookups.
 */
class Rate
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Rate)

  /**
   * @brief Construct a Rate from a frequency in hertz.
   * @param node Node whose clock and fss_time parameters are used.
   * @param rate Frequency in hertz. Must be greater than zero.
   * @throws std::invalid_argument if rate is not greater than zero.
   */
  explicit Rate(
    rclcpp::Node & node,
    const double rate);

  /**
   * @brief Construct a Rate from a period.
   * @param node Node whose clock and fss_time parameters are used.
   * @param period Loop period. Must be greater than zero.
   * @throws std::invalid_argument if period is not greater than zero.
   */
  explicit Rate(
    rclcpp::Node & node,
    const rclcpp::Duration & period);

  /**
   * @brief Sleep until the next cycle boundary.
   * @return true if sleep occurred, false if the cycle was already missed or sleep was interrupted.
   */
  bool sleep();

  /**
   * @brief Return the rcl clock type used for interval accounting.
   * @return Clock type from the node clock.
   */
  rcl_clock_type_t get_type() const;

  /**
   * @brief Check whether the underlying clock is steady.
   * @return true if the node clock type is RCL_STEADY_TIME.
   */
  bool is_steady() const;

  /**
   * @brief Reset the next cycle boundary to start from the current time.
   */
  void reset();

  /**
   * @brief Return the configured period as std::chrono nanoseconds.
   * @return Configured loop period.
   */
  std::chrono::nanoseconds period() const;

private:
  RCLCPP_DISABLE_COPY(Rate)

  /**
   * @brief Initialize simulated-time settings for fss_time-aware rate sleeps.
   *
   * If `use_fss_sim_time` is true, this enables `use_sim_time` before
   * last_interval_ is initialized from the node clock.
   */
  void initialize_fss_sim_time();

  /**
   * @brief Sleep for a duration using cached fss_time settings.
   * @param rel_time Relative duration to sleep for.
   * @return true if the underlying clock sleep reaches its target.
   */
  bool sleep_for(const rclcpp::Duration & rel_time);

  rclcpp::Node & node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Duration period_;
  rclcpp::Time last_interval_;
  bool use_fss_sim_time_{false};
};

}  // namespace fss_time

#endif  // FSS_TIME_RATE_HPP_
