#ifndef FSS_TIME_EXECUTORS_HPP_
#define FSS_TIME_EXECUTORS_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "fss_time/thread_time_participant.hpp"
#include "fss_time/tools.hpp"

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fss_time
{
/// @brief fss_time::executors namespace contains executor implementations with fss_time support.

/** 
 * Create a default fss_time::executors::SingleThreadedExecutor and spin the specified node.
 * \param[in] node_ptr Shared pointer to the node to spin. 
 */
void spin(const rclcpp::Node::SharedPtr & node_ptr);

/**
 * @brief fss_time::Executor is a base class for executors with fss_time support.
 * It is derived from rclcpp::Executor and adds support for fss_time::thread_time_participant and fss_time::fss_time_tools.
 * It is used as a base class for fss_time::executors::SingleThreadedExecutor and fss_time::executors::MultiThreadedExecutor.
 * It is not intended to be used directly, but rather through the fss_time::executors::SingleThreadedExecutor and fss_time::executors::MultiThreadedExecutor classes.
 */
class Executor : public rclcpp::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Executor)

  using rclcpp::Executor::add_node;
  using rclcpp::Executor::remove_node;

  /**
   * @brief Construct the fss_time executor base.
   * @param options rclcpp executor options.
   */
  explicit Executor(const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions())
  : rclcpp::Executor(options)
  {
  }

  /**
   * @brief Destroy the fss_time executor base.
   */
  ~Executor() override = default;

  /**
   * @brief Override rclcpp::Executor::add_node to set the time node for fss_time support.
   */
  void add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true) override
  {
    set_time_node(node_ptr);
    rclcpp::Executor::add_node(node_ptr, notify);
  }

  /**
   * @brief Override rclcpp::Executor::remove_node to reset the time node for fss_time support.
   */
  void remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true) override
  {
    rclcpp::Executor::remove_node(node_ptr, notify);
    if (time_node_ == node_ptr) {
      time_node_.reset();
      use_fss_sim_time_ = false;
    }
  }

protected:
  /**
   * @brief Set the time node for fss_time support. And declare or get the `use_fss_sim_time` parameter from the node.
   * If `use_fss_sim_time` is true, ensure that `use_sim_time` is also true.
   * @param time_node Shared pointer to the time node.
   */
  void set_time_node(const rclcpp::Node::SharedPtr & time_node)
  {
    time_node_ = time_node;
    use_fss_sim_time_ =
      fss_time_tools::declare_or_get_parameter<bool>(*time_node_, "use_fss_sim_time", false);
    if (use_fss_sim_time_) {
      fss_time_tools::wait_until_ros_time_is_active(*time_node_);  // wait for the use_sim_time parameter to take effect.
      
      // set use_sim_time to true
      // fss_time_tools::ensure_bool_parameter_enabled(*time_node_, "use_sim_time");
      // Note that use_sim_time will not take effect until the node is spun and the parameter change and clock subscription is processed.
    }
  }

  rclcpp::Node::SharedPtr time_node_;
  bool use_fss_sim_time_{false};

private:
  RCLCPP_DISABLE_COPY(Executor)
};


namespace executors
{

/// Single-threaded executor implementation with fss_time support.
/**
 * This is the default executor created by fss_time::spin.
 */
class SingleThreadedExecutor : public fss_time::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SingleThreadedExecutor)

  using fss_time::Executor::add_node;
  using fss_time::Executor::remove_node;

  /** 
   * Single-threaded executor implementation with fss_time support. This is the default executor created by fss_time::spin. 
   * If the ROS paramter `use_fss_sim_time` of the last added node is false, this executor acts like a normal rclcpp::SingleThreadedExecutor. 
   * If `use_fss_sim_time` is true, this executor will add fss_time announcement using fss_time::thread_time_participant while getting and executing work. 
   * \param[in] options Options used to configure the executor. Default options will use the default memory strategy and the global default context.
   */
  explicit SingleThreadedExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());

  /**
   * @brief Destroy the single-threaded fss_time executor.
   */
  ~SingleThreadedExecutor() override;

  /// Single-threaded implementation of spin.
  /**
   * This function will block until work comes in, execute it, and then repeat
   * the process until canceled. 
   * If `use_fss_sim_time` is true, fss_time::thread_time_participant will announce infinite safe time while waiting for work, and pin to the coordinator's current sim time while executing callbacks.
   * \throws std::runtime_error when spin() called while already spinning
   */
  void spin() override;

private:
  RCLCPP_DISABLE_COPY(SingleThreadedExecutor)
};


class MultiThreadedExecutor : public fss_time::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MultiThreadedExecutor)

  using fss_time::Executor::add_node;
  using fss_time::Executor::remove_node;

  /**
   * Multi-threaded executor implementation with fss_time support.
   * If the ROS parameter `use_fss_sim_time` of the last added node is false, this executor acts like a normal rclcpp::MultiThreadedExecutor. 
   * If `use_fss_sim_time` is true, this executor will add fss_time announcement using fss_time::thread_time_participant while getting and executing work.
   * \param[in] options Options used to configure the executor. Default options will use the default memory strategy and the global default context.
   * \param[in] number_of_threads number of threads to have in the thread pool,
   *   the default 0 will use the number of cpu cores found (minimum of 2)
   * \param[in] yield_before_execute if true std::this_thread::yield() is called after acquiring work (as an AnyExecutable) and releasing the spinning lock, but before executing the work. This is useful for reproducing some bugs related to taking work more than once. Default is false.
   * \param[in] timeout maximum time to wait. Default is -1, which means wait indefinitely for work.
   */
  explicit MultiThreadedExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions(),
    size_t number_of_threads = 0,
    bool yield_before_execute = false,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /**
   * @brief Destroy the multi-threaded fss_time executor.
   */
  ~MultiThreadedExecutor() override;

  /**
   * @brief The inner multi-threaded run() will block until work comes in, execute it, 
   * and then repeat the process until canceled. 
   * If `use_fss_sim_time` is true, fss_time::thread_time_participant will announce infinite safe time while waiting for work, 
   * and pin to the coordinator's current sim time while executing callbacks.
   * \throws std::runtime_error when spin() called while already spinning
   */
  void spin() override;

  /**
   * @brief Return the configured executor worker thread count.
   * @return Number of worker threads.
   */
  size_t get_number_of_threads();

protected:
  /**
   * @brief Run one multi-threaded executor worker loop.
   * @param this_thread_number Worker index.
   */
  void run(size_t this_thread_number);

private:
  RCLCPP_DISABLE_COPY(MultiThreadedExecutor)
  std::mutex wait_mutex_;
  size_t number_of_threads_;
  bool yield_before_execute_;
  std::chrono::nanoseconds next_exec_timeout_;
};

}  // namespace executors
}  // namespace fss_time

#endif  // FSS_TIME_EXECUTORS_HPP_
