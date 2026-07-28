#ifndef FSS_TIME_EXECUTORS_HPP_
#define FSS_TIME_EXECUTORS_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "fss_time/thread_time_participant.hpp"
#include "fss_time/time_types.hpp"

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fss_time
{

/// 
/** 
 * Create a default fss_time::executors::SingleThreadedExecutor and spin the specified node.
 * \param[in] node_ptr Shared pointer to the node to spin. 
 */
void spin(const rclcpp::Node::SharedPtr & node_ptr);


namespace executors
{

/// Single-threaded executor implementation with fss_time support.
/**
 * This is the default executor created by fss_time::spin.
 */
class SingleThreadedExecutor : public rclcpp::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SingleThreadedExecutor)

  /** 
   * Single-threaded executor implementation with fss_time support. This is the default executor created by fss_time::spin.
   * \param[in] time_node Shared pointer to the node that will be used to announce safe time and pin to sim time.
   * \param[in] options Options used to configure the executor. Default options will use the default memory strategy and the global default context.
   */
  explicit SingleThreadedExecutor(
    const rclcpp::Node::SharedPtr & time_node,
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());

  ~SingleThreadedExecutor() override;

  /// Single-threaded implementation of spin.
  /**
   * This function will block until work comes in, execute it, and then repeat
   * the process until canceled. And fss_time::thread_time_participant will announce infinite safe time while waiting for work, and pin to the broker's current sim time while executing callbacks.
   * It may be interrupt by a call to rclcpp::Executor::cancel() or by ctrl-c
   * if the associated context is configured to shutdown on SIGINT.
   * \throws std::runtime_error when spin() called while already spinning
   */
  void spin() override;

private:
  RCLCPP_DISABLE_COPY(SingleThreadedExecutor)

  rclcpp::Node::SharedPtr time_node_;
  bool use_fss_sim_time_{false};
};


class MultiThreadedExecutor : public rclcpp::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MultiThreadedExecutor)

  /**
   * Multi-threaded executor implementation with fss_time support.
   * \param[in] time_node Shared pointer to the node that will be used to announce safe time and pin to sim time.
   * \param[in] options Options used to configure the executor. Default options will use the default memory strategy and the global default context.
   * \param[in] number_of_threads number of threads to have in the thread pool,
   *   the default 0 will use the number of cpu cores found (minimum of 2)
   * \param[in] yield_before_execute if true std::this_thread::yield() is called after acquiring work (as an AnyExecutable) and releasing the spinning lock, but before executing the work. This is useful for reproducing some bugs related to taking work more than once. Default is false.
   * \param[in] timeout maximum time to wait. Default is -1, which means wait indefinitely for work.
   */
  explicit MultiThreadedExecutor(
    const rclcpp::Node::SharedPtr & time_node,
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions(),
    size_t number_of_threads = 0,
    bool yield_before_execute = false,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  ~MultiThreadedExecutor() override;

  /**
   * \sa This inner multi-threaded run() will block until work comes in, execute it, and then repeat
   * the process until canceled. And fss_time::thread_time_participant will announce infinite safe time while waiting for work, and pin to the broker's current sim time while executing callbacks.
   * \throws std::runtime_error when spin() called while already spinning
   */
  void spin() override;

  size_t get_number_of_threads();

protected:
  void run(size_t this_thread_number);

private:
  RCLCPP_DISABLE_COPY(MultiThreadedExecutor)

  rclcpp::Node::SharedPtr time_node_;
  bool use_fss_sim_time_{false};
  std::mutex wait_mutex_;
  size_t number_of_threads_;
  bool yield_before_execute_;
  std::chrono::nanoseconds next_exec_timeout_;
};


namespace fss_time_tools
{

template<typename T>
T declare_or_get_parameter(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<T>(name, default_value);
  }

  T value{};
  node.get_parameter(name, value);
  return value;
}

inline rclcpp::Node::SharedPtr require_time_node(const rclcpp::Node::SharedPtr & time_node)
{
  if (!time_node) {
    throw std::invalid_argument("fss_time executor requires a non-null time_node");
  }
  return time_node;
}

inline void announce_next_safe_time_infinite(thread_time_participant & participant)
{
  participant.announce_next_safe_time(rclcpp::Time(fss_time::kInfiniteTimeNs, RCL_ROS_TIME));
}

inline void announce_current_time(thread_time_participant & participant)
{
  participant.announce_next_safe_time(participant.get_sim_time());
}

}  // namespace fss_time_tools

}  // namespace executors
}  // namespace fss_time

#endif  // FSS_TIME_EXECUTORS_HPP_
