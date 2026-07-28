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

#include "rclcpp/any_executable.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fss_time
{
namespace executors
{

class SingleThreadedExecutor : public rclcpp::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SingleThreadedExecutor)

  explicit SingleThreadedExecutor(
    const rclcpp::Node::SharedPtr & time_node,
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());

  ~SingleThreadedExecutor() override;

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

  explicit MultiThreadedExecutor(
    const rclcpp::Node::SharedPtr & time_node,
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions(),
    size_t number_of_threads = 0,
    bool yield_before_execute = false,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  ~MultiThreadedExecutor() override;

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





namespace detail
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

template<typename GuardConditionT>
void trigger_guard_condition(GuardConditionT & guard_condition)
{
  guard_condition.trigger();
}

template<typename GuardConditionT>
void trigger_guard_condition(const std::shared_ptr<GuardConditionT> & guard_condition)
{
  guard_condition->trigger();
}

template<typename GuardConditionT>
void trigger_callback_group_guard_condition_if_needed(
  rclcpp::AnyExecutable & any_exec,
  GuardConditionT & interrupt_guard_condition)
{
  if (any_exec.callback_group &&
    any_exec.callback_group->type() == rclcpp::CallbackGroupType::MutuallyExclusive)
  {
    try {
      trigger_guard_condition(interrupt_guard_condition);
    } catch (const rclcpp::exceptions::RCLError & ex) {
      throw std::runtime_error(
              std::string(
                "Failed to trigger guard condition on callback group change: ") + ex.what());
    }
  }
}

}  // namespace detail

}  // namespace executors
}  // namespace fss_time

#endif  // FSS_TIME_EXECUTORS_HPP_
