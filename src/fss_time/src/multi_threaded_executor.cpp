#include "fss_time/executors.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rcpputils/scope_exit.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"

namespace fss_time
{
namespace executors
{
MultiThreadedExecutor::MultiThreadedExecutor(
  const rclcpp::Node::SharedPtr & time_node,
  const rclcpp::ExecutorOptions & options,
  size_t number_of_threads,
  bool yield_before_execute,
  std::chrono::nanoseconds next_exec_timeout)
: rclcpp::Executor(options),
  time_node_(detail::require_time_node(time_node)),
  use_fss_sim_time_(detail::declare_or_get_parameter<bool>(*time_node_, "use_fss_sim_time", false)),
  yield_before_execute_(yield_before_execute),
  next_exec_timeout_(next_exec_timeout)
{
  number_of_threads_ = number_of_threads > 0 ?
    number_of_threads :
    std::max(std::thread::hardware_concurrency(), 2U);

  if (number_of_threads_ == 1) {
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"),
      "MultiThreadedExecutor is used with a single thread.\n"
      "Use the SingleThreadedExecutor instead.");
  }
}

MultiThreadedExecutor::~MultiThreadedExecutor() {}

void
MultiThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););
  std::vector<std::thread> threads;
  size_t thread_id = 0;
  {
    std::lock_guard wait_lock{wait_mutex_};
    for (; thread_id < number_of_threads_ - 1; ++thread_id) {
      auto func = std::bind(&MultiThreadedExecutor::run, this, thread_id);
      threads.emplace_back(func);
    }
  }

  run(thread_id);
  for (auto & thread : threads) {
    thread.join();
  }
}

size_t
MultiThreadedExecutor::get_number_of_threads()
{
  return number_of_threads_;
}

void
MultiThreadedExecutor::run([[maybe_unused]] size_t this_thread_number)
{
  if (!use_fss_sim_time_) {
    // Keep the vanilla rclcpp multi-threaded behavior when FSS simulated time is disabled.
    while (rclcpp::ok(this->context_) && spinning.load()) {
      rclcpp::AnyExecutable any_exec;
      {
        // Only one executor thread may wait on or take from the shared wait set at a time.
        std::lock_guard wait_lock{wait_mutex_};
        if (!rclcpp::ok(this->context_) || !spinning.load()) {
          return;
        }
        if (!get_next_executable(any_exec, next_exec_timeout_)) {
          continue;
        }
      }
      if (yield_before_execute_) {
        std::this_thread::yield();
      }

      execute_any_executable(any_exec);
      // Wake other executor threads when a mutually-exclusive callback group becomes available.
      detail::trigger_callback_group_guard_condition_if_needed(any_exec, interrupt_guard_condition_);
      any_exec.callback_group.reset();
    }
    return;
  }

  const auto participant_id = "multi_threaded_executor_" + std::to_string(this_thread_number);
  // Each executor worker thread has its own participant and safe-time announcements.
  auto & participant = thread_time_participant::for_current_thread(*time_node_, participant_id);

  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_exec;
    {
      // Protect the shared wait set while announcing safe time and taking work.
      std::lock_guard wait_lock{wait_mutex_};
      if (!rclcpp::ok(this->context_) || !spinning.load()) {
        return;
      }
      // While blocking for ROS work, this worker does not constrain sim-time progress.
      detail::announce_next_safe_time_infinite(participant);
      if (!get_next_executable(any_exec, next_exec_timeout_)) {
        continue;
      }
    }

    while (rclcpp::ok(this->context_) && spinning.load()) {
      if (yield_before_execute_) {
        std::this_thread::yield();
      }

      // Before running callbacks, pin this participant to the broker's current sim time.
      detail::announce_current_time(participant);
      execute_any_executable(any_exec);
      // Wake peer threads if this callback released a mutually-exclusive group.
      detail::trigger_callback_group_guard_condition_if_needed(any_exec, interrupt_guard_condition_);
      any_exec.callback_group.reset();

      rclcpp::AnyExecutable next_exec;
      {
        // Drain work already ready in the wait set without blocking for new work.
        std::lock_guard wait_lock{wait_mutex_};
        if (!rclcpp::ok(this->context_) || !spinning.load()) {
          return;
        }
        if (!get_next_executable(next_exec, std::chrono::nanoseconds(0))) {
          // No immediately-ready work remains, so release this worker's sim-time constraint.
          detail::announce_next_safe_time_infinite(participant);
          break;
        }
      }
      any_exec = next_exec;
    }
  }
}

}  // namespace executors
}  // namespace fss_time
