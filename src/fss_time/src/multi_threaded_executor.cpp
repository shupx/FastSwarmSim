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
namespace
{

/** @brief Compatibility shim: Humble stores interrupt_guard_condition_ as an object, while Rolling/newer rclcpp stores it as a shared_ptr.
*/
template<typename GuardConditionT>
void trigger_guard_condition_compat(GuardConditionT & guard_condition)
{
  guard_condition.trigger();
}

/** @brief Compatibility shim: Humble stores interrupt_guard_condition_ as an object, while Rolling/newer rclcpp stores it as a shared_ptr.
*/
template<typename GuardConditionT>
void trigger_guard_condition_compat(const std::shared_ptr<GuardConditionT> & guard_condition)
{
  guard_condition->trigger();
}

}  // namespace

MultiThreadedExecutor::MultiThreadedExecutor(
  const rclcpp::ExecutorOptions & options,
  size_t number_of_threads,
  bool yield_before_execute,
  std::chrono::nanoseconds next_exec_timeout)
: fss_time::Executor(options),
  yield_before_execute_(yield_before_execute),
  next_exec_timeout_(next_exec_timeout)
{
  /* The same as the rclcpp::executors::MultiThreadedExecutor method */
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
  /* The same as the rclcpp::executors::MultiThreadedExecutor::spin() method */
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

/** @brief The same as the rclcpp::executors::MultiThreadedExecutor::get_number_of_threads() method */
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
      if (any_exec.callback_group &&
        any_exec.callback_group->type() == rclcpp::CallbackGroupType::MutuallyExclusive)
      {
        try {
          trigger_guard_condition_compat(interrupt_guard_condition_);
        } catch (const rclcpp::exceptions::RCLError & ex) {
          throw std::runtime_error(
                  std::string(
                    "Failed to trigger guard condition on callback group change: ") + ex.what());
        }
      }
      any_exec.callback_group.reset();
    }
    return;
  }

  const auto participant_id = "multi_threaded_executor_" + std::to_string(this_thread_number);
  // Each executor worker thread has its own participant and safe-time announcements.
  auto & participant = thread_time_participant::for_current_thread(*time_node_, participant_id);

  while (rclcpp::ok(this->context_) && spinning.load()) {
    // While get_next_executable(any_exec, next_exec_timeout_) with next_exec_timeout_=-1 by default blocks for ROS work, this worker does not constrain sim-time progress as thread_time_participant announces infinite safe time.
    fss_time_tools::announce_next_safe_time_infinite(participant);

    // std::cout << "thread " << this_thread_number << " announce_next_safe_time_infinite" << std::endl;

    rclcpp::AnyExecutable any_exec;
    bool got_work = false;
    while (rclcpp::ok(this->context_) && !got_work)
    {
      std::lock_guard wait_lock{wait_mutex_};
      if (!rclcpp::ok(this->context_) || !spinning.load()) {
        return;
      }
      got_work = get_next_executable(any_exec, next_exec_timeout_);
      // get_next_executable(x, -1) blocks only until the next timer time. If sim time is paused, this will return with false (no work ready). So a while loop is needed to keep waiting for work in sim time paused state.

      // if (!got_work) {
      //   std::cout << "thread " << this_thread_number << " get_next_executable() returned false, no work ready" << std::endl;
      // }
    }

    // Before executing any_executable callbacks, pin this thread_time_participant to the broker's current sim time, so that the sim time does not advance while callbacks are running. 
    fss_time_tools::announce_current_time(participant);

    // std::cout << "thread " << this_thread_number << " announce_current_time: " << participant.get_last_safe_time().nanoseconds() << " ns" << std::endl;

    while (rclcpp::ok(this->context_) && spinning.load()) {
      if (yield_before_execute_) {
        std::this_thread::yield();
      }

      // std::cout << "thread " << this_thread_number << " execute_any_executable: " << any_exec.callback_group.get() << std::endl;

      execute_any_executable(any_exec);

      // std::cout << "thread " << this_thread_number << " execute_any_executable done: " << any_exec.callback_group.get() << std::endl;

      // Wake peer threads if this callback released a mutually-exclusive group. (original rclcpp::executors::MultiThreadedExecutor behavior)
      if (any_exec.callback_group &&
        any_exec.callback_group->type() == rclcpp::CallbackGroupType::MutuallyExclusive)
      {
        try {
          trigger_guard_condition_compat(interrupt_guard_condition_);
        } catch (const rclcpp::exceptions::RCLError & ex) {
          throw std::runtime_error(
                  std::string(
                    "Failed to trigger guard condition on callback group change: ") + ex.what());
        }
      }
      any_exec.callback_group.reset();

      // std::cout << "thread " << this_thread_number << " check for more work ready to execute" << std::endl;

      // check if there is more work ready to execute, and if so, continue executing callbacks without releasing the sim-time constraint. If there is no more work ready, release the sim-time constraint again by announcing infinite safe time.
      rclcpp::AnyExecutable next_exec;
      {
        // only using try_to_lock. If this mutex is locked by other executor threads, then this thread does not need to check for more work ready to execute, and will proceed to the next loop and release the sim-time constraint by announcing infinite safe time. 
        std::unique_lock lock(wait_mutex_, std::try_to_lock); 
        if (!lock.owns_lock()) {
          break;
        } 
        else {
          if (!rclcpp::ok(this->context_) || !spinning.load()) {
            return;
          }
          // get_next_executable(next_exec, 0) drains work already ready in the wait set without blocking.
          if (!get_next_executable(next_exec, std::chrono::nanoseconds(0))) {
            // No immediately-ready work remains, so release this worker's sim-time constraint.
            break;
            // The speed_regulator in sim_time_broker will push one time step forward if all thread_time_participants have released their safe-time constraints. So the step of the speed_regulator determines the sim time step in this case, and a small step is preferred and more accurated.
          }
        }
        any_exec = next_exec;

        // std::cout << "thread " << this_thread_number << " more work ready to execute: " << any_exec.callback_group.get() << std::endl;
      }
    }
  }
}

}  // namespace executors
}  // namespace fss_time
