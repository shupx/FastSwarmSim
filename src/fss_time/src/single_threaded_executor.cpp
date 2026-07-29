#include "fss_time/executors.hpp"

#include <chrono>
#include <stdexcept>

#include "rcpputils/scope_exit.hpp"

#include "rclcpp/utilities.hpp"

namespace fss_time
{
namespace executors
{
SingleThreadedExecutor::SingleThreadedExecutor(
  const rclcpp::ExecutorOptions & options)
: fss_time::Executor(options)
{
}

SingleThreadedExecutor::~SingleThreadedExecutor() {}

void
SingleThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););

  if (!use_fss_sim_time_) {
    // Keep the vanilla rclcpp behavior when FSS simulated time is disabled.
    while (rclcpp::ok(this->context_) && spinning.load()) {
      rclcpp::AnyExecutable any_executable;
      if (get_next_executable(any_executable)) {
        execute_any_executable(any_executable);
      }
    }
    return;
  }

  auto & participant =
    thread_time_participant::for_current_thread(*time_node_, "single_threaded_executor");

  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_executable;
    // While get_next_executable(any_executable) blocks for ROS work, this executor thread does not constrain sim-time progress as thread_time_participant announces infinite safe time.
    fss_time_tools::announce_next_safe_time_infinite(participant);
    while (!get_next_executable(any_executable)) {
      // get_next_executable(x, -1) blocks until the next timer triggering walltime. If sim time is paused, this will return with false (no work ready). So a while loop is needed to keep waiting for work in sim time paused state.
    }

    while (rclcpp::ok(this->context_) && spinning.load()) {
      // Before executing any_executable callbacks, pin this thread_time_participant to the broker's current sim time, so that the sim time does not advance while callbacks are running. 
      fss_time_tools::announce_current_time(participant);
      execute_any_executable(any_executable);
      any_executable.callback_group.reset();

      // check if there is more work ready to execute, and if so, continue executing callbacks without releasing the sim-time constraint. If there is no more work ready, release the sim-time constraint again by announcing infinite safe time.
      rclcpp::AnyExecutable next_executable;
      // get_next_executable(next_executable, 0) drains work already ready in the wait set without blocking.
      if (!get_next_executable(next_executable, std::chrono::nanoseconds(0))) {
        // No immediately-ready work remains, so release this worker's sim-time constraint.
        break;
        // The speed_regulator in sim_time_broker will push one time step forward if all thread_time_participants have released their safe-time constraints. So the step of the speed_regulator determines the sim time step in this case, and a small step is preferred and more accurated.
      }
      any_executable = next_executable;
    }
  }
}

}  // namespace executors

void spin(const rclcpp::Node::SharedPtr & node_ptr)
{
  rclcpp::ExecutorOptions options;
  options.context = node_ptr->get_node_base_interface()->get_context();
  fss_time::executors::SingleThreadedExecutor exec(options);
  exec.add_node(node_ptr);
  exec.spin();
  exec.remove_node(node_ptr);
}

}  // namespace fss_time
