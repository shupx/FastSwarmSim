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
: rclcpp::Executor(options)
{
}

SingleThreadedExecutor::~SingleThreadedExecutor() {}

void
SingleThreadedExecutor::set_time_node(const rclcpp::Node::SharedPtr & time_node)
{
  time_node_ = time_node;
  use_fss_sim_time_ =
    fss_time_tools::declare_or_get_parameter<bool>(*time_node_, "use_fss_sim_time", false);
  if (use_fss_sim_time_) {
    fss_time_tools::ensure_use_sim_time_enabled(*time_node_);
  }
}

void
SingleThreadedExecutor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  set_time_node(node_ptr);
  rclcpp::Executor::add_node(node_ptr, notify);
}

void
SingleThreadedExecutor::remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  rclcpp::Executor::remove_node(node_ptr, notify);
  if (time_node_ == node_ptr) {
    time_node_.reset();
    use_fss_sim_time_ = false;
  }
}

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
    if (!get_next_executable(any_executable)) {
      continue;
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
        fss_time_tools::announce_next_safe_time_infinite(participant);
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
