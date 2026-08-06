#ifndef FSS_TIME_THREAD_TIME_PARTICIPANT_HPP_
#define FSS_TIME_THREAD_TIME_PARTICIPANT_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "fss_time/zeromq_time_participant_backend.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fss_time
{

class thread_time_participant
{
public:
  /**
   * @brief Get or create the fss_time participant associated with this thread.
   * @param node Node used for participant options, clock access, and coordinator endpoint parameters.
   * @param participant_id_hint Optional stable prefix used when generating the participant id.
   * @param follows_real_time Whether this participant should follow real time.
   * @return Thread-local participant instance.
   */
  static thread_time_participant & for_current_thread(
    rclcpp::Node & node,
    const std::string & participant_id_hint = "",
    bool follows_real_time = true);
#ifdef BUILD_TESTING
  /**
   * @brief Reset the current thread's participant for tests.
   */
  static void reset_current_thread_for_testing();
#endif

  /**
   * @brief Unregister this participant from the coordinator.
   */
  ~thread_time_participant();

  /**
   * @brief Announce the next simulation time this thread can safely allow. It will register the participant with the coordinator if not already registered.
   * @param next_safe_time Safe time request in ROS time.
   */
  void announce_next_safe_time(const rclcpp::Time & next_safe_time);

  /**
   * @brief Unregister the participant from the coordinator.
   */
  void unregister_participant();

  /**
   * @brief Set whether this thread's participant follows real time. 
   * The default is true, which means the coordinator will apply a real-time
   *  floor to the simulation time even if some participants do not 
   * finish their work in real time, like what happens in a real-time system. 
   * Setting this to false will allow the simulation to run slower than 
   * real time if necessary, and make sure every participant 
   * finishes its work before the next simulation time step is allowed,
   * which is necessary for stable simulation of physics and control systems. 
   * @param follows_real_time true to apply the coordinator real-time floor.
   */
  void set_follows_real_time(bool follows_real_time);

  /**
   * @brief Return the latest coordinator simulation time observed by this participant.
   * @return Current coordinator simulation time as RCL_ROS_TIME.
   */
  rclcpp::Time get_sim_time() const;

  /**
   * @brief Return the last effective safe time announced by this participant.
   * @return Last safe time as RCL_ROS_TIME.
   */
  rclcpp::Time get_last_safe_time() const;

private:
  explicit thread_time_participant(std::shared_ptr<ZeroMqTimeParticipantBackend> backend);

  std::shared_ptr<ZeroMqTimeParticipantBackend> backend_;
  mutable std::mutex mutex_;
  int64_t last_safe_time_ns_{0};
};

}  // namespace fss_time

#endif  // FSS_TIME_THREAD_TIME_PARTICIPANT_HPP_
