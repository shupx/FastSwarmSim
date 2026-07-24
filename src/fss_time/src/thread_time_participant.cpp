#include "fss_time/thread_time_participant.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace fss_time
{

namespace
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

std::string sanitize_helics_name(std::string name)
{
  if (name.empty() || name == "/") {
    return "participant";
  }

  while (!name.empty() && name.front() == '/') {
    name.erase(name.begin());
  }
  for (auto & c : name) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' && c != '-') {
      c = '_';
    }
  }
  return name.empty() ? "participant" : name;
}

HelicsThreadParticipantOptions make_options(rclcpp::Node & node, const std::string & participant_id_hint)
{
  HelicsThreadParticipantOptions options;
  auto base_name = participant_id_hint;
  if (base_name.empty()) {
    base_name = std::string(node.get_namespace());
    if (base_name.empty() || base_name == "/") {
      base_name = node.get_name();
    }
  }
  options.participant_id = sanitize_helics_name(base_name + "_" + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())));
  options.core_type = declare_or_get_parameter<std::string>(node, "helics_core_type", options.core_type);
  options.broker_address = declare_or_get_parameter<std::string>(node, "helics_broker_address", options.broker_address);
  options.broker_port = declare_or_get_parameter<int>(node, "helics_broker_port", options.broker_port);
  options.time_delta_ns = declare_or_get_parameter<int64_t>(node, "helics_time_delta_ns", options.time_delta_ns);
  return options;
}

class ThreadParticipantRegistry
{
public:
  static ThreadParticipantRegistry & instance()
  {
    static ThreadParticipantRegistry registry;
    return registry;
  }

  void register_backend(const std::shared_ptr<HelicsThreadParticipantBackend> & backend)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      backends_.push_back(backend);
    }
    ensure_started();
  }

  uint32_t participant_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    uint32_t count = 0;
    for (const auto & weak_backend : backends_) {
      const auto backend = weak_backend.lock();
      if (backend && backend->count_for_participant_metrics()) {
        ++count;
      }
    }
    return count;
  }

  ~ThreadParticipantRegistry()
  {
    stop_ = true;
    if (poll_thread_.joinable()) {
      poll_thread_.join();
    }
  }

private:
  void ensure_started()
  {
    std::lock_guard<std::mutex> lock(start_mutex_);
    if (started_) {
      return;
    }

    poll_thread_ = std::thread([this]() {
      while (!stop_) {
        std::vector<std::shared_ptr<HelicsThreadParticipantBackend>> live_backends;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          auto it = backends_.begin();
          while (it != backends_.end()) {
            if (auto backend = it->lock()) {
              live_backends.push_back(std::move(backend));
              ++it;
            } else {
              it = backends_.erase(it);
            }
          }
        }

        for (const auto & backend : live_backends) {
          backend->poll();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    started_ = true;
  }

  mutable std::mutex mutex_;
  std::mutex start_mutex_;
  std::vector<std::weak_ptr<HelicsThreadParticipantBackend>> backends_;
  std::thread poll_thread_;
  bool started_{false};
  bool stop_{false};
};

thread_local std::unique_ptr<thread_time_participant> tls_participant;

}  // namespace

thread_time_participant::thread_time_participant(std::shared_ptr<HelicsThreadParticipantBackend> backend)
: backend_(std::move(backend))
{
  ThreadParticipantRegistry::instance().register_backend(backend_);
}

thread_time_participant & thread_time_participant::for_current_thread(
  rclcpp::Node & node,
  const std::string & participant_id_hint)
{
  if (!tls_participant) {
    tls_participant = std::unique_ptr<thread_time_participant>(
      new thread_time_participant(std::make_shared<HelicsThreadParticipantBackend>(make_options(node, participant_id_hint))));
  }
  return *tls_participant;
}

void thread_time_participant::reset_current_thread_for_testing()
{
  tls_participant.reset();
}

uint32_t thread_time_participant::participant_count()
{
  return ThreadParticipantRegistry::instance().participant_count();
}

void thread_time_participant::announce_next_safe_time(const rclcpp::Time & next_safe_time)
{
  backend_->announce_next_safe_time(next_safe_time.nanoseconds());
}

rclcpp::Time thread_time_participant::get_sim_time() const
{
  return rclcpp::Time(backend_->current_time_ns(), RCL_ROS_TIME);
}

}  // namespace fss_time
