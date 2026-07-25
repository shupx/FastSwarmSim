#ifndef FSS_TIME_HELICS_THREAD_PARTICIPANT_BACKEND_HPP_
#define FSS_TIME_HELICS_THREAD_PARTICIPANT_BACKEND_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace fss_time
{

struct HelicsThreadParticipantOptions
{
  std::string participant_id;
  std::string core_type{"zmq"};
  std::string broker_address{"127.0.0.1"};
  int broker_port{23404};
  int64_t time_delta_ns{1000000};
  bool count_for_participant_metrics{true};
};

class HelicsThreadParticipantBackend
{
public:
  explicit HelicsThreadParticipantBackend(HelicsThreadParticipantOptions options);
  ~HelicsThreadParticipantBackend();

  HelicsThreadParticipantBackend(const HelicsThreadParticipantBackend &) = delete;
  HelicsThreadParticipantBackend & operator=(const HelicsThreadParticipantBackend &) = delete;

  void start();
  void finalize();
  void announce_next_safe_time(int64_t next_safe_time_ns);
  bool poll();

  int64_t current_time_ns() const;
  int64_t last_requested_time_ns() const;
  bool is_request_in_flight() const;
  const std::string & participant_id() const;
  bool count_for_participant_metrics() const;

private:
  int64_t normalize_request_time_locked(int64_t requested_time_ns) const;
  struct Impl;

  void start_async_request_locked(int64_t request_time_ns);

  HelicsThreadParticipantOptions options_;
  std::unique_ptr<Impl> impl_;
  mutable std::mutex mutex_;
  int64_t current_time_ns_{0};
  int64_t last_requested_time_ns_{0};
  int64_t pending_request_time_ns_{0};
  bool started_{false};
  bool finalized_{false};
  bool request_in_flight_{false};
};

}  // namespace fss_time

#endif  // FSS_TIME_HELICS_THREAD_PARTICIPANT_BACKEND_HPP_
