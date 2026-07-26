#include "fss_time/helics_thread_participant_backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <helics/helics.h>

namespace fss_time
{

namespace
{

constexpr int64_t kInfiniteTimeNs = std::numeric_limits<int64_t>::max();
constexpr double kNsPerSecond = 1000000000.0;

double ns_to_helics_time(int64_t time_ns)
{
  if (time_ns >= kInfiniteTimeNs / 2) {
    return HELICS_TIME_MAXTIME;
  }
  return static_cast<double>(time_ns) / kNsPerSecond;
}

int64_t helics_time_to_ns(double time)
{
  if (time >= HELICS_TIME_MAXTIME / 2.0) {
    return kInfiniteTimeNs;
  }
  return static_cast<int64_t>(std::llround(time * kNsPerSecond));
}

int64_t ceil_to_step(int64_t value, int64_t step)
{
  if (step <= 1 || value <= 0) {
    return value;
  }
  const auto remainder = value % step;
  if (remainder == 0) {
    return value;
  }
  return value + (step - remainder);
}

void throw_on_helics_error(const HelicsError & error, const std::string & context)
{
  if (error.error_code == 0) {
    return;
  }
  std::ostringstream oss;
  oss << context;
  if (error.message != nullptr) {
    oss << ": " << error.message;
  }
  throw std::runtime_error(oss.str());
}

std::string broker_init_string(const HelicsThreadParticipantOptions & options)
{
  std::ostringstream oss;
  oss << "--broker_address=" << options.broker_address
      << " --brokerport=" << options.broker_port;
  return oss.str();
}

}  // namespace

struct HelicsThreadParticipantBackend::Impl
{
  HelicsFederateInfo federate_info{nullptr};
  HelicsFederate federate{nullptr};
};

HelicsThreadParticipantBackend::HelicsThreadParticipantBackend(HelicsThreadParticipantOptions options)
: options_(std::move(options)), impl_(std::make_unique<Impl>())
{
}

HelicsThreadParticipantBackend::~HelicsThreadParticipantBackend()
{
  finalize();
}

void HelicsThreadParticipantBackend::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (started_ || finalized_) {
    return;
  }

  HelicsError error = helicsErrorInitialize();
  impl_->federate_info = helicsCreateFederateInfo();
  helicsFederateInfoSetCoreTypeFromString(impl_->federate_info, options_.core_type.c_str(), &error);
  throw_on_helics_error(error, "failed to set HELICS core type");

  error = helicsErrorInitialize();
  const auto init = broker_init_string(options_);
  helicsFederateInfoSetCoreInitString(impl_->federate_info, init.c_str(), &error);
  throw_on_helics_error(error, "failed to set HELICS broker init string");

  error = helicsErrorInitialize();
  helicsFederateInfoSetTimeProperty(
    impl_->federate_info,
    HELICS_PROPERTY_TIME_DELTA,
    ns_to_helics_time(std::max<int64_t>(1, options_.time_delta_ns)),
    &error);
  throw_on_helics_error(error, "failed to set HELICS time delta");

  error = helicsErrorInitialize();
  impl_->federate = helicsCreateValueFederate(options_.participant_id.c_str(), impl_->federate_info, &error);
  throw_on_helics_error(error, "failed to create HELICS value federate");

  error = helicsErrorInitialize();
  helicsFederateEnterExecutingMode(impl_->federate, &error);
  throw_on_helics_error(error, "failed to enter HELICS executing mode");

  started_ = true;
}

void HelicsThreadParticipantBackend::finalize()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (finalized_ || impl_ == nullptr) {
    return;
  }

  HelicsError error = helicsErrorInitialize();
  if (impl_->federate != nullptr) {
    helicsFederateFinalize(impl_->federate, &error);
    helicsFederateFree(impl_->federate);
    impl_->federate = nullptr;
  }
  if (impl_->federate_info != nullptr) {
    helicsFederateInfoFree(impl_->federate_info);
    impl_->federate_info = nullptr;
  }
  request_in_flight_ = false;
  started_ = false;
  finalized_ = true;
}

void HelicsThreadParticipantBackend::announce_next_safe_time(int64_t next_safe_time_ns)
{
  start();

  std::lock_guard<std::mutex> lock(mutex_);
  if (finalized_) {
    return;
  }

  const auto effective_request_ns = normalize_request_time_locked(next_safe_time_ns);
  pending_request_time_ns_ = std::max(pending_request_time_ns_, effective_request_ns);
  if (!request_in_flight_) {
    start_async_request_locked(pending_request_time_ns_);
  }
}

bool HelicsThreadParticipantBackend::poll()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_ || finalized_ || impl_ == nullptr || impl_->federate == nullptr || !request_in_flight_) {
    return false;
  }

  HelicsError error = helicsErrorInitialize();
  const auto completed = helicsFederateIsAsyncOperationCompleted(impl_->federate, &error);
  throw_on_helics_error(error, "failed to check HELICS async status");
  if (completed == HELICS_FALSE) {
    return false;
  }

  error = helicsErrorInitialize();
  const auto granted_time = helicsFederateRequestTimeComplete(impl_->federate, &error);
  throw_on_helics_error(error, "failed to complete HELICS async time request");

  request_in_flight_ = false;
  const auto previous_time_ns = current_time_ns_;
  current_time_ns_ = std::max(current_time_ns_, helics_time_to_ns(granted_time));
  if (pending_request_time_ns_ > current_time_ns_) {
    start_async_request_locked(pending_request_time_ns_);
  }

  return current_time_ns_ > previous_time_ns;
}

std::string HelicsThreadParticipantBackend::query(const std::string & target, const std::string & query)
{
  start();

  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_ || finalized_ || impl_ == nullptr || impl_->federate == nullptr) {
    return {};
  }

  HelicsError error = helicsErrorInitialize();
  auto core = helicsFederateGetCore(impl_->federate, &error);
  throw_on_helics_error(error, "failed to get HELICS core for query");

  auto query_handle = helicsCreateQuery(target.c_str(), query.c_str());
  if (query_handle == nullptr) {
    throw std::runtime_error("failed to create HELICS query");
  }

  error = helicsErrorInitialize();
  const char * result = helicsQueryCoreExecute(query_handle, core, &error);
  helicsQueryFree(query_handle);
  throw_on_helics_error(error, "failed to execute HELICS query");
  return (result != nullptr) ? std::string(result) : std::string{};
}

int64_t HelicsThreadParticipantBackend::current_time_ns() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_time_ns_;
}

int64_t HelicsThreadParticipantBackend::last_requested_time_ns() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_requested_time_ns_;
}

bool HelicsThreadParticipantBackend::is_request_in_flight() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return request_in_flight_;
}

const std::string & HelicsThreadParticipantBackend::participant_id() const
{
  return options_.participant_id;
}

bool HelicsThreadParticipantBackend::count_for_participant_metrics() const
{
  return options_.count_for_participant_metrics;
}

int64_t HelicsThreadParticipantBackend::normalize_request_time_locked(int64_t requested_time_ns) const
{
  if (requested_time_ns >= kInfiniteTimeNs / 2) {
    return kInfiniteTimeNs;
  }

  const auto base_request_ns = std::max({requested_time_ns, current_time_ns_, last_requested_time_ns_});
  const auto step_ns = std::max<int64_t>(1, options_.time_delta_ns);
  const auto minimum_forward_ns =
    current_time_ns_ >= kInfiniteTimeNs / 2 ? current_time_ns_ : current_time_ns_ + step_ns;
  return ceil_to_step(std::max(base_request_ns, minimum_forward_ns), step_ns);
}

void HelicsThreadParticipantBackend::start_async_request_locked(int64_t request_time_ns)
{
  HelicsError error = helicsErrorInitialize();
  helicsFederateRequestTimeAsync(impl_->federate, ns_to_helics_time(request_time_ns), &error);
  throw_on_helics_error(error, "failed to start HELICS async time request");
  request_in_flight_ = true;
  last_requested_time_ns_ = request_time_ns;
}

}  // namespace fss_time
