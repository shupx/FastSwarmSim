#include "fss_time/helics_time_coordinator.hpp"

#include <algorithm>
#include <cmath>
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
  if (time_ns == kInfiniteTimeNs) {
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

std::string broker_init_string(const HelicsTimeOptions & options)
{
  std::ostringstream oss;
  oss << "--broker_address=" << options.broker_address
      << " --brokerport=" << options.broker_port;
  return oss.str();
}

}  // namespace

struct HelicsTimeCoordinator::Impl
{
  HelicsFederateInfo federate_info{nullptr};
  HelicsFederate federate{nullptr};
};

HelicsTimeCoordinator::HelicsTimeCoordinator(HelicsTimeOptions options)
: options_(std::move(options)),
  impl_(std::make_unique<Impl>()),
  next_safe_time_ns_(kInfiniteTimeNs),
  max_speed_ratio_(options_.max_speed_ratio)
{
}

HelicsTimeCoordinator::~HelicsTimeCoordinator()
{
  finalize();
}

void HelicsTimeCoordinator::start()
{
  if (started_) {
    return;
  }

  HelicsError error = helicsErrorInitialize();
  impl_->federate_info = helicsCreateFederateInfo();
  helicsFederateInfoSetCoreTypeFromString(
    impl_->federate_info, options_.core_type.c_str(), &error);
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
  throw_on_helics_error(error, "failed to set HELICS time_delta");

  error = helicsErrorInitialize();
  impl_->federate = helicsCreateValueFederate(
    options_.participant_id.c_str(), impl_->federate_info, &error);
  throw_on_helics_error(error, "failed to create HELICS value federate");

  error = helicsErrorInitialize();
  helicsFederateEnterExecutingMode(impl_->federate, &error);
  throw_on_helics_error(error, "failed to enter HELICS executing mode");

  started_ = true;
}

void HelicsTimeCoordinator::finalize()
{
  if (impl_ == nullptr) {
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
  started_ = false;
}

void HelicsTimeCoordinator::set_next_safe_time(int64_t next_safe_time_ns)
{
  next_safe_time_ns_ = std::max(next_safe_time_ns, current_time_ns_);
  active_ = true;
}

void HelicsTimeCoordinator::set_idle()
{
  next_safe_time_ns_ = kInfiniteTimeNs;
  active_ = false;
}

void HelicsTimeCoordinator::set_speed(double max_speed_ratio, int64_t local_steady_now_ns)
{
  sim_start_ns_ = current_time_ns_;
  wall_start_steady_ns_ = local_steady_now_ns;
  max_speed_ratio_ = max_speed_ratio;
}

void HelicsTimeCoordinator::set_paused(bool paused)
{
  paused_ = paused;
}

void HelicsTimeCoordinator::reset(int64_t sim_time_ns, int64_t local_steady_now_ns)
{
  const auto monotonic_reset_ns = std::max(sim_time_ns, current_time_ns_);
  sim_start_ns_ = monotonic_reset_ns;
  wall_start_steady_ns_ = local_steady_now_ns;
  current_time_ns_ = monotonic_reset_ns;
  next_safe_time_ns_ = monotonic_reset_ns;
  active_ = true;
}

HelicsGrantResult HelicsTimeCoordinator::request_grant(int64_t local_steady_now_ns)
{
  HelicsGrantResult result;
  result.paused = paused_;
  result.grant_time_ns = current_time_ns_;

  if (!started_ || impl_->federate == nullptr || paused_) {
    return result;
  }

  const auto request_time_ns = compute_request_time_ns(local_steady_now_ns);
  if (request_time_ns <= current_time_ns_) {
    return result;
  }

  HelicsError error = helicsErrorInitialize();
  const auto granted_time = helicsFederateRequestTime(
    impl_->federate, ns_to_helics_time(request_time_ns), &error);
  throw_on_helics_error(error, "HELICS time request failed");

  const auto granted_time_ns = helics_time_to_ns(granted_time);
  if (granted_time_ns > current_time_ns_ && granted_time_ns != kInfiniteTimeNs) {
    current_time_ns_ = granted_time_ns;
    result.advanced = true;
  }
  result.grant_time_ns = current_time_ns_;
  return result;
}

int64_t HelicsTimeCoordinator::speed_cap_ns(int64_t local_steady_now_ns) const
{
  if (max_speed_ratio_ <= 0.0) {
    return kInfiniteTimeNs;
  }

  const auto elapsed_ns = std::max<int64_t>(0, local_steady_now_ns - wall_start_steady_ns_);
  return sim_start_ns_ + static_cast<int64_t>(elapsed_ns * max_speed_ratio_);
}

int64_t HelicsTimeCoordinator::compute_request_time_ns(int64_t local_steady_now_ns) const
{
  auto request_time_ns = active_ ? next_safe_time_ns_ : kInfiniteTimeNs;
  request_time_ns = std::min(request_time_ns, speed_cap_ns(local_steady_now_ns));
  if (request_time_ns == kInfiniteTimeNs) {
    return current_time_ns_;
  }
  return std::max(request_time_ns, current_time_ns_);
}

}  // namespace fss_time
