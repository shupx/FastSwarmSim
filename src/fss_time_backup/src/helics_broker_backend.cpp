#include "fss_time/helics_broker_backend.hpp"

#include <sstream>
#include <stdexcept>

#include <helics/helics.h>

namespace fss_time
{

namespace
{

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

}  // namespace

struct HelicsBrokerBackend::Impl
{
  HelicsBroker broker{nullptr};
};

HelicsBrokerBackend::HelicsBrokerBackend(HelicsBrokerOptions options)
: options_(std::move(options)), impl_(std::make_unique<Impl>())
{
}

HelicsBrokerBackend::~HelicsBrokerBackend()
{
  finalize();
}

void HelicsBrokerBackend::start()
{
  if (started_ || !options_.start_broker) {
    started_ = true;
    return;
  }

  HelicsError error = helicsErrorInitialize();
  std::ostringstream init;
  init << "--port=" << options_.broker_port;
  if (options_.federates > 0) {
    init << " --federates=" << options_.federates;
  }

  impl_->broker = helicsCreateBroker(options_.core_type.c_str(), "fss_time_broker", init.str().c_str(), &error);
  throw_on_helics_error(error, "failed to create HELICS broker");
  started_ = true;
}

void HelicsBrokerBackend::finalize()
{
  if (impl_ == nullptr || !started_) {
    return;
  }

  if (impl_->broker != nullptr) {
    HelicsError error = helicsErrorInitialize();
    helicsBrokerDisconnect(impl_->broker, &error);
    helicsBrokerFree(impl_->broker);
    impl_->broker = nullptr;
  }
  started_ = false;
}

bool HelicsBrokerBackend::running_local_broker() const
{
  return options_.start_broker;
}

}  // namespace fss_time
