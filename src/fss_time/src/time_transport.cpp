#include "fss_time/time_transport.hpp"

#include <stdexcept>

namespace fss_time
{

std::unique_ptr<TimeTransport> create_ecal_time_transport(
  const TimeTransportOptions & options);

std::unique_ptr<TimeTransport> create_time_transport(
  const std::string & transport_name,
  const TimeTransportOptions & options)
{
  if (transport_name == "ecal") {
    return create_ecal_time_transport(options);
  }

  throw std::runtime_error("Unsupported fss_time transport: " + transport_name + ". Only ecal is supported.");
}

}  // namespace fss_time
