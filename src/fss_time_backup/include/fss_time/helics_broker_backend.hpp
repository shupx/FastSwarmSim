#ifndef FSS_TIME_HELICS_BROKER_BACKEND_HPP_
#define FSS_TIME_HELICS_BROKER_BACKEND_HPP_

#include <memory>
#include <string>

namespace fss_time
{

struct HelicsBrokerOptions
{
  std::string core_type{"zmq"};
  std::string broker_address{"127.0.0.1"};
  int broker_port{23404};
  bool start_broker{true};
  int federates{0};
};

class HelicsBrokerBackend
{
public:
  explicit HelicsBrokerBackend(HelicsBrokerOptions options);
  ~HelicsBrokerBackend();

  HelicsBrokerBackend(const HelicsBrokerBackend &) = delete;
  HelicsBrokerBackend & operator=(const HelicsBrokerBackend &) = delete;

  void start();
  void finalize();
  bool running_local_broker() const;

private:
  struct Impl;

  HelicsBrokerOptions options_;
  std::unique_ptr<Impl> impl_;
  bool started_{false};
};

}  // namespace fss_time

#endif  // FSS_TIME_HELICS_BROKER_BACKEND_HPP_
