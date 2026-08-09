/**
 * @file px4_uorb_lists.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Store PX4 uORB messages for the simulator.
 *
 * Note: This program relies on px4_lib/uORB/.
 * Modified by Peixuan Shu (2026-08-09): replace agent-indexed global message
 * vectors with instance-local uORB domains and PX4-compatible topic metadata.
 *
 * @version 1.1
 * @date 2026-08-09
 *
 * @license BSD 3-Clause License
 * @copyright (c) 2023-2026, Peixuan Shu
 * All rights reserved.
 */

#pragma once

#include <px4_platform_common/defines.h>

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/commander_state.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <unordered_map>

namespace uORB_sim
{

// Added by Peixuan Shu: static topic metadata used by the instance-local domain.
enum class TopicId : uint16_t {
  actuator_armed,
  autotune_attitude_control_status,
  battery_status,
  commander_state,
  cpuload,
  home_position,
  manual_control_setpoint,
  offboard_control_mode,
  takeoff_status,
  vehicle_air_data,
  vehicle_angular_velocity,
  vehicle_attitude,
  vehicle_attitude_setpoint,
  vehicle_command,
  vehicle_command_ack,
  vehicle_constraints,
  vehicle_control_mode,
  vehicle_global_position,
  vehicle_gps_position,
  vehicle_land_detected,
  vehicle_local_position,
  vehicle_local_position_setpoint,
  trajectory_setpoint,
  vehicle_odometry,
  vehicle_rates_setpoint,
  vehicle_status,
  vehicle_status_flags,
};

struct orb_metadata {
  TopicId id;
  const char * name;
  std::size_t size;
};

#define UORB_SIM_TOPIC_LIST(X) \
  X(actuator_armed, actuator_armed_s) \
  X(autotune_attitude_control_status, autotune_attitude_control_status_s) \
  X(battery_status, battery_status_s) \
  X(commander_state, commander_state_s) \
  X(cpuload, cpuload_s) \
  X(home_position, home_position_s) \
  X(manual_control_setpoint, manual_control_setpoint_s) \
  X(offboard_control_mode, offboard_control_mode_s) \
  X(takeoff_status, takeoff_status_s) \
  X(vehicle_air_data, vehicle_air_data_s) \
  X(vehicle_angular_velocity, vehicle_angular_velocity_s) \
  X(vehicle_attitude, vehicle_attitude_s) \
  X(vehicle_attitude_setpoint, vehicle_attitude_setpoint_s) \
  X(vehicle_command, vehicle_command_s) \
  X(vehicle_command_ack, vehicle_command_ack_s) \
  X(vehicle_constraints, vehicle_constraints_s) \
  X(vehicle_control_mode, vehicle_control_mode_s) \
  X(vehicle_global_position, vehicle_global_position_s) \
  X(vehicle_gps_position, vehicle_gps_position_s) \
  X(vehicle_land_detected, vehicle_land_detected_s) \
  X(vehicle_local_position, vehicle_local_position_s) \
  X(vehicle_local_position_setpoint, vehicle_local_position_setpoint_s) \
  X(trajectory_setpoint, vehicle_local_position_setpoint_s) \
  X(vehicle_odometry, vehicle_odometry_s) \
  X(vehicle_rates_setpoint, vehicle_rates_setpoint_s) \
  X(vehicle_status, vehicle_status_s) \
  X(vehicle_status_flags, vehicle_status_flags_s)

#define UORB_SIM_DEFINE_METADATA(name, type) \
  inline const orb_metadata metadata_##name{TopicId::name, #name, sizeof(type)};
UORB_SIM_TOPIC_LIST(UORB_SIM_DEFINE_METADATA)
#undef UORB_SIM_DEFINE_METADATA

class TopicNodeBase
{
public:
  virtual ~TopicNodeBase() = default;
  virtual std::size_t message_size() const = 0;
};

template<typename T>
class TopicNode final : public TopicNodeBase
{
public:
  std::size_t message_size() const override { return sizeof(T); }

  mutable std::mutex mutex;
  T data{};
  uint64_t generation{0};
  bool advertised{false};
};

/** One isolated uORB namespace, intended to be owned by one PX4 instance. Added by Peixuan Shu. */
class Domain
{
public:
  class Scope
  {
  public:
    explicit Scope(Domain & domain);
    ~Scope();

    Scope(const Scope &) = delete;
    Scope & operator=(const Scope &) = delete;

  private:
    Domain * previous_;
  };

  Domain() = default;
  Domain(const Domain &) = delete;
  Domain & operator=(const Domain &) = delete;

  static Domain & current();

  template<typename T>
  std::shared_ptr<TopicNode<T>> topic(const orb_metadata * metadata, uint8_t instance = 0)
  {
    if (metadata == nullptr || metadata->size != sizeof(T)) {
      throw std::invalid_argument("uORB topic metadata does not match message type");
    }

    const Key key{metadata, instance};
    std::lock_guard<std::mutex> lock(nodes_mutex_);
    const auto it = nodes_.find(key);

    if (it == nodes_.end()) {
      auto node = std::make_shared<TopicNode<T>>();
      nodes_.emplace(key, node);
      return node;
    }

    auto node = std::dynamic_pointer_cast<TopicNode<T>>(it->second);
    if (!node) {
      throw std::logic_error("uORB topic has inconsistent message type");
    }
    return node;
  }

private:
  struct Key {
    const orb_metadata * metadata;
    uint8_t instance;

    bool operator==(const Key & other) const
    {
      return metadata == other.metadata && instance == other.instance;
    }
  };

  struct KeyHash {
    std::size_t operator()(const Key & key) const
    {
      return std::hash<const orb_metadata *>{}(key.metadata) ^
             (static_cast<std::size_t>(key.instance) << 1U);
    }
  };

  std::mutex nodes_mutex_;
  std::unordered_map<Key, std::shared_ptr<TopicNodeBase>, KeyHash> nodes_;
};

template<typename T>
class Subscription
{
public:
  explicit Subscription(const orb_metadata * metadata, uint8_t instance = 0)
  : Subscription(Domain::current(), metadata, instance) {}

  Subscription(Domain & domain, const orb_metadata * metadata, uint8_t instance = 0)
  : node_(domain.topic<T>(metadata, instance))
  {
    std::lock_guard<std::mutex> lock(node_->mutex);
    last_generation_ = node_->generation;
  }

  bool valid() const { return static_cast<bool>(node_); }

  bool copy(void * dst)
  {
    if (dst == nullptr || !node_) {
      return false;
    }

    std::lock_guard<std::mutex> lock(node_->mutex);
    if (!node_->advertised) {
      return false;
    }
    *static_cast<T *>(dst) = node_->data;
    last_generation_ = node_->generation;
    return true;
  }

  bool updated() const
  {
    if (!node_) {
      return false;
    }
    std::lock_guard<std::mutex> lock(node_->mutex);
    return node_->advertised && node_->generation != last_generation_;
  }

  bool update(void * dst)
  {
    return updated() && copy(dst);
  }

private:
  std::shared_ptr<TopicNode<T>> node_;
  uint64_t last_generation_{0};
};

template<class T>
class SubscriptionData : public Subscription<T>
{
public:
  explicit SubscriptionData(const orb_metadata * metadata, uint8_t instance = 0)
  : Subscription<T>(metadata, instance) {}

  SubscriptionData(Domain & domain, const orb_metadata * metadata, uint8_t instance = 0)
  : Subscription<T>(domain, metadata, instance) {}

  bool update() { return Subscription<T>::update(&_data); }
  const T & get() const { return _data; }

private:
  T _data{};
};

template<typename T>
class Publication
{
public:
  explicit Publication(const orb_metadata * metadata, uint8_t instance = 0)
  : Publication(Domain::current(), metadata, instance) {}

  Publication(Domain & domain, const orb_metadata * metadata, uint8_t instance = 0)
  : node_(domain.topic<T>(metadata, instance)) {}

  bool advertised() const
  {
    std::lock_guard<std::mutex> lock(node_->mutex);
    return node_->advertised;
  }

  bool advertise()
  {
    std::lock_guard<std::mutex> lock(node_->mutex);
    node_->advertised = true;
    return true;
  }

  bool publish(const T & data)
  {
    std::lock_guard<std::mutex> lock(node_->mutex);
    node_->data = data;
    node_->advertised = true;
    ++node_->generation;
    return true;
  }

private:
  std::shared_ptr<TopicNode<T>> node_;
};

template<typename T>
class PublicationData : public Publication<T>
{
public:
  explicit PublicationData(const orb_metadata * metadata, uint8_t instance = 0)
  : Publication<T>(metadata, instance) {}

  PublicationData(Domain & domain, const orb_metadata * metadata, uint8_t instance = 0)
  : Publication<T>(domain, metadata, instance) {}

  T & get() { return data_; }
  void set(const T & data) { data_ = data; }
  bool update() { return Publication<T>::publish(data_); }
  bool update(const T & data)
  {
    data_ = data;
    return Publication<T>::publish(data_);
  }

private:
  T data_{};
};

}  // namespace uORB_sim

// Added by Peixuan Shu: preserve the namespace used by PX4 C++ uORB wrappers.
namespace uORB
{
using Domain = uORB_sim::Domain;
using orb_metadata = uORB_sim::orb_metadata;

template<typename T>
using Subscription = uORB_sim::Subscription<T>;
template<typename T>
using SubscriptionData = uORB_sim::SubscriptionData<T>;
template<typename T>
using Publication = uORB_sim::Publication<T>;
template<typename T>
using PublicationData = uORB_sim::PublicationData<T>;
}  // namespace uORB

#ifdef ORB_ID
#error "uORB_sim.hpp must be included before another uORB implementation"
#endif
// Modified by Peixuan Shu: ORB_ID identifies only a topic, never an agent.
#define ORB_ID(x) (&uORB_sim::metadata_##x)

#define ORB_ID_VEHICLE_ATTITUDE_CONTROLS ORB_ID(actuator_controls_0)
typedef uint8_t arming_state_t;
typedef uint8_t main_state_t;
typedef uint8_t hil_state_t;
typedef uint8_t navigation_state_t;
typedef uint8_t switch_pos_t;
