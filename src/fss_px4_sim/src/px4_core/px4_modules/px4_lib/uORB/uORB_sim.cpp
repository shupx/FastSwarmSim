/**
 * @file px4_uorb_lists.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Store PX4 uORB messages for the simulator.
 *
 * Note: This program relies on px4_lib/uORB/.
 * Modified by Peixuan Shu (2026-08-09): implement the instance-local uORB
 * domain binding used by Publication and Subscription handles.
 *
 * @version 1.1
 * @date 2026-08-09
 *
 * @license BSD 3-Clause License
 * @copyright (c) 2023-2026, Peixuan Shu
 * All rights reserved.
 */

#include "uORB_sim.hpp"

namespace uORB_sim
{
namespace
{
// Added by Peixuan Shu: construction-time domain binding for PX4-style wrappers.
thread_local Domain * current_domain = nullptr;
}

Domain::Scope::Scope(Domain & domain)
: previous_(current_domain)
{
  current_domain = &domain;
}

Domain::Scope::~Scope()
{
  current_domain = previous_;
}

Domain & Domain::current()
{
  if (current_domain == nullptr) {
    throw std::logic_error("construct uORB handles inside a uORB_sim::Domain::Scope or pass a Domain explicitly");
  }
  return *current_domain;
}

}  // namespace uORB_sim
