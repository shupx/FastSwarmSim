/**
 * @file px4_instance_context.hpp
 * @brief Instance-local runtime state for one simulated PX4 flight stack.
 * Added by Peixuan Shu (2026-08-09).
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <parameters/px4_parameters.hpp>
#include <uORB/uORB_sim.hpp>

namespace MavrosQuadSimulator
{

// Added by Peixuan Shu: keep all formerly process-global PX4 simulation state
// together so a PX4SITL instance can own its complete runtime namespace.
struct Px4InstanceContext {
    class Scope {
    public:
        explicit Scope(Px4InstanceContext &context)
            : uorb_scope_(context.uorb_domain),
              clock_scope_(context.clock),
              parameter_scope_(context.parameter_store)
        {
        }

    private:
        uORB_sim::Domain::Scope uorb_scope_;
        px4::SimClock::Scope clock_scope_;
        px4::ParameterStore::Scope parameter_scope_;
    };

    uORB_sim::Domain uorb_domain;
    px4::SimClock clock;
    px4::ParameterStore parameter_store;
};

} // namespace MavrosQuadSimulator
