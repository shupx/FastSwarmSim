/**
 * Copied from px4_sitl_default build.
 * Modified by Peixuan Shu
 * Store all global(extern) px4 parameters
 * Peixuan Shu
 */

#include "px4_parameters.hpp"

namespace px4 { 

std::vector<param_info_s>  parameters_default(parameters, parameters+sizeof(parameters)/sizeof(param_info_s)); // used as the default element of parameters_vectors // added by Peixuan Shu


// Store global(extern) px4 parameters of all UAVs (Created by Peixuan Shu)
// initialize with at least one element for UAV 1
std::vector<std::vector<param_info_s>> parameters_vectors{parameters_default};  // define (allocate storage)


/* allocate global storage for messages of agent i */
void allocate_px4_params_storage(int expected_agent_num)
{
    while(parameters_vectors.size() < expected_agent_num)
    {
        parameters_vectors.emplace_back(parameters_default);
    }
}

} // namespace px4