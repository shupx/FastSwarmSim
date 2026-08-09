/**
 * Copied from px4_sitl_default build.
 * Modified by Peixuan Shu
 * Store PX4 parameters for the simulator.
 * Modified by Peixuan Shu (2026-08-09): replace agent-indexed global parameter
 * vectors with an instance-local ParameterStore.
 */

#include "px4_parameters.hpp"

#include <cstring>
#include <stdexcept>

#include <px4_platform_common/defines.h>

namespace px4 { 

namespace
{
// Added by Peixuan Shu: construction-time parameter store binding.
thread_local ParameterStore *current_parameter_store = nullptr;
}

ParameterStore::Scope::Scope(ParameterStore &store)
	: previous_(current_parameter_store)
{
	current_parameter_store = &store;
}

ParameterStore::Scope::~Scope()
{
	current_parameter_store = previous_;
}

ParameterStore::ParameterStore()
	: values_(parameters, parameters + sizeof(parameters) / sizeof(parameters[0]))
{
}

ParameterStore &ParameterStore::current()
{
	if (current_parameter_store == nullptr) {
		throw std::logic_error("construct PX4 parameter users inside a px4::ParameterStore::Scope");
	}
	return *current_parameter_store;
}

int ParameterStore::get(param_t param, void *value) const
{
	if (value == nullptr || param >= values_.size()) {
		return PX4_ERROR;
	}

	std::lock_guard<std::mutex> lock(mutex_);
	switch (parameters_type[param]) {
	case PARAM_TYPE_INT32:
		memcpy(value, &values_[param].val.i, sizeof(values_[param].val.i));
		return PX4_OK;
	case PARAM_TYPE_FLOAT:
		memcpy(value, &values_[param].val.f, sizeof(values_[param].val.f));
		return PX4_OK;
	default:
		return PX4_ERROR;
	}
}

int ParameterStore::set(param_t param, const void *value)
{
	if (value == nullptr || param >= values_.size()) {
		return PX4_ERROR;
	}

	std::lock_guard<std::mutex> lock(mutex_);
	switch (parameters_type[param]) {
	case PARAM_TYPE_INT32:
		memcpy(&values_[param].val.i, value, sizeof(values_[param].val.i));
		return PX4_OK;
	case PARAM_TYPE_FLOAT:
		memcpy(&values_[param].val.f, value, sizeof(values_[param].val.f));
		return PX4_OK;
	default:
		return PX4_ERROR;
	}
}

int ParameterStore::reset(param_t param)
{
	if (param >= values_.size()) {
		return PX4_ERROR;
	}

	std::lock_guard<std::mutex> lock(mutex_);
	values_[param] = parameters[param];
	return PX4_OK;
}

} // namespace px4
