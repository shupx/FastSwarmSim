/**
 * Copied from PX4-Autopilot/platforms/common/include/px4_platform_common/param.h
 * Modified by Peixuan Shu (2026-08-09): replace agent-indexed global
 * parameter storage with an instance-local ParameterStore.
 * @author Peixuan Shu
 */

/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file param.h
 *
 * C++ Parameter class
 */

#pragma once

#include "param_macros.h"
#include <float.h>
#include <math.h>
#include <parameters/px4_parameters.hpp>

#include <px4_platform_common/defines.h> // added by Peixuan Shu 


/************* Added by Peixuan Shu **********/
#ifndef PX4_WARN
#include <iostream> // added by Peixuan Shu
#define PX4_WARN(x) std::cout << #x << std::endl
#endif

#ifndef PX4_ERR
#include <iostream> // added by Peixuan Shu
#define PX4_ERR(x) std::cout << #x << std::endl
#endif
/********************************************/

/******************************************************************************
 * /***************************************************************************
 * /***************************************************************************
 * Redefine some definitions to substitude for those in <parameters/param.h> 
 * and <parameters/param.cpp>
 * Added by Peixuan Shu
 * /

/**
 * Mark a parameter as used. Only marked parameters will be sent to a GCS.
 * A call to param_find() will mark a param as used as well.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 */
static inline void		param_set_used(param_t param) { /* Do nothing */ }

/**
 * Set the value of a parameter.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @param val		The value to set; assumed to point to a variable of the parameter type.
 * @return		Zero if the parameter's value could be set from a scalar, nonzero otherwise.
 */
static inline int		param_set(param_t param, const void *val){ return PX4_OK; /* Do nothing */ }

/**
 * Set the value of a parameter, but do not notify the system about the change.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @param val		The value to set; assumed to point to a variable of the parameter type.
 * @return		Zero if the parameter's value could be set from a scalar, nonzero otherwise.
 */
static inline int		param_set_no_notification(param_t param, const void *val){ return PX4_OK; /* Do nothing */ }

/**
 * Reset a parameter to its default value, but do not notify the system about the change.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @return		Zero on success, nonzero on failure
 */
static inline int		param_reset_no_notification(param_t param){ return 0; /* Do nothing */ }



static constexpr uint16_t param_info_count = sizeof(px4::parameters) / sizeof(param_info_s);

/**
 * Test whether a param_t is value.
 *
 * @param param			The parameter handle to test.
 * @return			True if the handle is valid.
 */
static constexpr bool handle_in_range(param_t param) { return (param < param_info_count); }

static inline param_type_t param_type(param_t param)
{
	return handle_in_range(param) ? px4::parameters_type[param] : PARAM_TYPE_UNKNOWN;
}

/**
 * Copy the value of a parameter. 
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @param val		Where to return the value, assumed to point to suitable storage for the parameter type.
 * @return		Zero if the parameter's value could be returned, nonzero otherwise.
 */
// Modified by Peixuan Shu: preserve the PX4-style helper while selecting the
// parameter store explicitly.
static inline int param_get(px4::ParameterStore &store, param_t param, void *val)
{
	if (!handle_in_range(param)) {
		return PX4_ERROR;
	}
	return store.get(param, val);
}

/**
 * Look up a parameter by name.
 *
 * @param name		The canonical name of the parameter being looked up.
 * @return		A handle to the parameter, or PARAM_INVALID if the parameter does not exist.
 *			This call will also set the parameter as "used" in the system, which is used
 *			to e.g. show the parameter via the RC interface
 */
static inline param_t	param_find(const char *name)
{
	param_t middle;
	param_t front = 0;
	param_t last = param_info_count;

	/* perform a binary search of the known parameters */

	while (front <= last) {
		middle = front + (last - front) / 2;
		int ret = strcmp(name, px4::parameters[middle].name);

		if (ret == 0) {
			// if (notification) {
			// 	param_set_used(middle);
			// }

			return middle;

		} else if (middle == front) {
			/* An end point has been hit, but there has been no match */
			break;

		} else if (ret < 0) {
			last = middle;

		} else {
			front = middle;
		}
	}

	/* not found */
	return PARAM_INVALID;
}

/**
 * Obtain the name of a parameter.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @return		The name assigned to the parameter, or NULL if the handle is invalid.
 */
static inline const char *param_name(param_t param)
{
	return handle_in_range(param) ? px4::parameters[param].name : nullptr;
}


#define CHECK_PARAM_TYPE(param, type) \
	if (param_type(param) != type) { \
		/* use printf() to avoid having to use more includes */ \
		PX4_ERR("wrong type passed to param_get() for param " + param_name(param)); \
	}
// #define CHECK_PARAM_TYPE(param, type)

// param is a C-interface. This means there is no overloading, and thus no type-safety for param_get().
// So for C++ code we redefine param_get() to inlined overloaded versions, which gives us type-safety
// w/o having to use a different interface

// static inline int param_get_cplusplus(param_t param, float *val)
static inline int param_get_cplusplus(px4::ParameterStore &store, param_t param, float *val)
{
	CHECK_PARAM_TYPE(param, PARAM_TYPE_FLOAT);
	return param_get(store, param, static_cast<void *>(val));
}
// static inline int param_get_cplusplus(param_t param, int32_t *val)
static inline int param_get_cplusplus(px4::ParameterStore &store, param_t param, int32_t *val)
{
	CHECK_PARAM_TYPE(param, PARAM_TYPE_INT32);
	return param_get(store, param, static_cast<void *>(val));
}
#undef CHECK_PARAM_TYPE

#define param_get(param, val) param_get_cplusplus(parameter_store_, param, val)

/*****************************************************************************
******************************************************************************
******************************************************************************
******************************************************************************/



/**
 * get the parameter handle from a parameter enum
 */
inline static param_t param_handle(px4::params p)
{
	return (param_t)p;
}




// Modified by Peixuan Shu: parameter wrappers bind their current local store.
#define _DEFINE_SINGLE_PARAMETER(x) do_not_explicitly_use_this_namespace::PAIR(x) {};

#define _CALL_UPDATE(x) \
	STRIP(x).update();

// define the parameter update method, which will update all parameters.
// It is marked as 'final', so that wrong usages lead to a compile error (see below)
#define _DEFINE_PARAMETER_UPDATE_METHOD(...) \
	protected: \
	void updateParamsImpl() final { \
		APPLY_ALL(_CALL_UPDATE, __VA_ARGS__) \
	} \
	private:

// Define a list of parameters. This macro also creates code to update parameters.
// If you get a compile error like:
//   error: virtual function ‘virtual void <class>::updateParamsImpl()’
// It means you have a custom inheritance tree (at least one class with params that inherits from another
// class with params) and you need to use DEFINE_PARAMETERS_CUSTOM_PARENT() for **all** classes in
// that tree.
#define DEFINE_PARAMETERS(...) \
	APPLY_ALL(_DEFINE_SINGLE_PARAMETER, __VA_ARGS__) \
	_DEFINE_PARAMETER_UPDATE_METHOD(__VA_ARGS__)


#define _DEFINE_PARAMETER_UPDATE_METHOD_CUSTOM_PARENT(parent_class, ...) \
	protected: \
	void updateParamsImpl() override { \
		parent_class::updateParamsImpl(); \
		APPLY_ALL(_CALL_UPDATE, __VA_ARGS__) \
	} \
	private:

#define DEFINE_PARAMETERS_CUSTOM_PARENT(parent_class, ...) \
	APPLY_ALL(_DEFINE_SINGLE_PARAMETER, __VA_ARGS__) \
	_DEFINE_PARAMETER_UPDATE_METHOD_CUSTOM_PARENT(parent_class, __VA_ARGS__)



// This namespace never needs to be used directly. Use the DEFINE_PARAMETERS_CUSTOM_PARENT and
// DEFINE_PARAMETERS macros instead (the Param classes don't depend on using the macro, the macro just
// makes sure that update() is automatically called).
namespace do_not_explicitly_use_this_namespace
{

template<typename T, px4::params p>
class Param
{
};

// We use partial template specialization for each param type. This is only supported for classes, not individual methods,
// which is why we have to repeat the whole class
template<px4::params p>
class Param<float, p>
{
public:
	// static type-check
	static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_FLOAT, "parameter type must be float");

	// Added by Peixuan Shu: capture the ParameterStore of this PX4 instance.
	Param() : parameter_store_(px4::ParameterStore::current())
	{
		param_set_used(handle());
		update();
	}

	// float get() const { return _val; }
	float get() 
	{ 
		update();
		return _val;
	}

	const float &reference() const { return _val; }

	/// Store the parameter value to the parameter storage (@see param_set())
	bool commit() const { return parameter_store_.set(handle(), &_val) == PX4_OK; }

	/// Store the parameter value to the parameter storage, w/o notifying the system (@see param_set_no_notification())
	bool commit_no_notification() const { return parameter_store_.set(handle(), &_val) == PX4_OK; }

	/// Set and commit a new value. Returns true if the value changed.
	bool commit_no_notification(float val)
	{
		if (fabsf(val - _val) > FLT_EPSILON) {
			set(val);
			commit_no_notification();
			return true;
		}

		return false;
	}

	void set(float val) { _val = val; }

	void reset()
	{
		parameter_store_.reset(handle());
		update();
	}

	bool update() { return parameter_store_.get(handle(), &_val) == PX4_OK; }

	param_t handle() const { return param_handle(p); }
private:
	px4::ParameterStore &parameter_store_;
	float _val;
};

// // external version
// template<px4::params p>
// class Param<float &, p>
// {
// public:
// 	// static type-check
// 	static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_FLOAT, "parameter type must be float");

// 	Param(float &external_val)
// 		: _val(external_val)
// 	{
// 		param_set_used(handle());
// 		update();
// 	}

// 	float get() const { return _val; }

// 	const float &reference() const { return _val; }

// 	/// Store the parameter value to the parameter storage (@see param_set())
// 	bool commit() const { return param_set(handle(), &_val) == 0; }

// 	/// Store the parameter value to the parameter storage, w/o notifying the system (@see param_set_no_notification())
// 	bool commit_no_notification() const { return param_set_no_notification(handle(), &_val) == 0; }

// 	/// Set and commit a new value. Returns true if the value changed.
// 	bool commit_no_notification(float val)
// 	{
// 		if (fabsf(val - _val) > FLT_EPSILON) {
// 			set(val);
// 			commit_no_notification();
// 			return true;
// 		}

// 		return false;
// 	}

// 	void set(float val) { _val = val; }

// 	void reset()
// 	{
// 		param_reset_no_notification(handle());
// 		update();
// 	}

// 	bool update() { return param_get(handle(), &_val) == 0; }

// 	param_t handle() const { return param_handle(p); }
// private:
// 	float &_val;
// };

template<px4::params p>
class Param<int32_t, p>
{
public:
	// static type-check
	static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_INT32, "parameter type must be int32_t");

	// Added by Peixuan Shu: capture the ParameterStore of this PX4 instance.
	Param() : parameter_store_(px4::ParameterStore::current())
	{
		param_set_used(handle());
		update();
	}

	// int32_t get() const { return _val; }
	int32_t get() 
	{ 
		update();
		return _val;
	}

	const int32_t &reference() const { return _val; }

	/// Store the parameter value to the parameter storage (@see param_set())
	bool commit() const { return parameter_store_.set(handle(), &_val) == PX4_OK; }

	/// Store the parameter value to the parameter storage, w/o notifying the system (@see param_set_no_notification())
	bool commit_no_notification() const { return parameter_store_.set(handle(), &_val) == PX4_OK; }

	/// Set and commit a new value. Returns true if the value changed.
	bool commit_no_notification(int32_t val)
	{
		if (val != _val) {
			set(val);
			commit_no_notification();
			return true;
		}

		return false;
	}

	void set(int32_t val) { _val = val; }

	void reset()
	{
		parameter_store_.reset(handle());
		update();
	}

	bool update() { return parameter_store_.get(handle(), &_val) == PX4_OK; }

	param_t handle() const { return param_handle(p); }
private:
	px4::ParameterStore &parameter_store_;
	int32_t _val;
};

// //external version
// template<px4::params p>
// class Param<int32_t &, p>
// {
// public:
// 	// static type-check
// 	static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_INT32, "parameter type must be int32_t");

// 	Param(int32_t &external_val)
// 		: _val(external_val)
// 	{
// 		param_set_used(handle());
// 		update();
// 	}

// 	int32_t get() const { return _val; }

// 	const int32_t &reference() const { return _val; }

// 	/// Store the parameter value to the parameter storage (@see param_set())
// 	bool commit() const { return param_set(handle(), &_val) == 0; }

// 	/// Store the parameter value to the parameter storage, w/o notifying the system (@see param_set_no_notification())
// 	bool commit_no_notification() const { return param_set_no_notification(handle(), &_val) == 0; }

// 	/// Set and commit a new value. Returns true if the value changed.
// 	bool commit_no_notification(int32_t val)
// 	{
// 		if (val != _val) {
// 			set(val);
// 			commit_no_notification();
// 			return true;
// 		}

// 		return false;
// 	}

// 	void set(int32_t val) { _val = val; }

// 	void reset()
// 	{
// 		param_reset_no_notification(handle());
// 		update();
// 	}

// 	bool update() { return param_get(handle(), &_val) == 0; }

// 	param_t handle() const { return param_handle(p); }
// private:
// 	int32_t &_val;
// };

template<px4::params p>
class Param<bool, p>
{
public:
	// static type-check
	static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_INT32, "parameter type must be int32_t");

	// Added by Peixuan Shu: capture the ParameterStore of this PX4 instance.
	Param() : parameter_store_(px4::ParameterStore::current())
	{
		param_set_used(handle());
		update();
	}

	// bool get() const { return _val; }
	bool get() 
	{ 
		update();
		return _val;
	}

	const bool &reference() const { return _val; }

	/// Store the parameter value to the parameter storage (@see param_set())
	bool commit() const
	{
		int32_t value_int = (int32_t)_val;
		return parameter_store_.set(handle(), &value_int) == PX4_OK;
	}

	/// Store the parameter value to the parameter storage, w/o notifying the system (@see param_set_no_notification())
	bool commit_no_notification() const
	{
		int32_t value_int = (int32_t)_val;
		return parameter_store_.set(handle(), &value_int) == PX4_OK;
	}

	/// Set and commit a new value. Returns true if the value changed.
	bool commit_no_notification(bool val)
	{
		if (val != _val) {
			set(val);
			commit_no_notification();
			return true;
		}

		return false;
	}

	void set(bool val) { _val = val; }

	void reset()
	{
		parameter_store_.reset(handle());
		update();
	}

	bool update()
	{
		int32_t value_int;
		int ret = parameter_store_.get(handle(), &value_int);

		if (ret == 0) {
			_val = value_int != 0;
			return true;
		}

		return false;
	}

	param_t handle() const { return param_handle(p); }
private:
	px4::ParameterStore &parameter_store_;
	bool _val;
};

template <px4::params p>
using ParamFloat = Param<float, p>;

template <px4::params p>
using ParamInt = Param<int32_t, p>;

// template <px4::params p>
// using ParamExtFloat = Param<float &, p>;

// template <px4::params p>
// using ParamExtInt = Param<int32_t &, p>;

template <px4::params p>
using ParamBool = Param<bool, p>;

} /* namespace do_not_explicitly_use_this_namespace */


// Raise an appropriate compile error if a Param class is used directly (just to simplify debugging)
template<px4::params p>
class ParamInt
{
	static_assert((int)p &&false, "Do not use this class directly, use the DEFINE_PARAMETERS macro instead");
};
template<px4::params p>
class ParamFloat
{
	static_assert((int)p &&false, "Do not use this class directly, use the DEFINE_PARAMETERS macro instead");
};
