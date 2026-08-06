/**
 * @file mavlink_streamer.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Simulate the streaming of mavlink messages in PX4. 
 * 
 * Note: This program relies on mavlink
 * 
 * @version 1.0
 * @date 2023-12-17
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#pragma once

#include <iostream> // for std::cout, std::endl
#include <memory>  // for std::shared_ptr
#include <boost/signals2.hpp> // for signal2

#include "streams/ATTITUDE_QUATERNION.hpp"
#include "streams/ATTITUDE_TARGET.hpp"
#include "streams/HEARTBEAT.hpp"
#include "streams/LOCAL_POSITION_NED.hpp"
#include "streams/POSITION_TARGET_LOCAL_NED.hpp"
#include "streams/SYS_STATUS.hpp"
#include "streams/GLOBAL_POSITION_INT.hpp"
#include "streams/GPS_GLOBAL_ORIGIN.hpp"

#include "mavlink_msg_list.hpp"  // store the simulated static(global) mavlink messages


#define STREAM_PTR(...) std::shared_ptr<MavlinkStream<__VA_ARGS__> >
#define STREAM_MAKE_PTR(...) std::make_shared<MavlinkStream<__VA_ARGS__> >

class MavlinkStreamer
{
private:
    int agent_id_ = -1; // agent id.

public:
	typedef boost::signals2::signal<void(const uint64_t&)> VoidSignal;
	VoidSignal stream_signal_;

	template <typename T> 
	class MavlinkStream
	{
		public:
			MavlinkStream(const float& rate, MavlinkStreamer* parent) : period_us_(1.e6/rate) 
			{
				stream_.set_agent_id(parent->agent_id_);
				parent->stream_signal_.connect(boost::bind(&MavlinkStream::Stream, this, boost::placeholders::_1));
			}

			/* Streaming the mavlink messages at a given rate */
			void Stream(const uint64_t &time_us)
			{
				// static uint64_t last_send = 0;
				if (time_us >= time_last_send_ + period_us_)
				{
					// std::cout << "[MavlinkStream::Stream] time_us: " << time_us << std::endl;
					// std::cout << "[MavlinkStream::Stream] time_us - last_send = " << (time_us - time_last_send_) << " us" << std::endl;
					stream_.send();
					time_last_send_ = time_us < time_last_send_ + 2*period_us_ ? time_last_send_ + period_us_ : time_us;
				}
			}

		private:
			uint64_t time_last_send_ = 0; // us
			T stream_; 
			uint64_t period_us_; // us
	};

	STREAM_PTR(MavlinkStreamAttitudeQuaternion) mavlink_stream_AttitudeQuaternion_;
	STREAM_PTR(MavlinkStreamLocalPositionNED) mavlink_stream_LocalPositionNED_;
	STREAM_PTR(MavlinkStreamAttitudeTarget) mavlink_stream_AttitudeTarget_;
	STREAM_PTR(MavlinkStreamPositionTargetLocalNed) mavlink_stream_PositionTargetLocalNed_;
	STREAM_PTR(MavlinkStreamHeartbeat) mavlink_stream_Heartbeat_;
	STREAM_PTR(MavlinkStreamSysStatus) mavlink_stream_SysStatus_;
	STREAM_PTR(MavlinkStreamGlobalPositionInt) mavlink_stream_GlobalPositionInt_;
	STREAM_PTR(MavlinkStreamGpsGlobalOrigin) mavlink_stream_GpsGlobalOrigin_;

	MavlinkStreamer(int agent_id);

	~MavlinkStreamer();

	/**
	 * \brief stream all mavlink messages
	 * @param time_us time now (microseconds, us)
	 */
	void Stream(const uint64_t &time_us);
};


