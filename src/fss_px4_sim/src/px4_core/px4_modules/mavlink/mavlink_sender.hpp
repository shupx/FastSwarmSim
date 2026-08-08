#pragma once

#include <functional>
#include <utility>

#include <mavlink/v2.0/common/mavlink.h>

using MavlinkSender = std::function<void(const mavlink_message_t &)>;
