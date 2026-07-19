#include "fss_time/time_transport_serialization.hpp"

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "fss_time/time_types.hpp"

namespace fss_time
{
namespace
{

constexpr const char * kIntentMagic = "FSS_TIME_INTENT_V1";
constexpr const char * kControlMagic = "FSS_TIME_CONTROL_V1";

std::vector<std::string> split_lines(const std::string & payload)
{
  std::vector<std::string> lines;
  std::stringstream stream(payload);
  std::string line;
  while (std::getline(stream, line, '\n')) {
    lines.push_back(line);
  }
  return lines;
}

bool parse_u64(const std::string & text, uint64_t & value)
{
  try {
    std::size_t parsed = 0;
    value = std::stoull(text, &parsed);
    return parsed == text.size();
  } catch (...) {
    return false;
  }
}

bool parse_i64(const std::string & text, int64_t & value)
{
  try {
    std::size_t parsed = 0;
    value = std::stoll(text, &parsed);
    return parsed == text.size();
  } catch (...) {
    return false;
  }
}

bool parse_f64(const std::string & text, double & value)
{
  try {
    std::size_t parsed = 0;
    value = std::stod(text, &parsed);
    return parsed == text.size();
  } catch (...) {
    return false;
  }
}

}  // namespace

std::string serialize_time_intent(const fss_time_interfaces::msg::TimeIntent & intent)
{
  std::ostringstream out;
  out << kIntentMagic << '\n'
      << intent.participant_id.size() << '\n'
      << intent.participant_id << '\n'
      << intent.epoch << '\n'
      << to_ns(intent.current_time) << '\n'
      << to_ns(intent.next_safe_time) << '\n'
      << duration_to_ns(intent.lookahead) << '\n'
      << static_cast<unsigned int>(intent.state) << '\n'
      << intent.lease_deadline_steady_ns << '\n';
  return out.str();
}

bool deserialize_time_intent(
  const void * data,
  std::size_t size,
  fss_time_interfaces::msg::TimeIntent & intent)
{
  if (data == nullptr || size == 0) {
    return false;
  }

  const std::string payload(static_cast<const char *>(data), size);
  const auto first_line_end = payload.find('\n');
  if (first_line_end == std::string::npos || payload.substr(0, first_line_end) != kIntentMagic) {
    return false;
  }

  const auto length_begin = first_line_end + 1;
  const auto length_end = payload.find('\n', length_begin);
  if (length_end == std::string::npos) {
    return false;
  }

  uint64_t participant_id_size = 0;
  if (!parse_u64(payload.substr(length_begin, length_end - length_begin), participant_id_size)) {
    return false;
  }

  const auto id_begin = length_end + 1;
  const auto id_end = id_begin + participant_id_size;
  if (id_end >= payload.size() || payload[id_end] != '\n') {
    return false;
  }

  intent.participant_id = payload.substr(id_begin, participant_id_size);
  const auto rest = split_lines(payload.substr(id_end + 1));
  if (rest.size() < 6) {
    return false;
  }

  uint64_t epoch = 0;
  int64_t current_time_ns = 0;
  int64_t next_safe_time_ns = 0;
  int64_t lookahead_ns = 0;
  uint64_t state = 0;
  uint64_t lease_deadline = 0;
  if (!parse_u64(rest[0], epoch) ||
    !parse_i64(rest[1], current_time_ns) ||
    !parse_i64(rest[2], next_safe_time_ns) ||
    !parse_i64(rest[3], lookahead_ns) ||
    !parse_u64(rest[4], state) ||
    !parse_u64(rest[5], lease_deadline))
  {
    return false;
  }

  intent.epoch = epoch;
  intent.current_time = from_ns(current_time_ns);
  intent.next_safe_time = from_ns(next_safe_time_ns);
  intent.lookahead = duration_from_ns(lookahead_ns);
  intent.state = static_cast<uint8_t>(state);
  intent.lease_deadline_steady_ns = lease_deadline;
  return true;
}

std::string serialize_time_control(const fss_time_interfaces::msg::TimeControl & control)
{
  std::ostringstream out;
  out << std::setprecision(17);
  out << kControlMagic << '\n'
      << control.epoch << '\n'
      << static_cast<unsigned int>(control.command) << '\n'
      << control.max_speed_ratio << '\n'
      << to_ns(control.reset_time) << '\n';
  return out.str();
}

bool deserialize_time_control(
  const void * data,
  std::size_t size,
  fss_time_interfaces::msg::TimeControl & control)
{
  if (data == nullptr || size == 0) {
    return false;
  }

  const std::string payload(static_cast<const char *>(data), size);
  const auto lines = split_lines(payload);
  if (lines.size() < 5 || lines[0] != kControlMagic) {
    return false;
  }

  uint64_t epoch = 0;
  uint64_t command = 0;
  double max_speed_ratio = 0.0;
  int64_t reset_time_ns = 0;
  if (!parse_u64(lines[1], epoch) ||
    !parse_u64(lines[2], command) ||
    !parse_f64(lines[3], max_speed_ratio) ||
    !parse_i64(lines[4], reset_time_ns))
  {
    return false;
  }

  control.epoch = epoch;
  control.command = static_cast<uint8_t>(command);
  control.max_speed_ratio = max_speed_ratio;
  control.reset_time = from_ns(reset_time_ns);
  return true;
}

}  // namespace fss_time
