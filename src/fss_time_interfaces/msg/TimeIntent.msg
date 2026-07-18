# Conservative distributed logical-time intent from one simulation participant.

uint8 STATE_ACTIVE=0
uint8 STATE_IDLE=1
uint8 STATE_PAUSED=2
uint8 STATE_LEAVING=3

string participant_id
uint64 epoch
builtin_interfaces/Time current_time
builtin_interfaces/Time next_safe_time
builtin_interfaces/Duration lookahead
uint8 state

# Sender steady-clock lease deadline expressed as nanoseconds since sender start.
# Receivers use their local receive time plus configured lease_timeout_ms for expiry.
uint64 lease_deadline_steady_ns
