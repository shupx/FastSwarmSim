# Distributed clock control command. Higher epoch wins.

uint8 COMMAND_RESUME=0
uint8 COMMAND_PAUSE=1
uint8 COMMAND_SET_SPEED=2
uint8 COMMAND_RESET=3

uint64 epoch
uint8 command

# 0.0 means as-fast-as-safe. Positive values cap sim-time speed against steady wall time.
float64 max_speed_ratio
builtin_interfaces/Time reset_time
