# Result of the publish gate. A rejected report must never be sent to a controller.
builtin_interfaces/Time stamp
uint64 map_version

uint8 ACCEPTED=0
uint8 INVALID_OPTIONS=1
uint8 INVALID_INPUT=2
uint8 EMPTY_CANDIDATE=3
uint8 INVALID_SEGMENT=4
uint8 TIME_DISCONTINUITY=5
uint8 STATE_DISCONTINUITY=6
uint8 CURRENT_STATE_DISCONTINUITY=7
uint8 OUT_OF_MAP=8
uint8 STATIC_COLLISION=9
uint8 UNKNOWN_SPACE=10
uint8 VELOCITY_LIMIT=11
uint8 ACCELERATION_LIMIT=12
uint8 DYNAMIC_COLLISION=13
uint8 RISK_LIMIT=14
uint8 INFORMATION_STALE=15
uint8 status
bool accepted

int64 failed_segment
uint64 checked_segments
uint64 checked_samples
uint64 occupied_samples
uint64 unknown_samples
float64 maximum_velocity
float64 maximum_acceleration

builtin_interfaces/Time candidate_start_stamp
builtin_interfaces/Time candidate_end_stamp
bool current_trajectory_available
bool current_state_checked
bool current_state_continuous
bool current_trajectory_fallback_available
string detail
