# Risk metrics are model-dependent until the risk policy is frozen.
builtin_interfaces/Time stamp
string model_id
float64 total_risk
float64 static_risk
float64 dynamic_risk
float64 information_risk
float64 tracking_risk
float64 minimum_clearance
float64 information_age
int64 worst_obstacle_id
builtin_interfaces/Time worst_time
bool dynamic_information_available
bool dynamic_information_stale

uint8 LOW=0
uint8 MEDIUM=1
uint8 HIGH=2
uint8 UNKNOWN=3
uint8 risk_level
string detail
