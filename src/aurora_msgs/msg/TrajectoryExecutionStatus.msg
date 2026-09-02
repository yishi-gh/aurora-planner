std_msgs/Header header
uint64 trajectory_id

uint8 IDLE=0
uint8 ACTIVE=1
uint8 COMPLETED=2
uint8 REJECTED_UNVALIDATED=3
uint8 REJECTED_UNSAFE=4
uint8 REJECTED_INVALID=5
uint8 EMERGENCY_STOP=6
uint8 TIME_ROLLBACK=7
uint8 TIME_GAP=8
uint8 status

bool accepted
bool active
float64 tracking_error
string detail
