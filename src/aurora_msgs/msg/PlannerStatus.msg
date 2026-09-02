std_msgs/Header header
uint64 request_id
uint64 trajectory_id

uint8 INIT=0
uint8 WAIT_TARGET=1
uint8 GENERATE=2
uint8 EXECUTE=3
uint8 REPLAN=4
uint8 DEGRADED=5
uint8 EMERGENCY_STOP=6
uint8 planner_state

uint8 WAIT=0
uint8 REQUEST_REPLAN=1
uint8 START_PLANNING=2
uint8 WAIT_FOR_RESULT=3
uint8 ACCEPT_NEW_TRAJECTORY=4
uint8 KEEP_CURRENT_TRAJECTORY=5
uint8 HOLD_POSITION=6
uint8 EMERGENCY_STOP_ACTION=7
uint8 INVALID_INPUT_ACTION=8
uint8 planner_action

uint8 NONE=0
uint8 CURRENT_TRAJECTORY_COLLISION=1
uint8 SAFETY_INFORMATION_STALE=2
uint8 PLANNING_FAILURE=3
uint8 MAP_UPDATED=4
uint8 TRAJECTORY_NEAR_END=5
uint8 LOCAL_GOAL_EXPIRED=6
uint8 DYNAMIC_OBSTACLE_UPDATED=7
uint8 replan_trigger

uint8 planning_status
uint8 safety_status
uint32 consecutive_failures
bool valid
string detail
