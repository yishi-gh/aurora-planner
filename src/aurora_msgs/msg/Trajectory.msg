std_msgs/Header header
uint64 trajectory_id
uint64 map_version

uint8 VALIDATED=0
uint8 DEGRADED=1
uint8 REJECTED=2
uint8 validation_state

TrajectoryConstraints constraints
RiskReport risk_report
SafetyReport safety_report
TrajectorySegment[] segments
