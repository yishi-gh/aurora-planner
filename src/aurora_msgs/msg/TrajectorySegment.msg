# A time-windowed uniform B-spline segment.
# start_stamp is an absolute ROS time in the trajectory header frame.
builtin_interfaces/Time start_stamp
float64 source_start_time
float64 duration
float64 dt
int32 degree

uint8 CLAMPED=0
uint8 EGO_UNCLAMPED=1
uint8 knot_mode

# Control points are ordered by spline index, not by sampled execution time.
geometry_msgs/Point[] control_points
