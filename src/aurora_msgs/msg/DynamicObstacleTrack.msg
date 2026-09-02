std_msgs/Header header
uint64 track_id
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Accel acceleration

uint8 SPHERE=0
uint8 BOX=1
uint8 CAPSULE=2
uint8 MULTI_SPHERE=3
uint8 shape_type
geometry_msgs/Vector3 dimensions
float64 radius

float64 existence_probability
# Row-major covariance for [px, py, pz, vx, vy, vz].
bool has_state_covariance
float64[36] state_covariance

uint8 CV=0
uint8 CA=1
uint8 prediction_model
