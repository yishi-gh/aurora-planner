std_msgs/Header header

# Three-dimensional position is the required detection measurement.
geometry_msgs/Point position

bool has_position_covariance
# Row-major covariance for [px, py, pz].
float64[9] position_covariance

bool has_velocity
geometry_msgs/Vector3 velocity
bool has_velocity_covariance
# Row-major covariance for [vx, vy, vz].
float64[9] velocity_covariance

# Shape is optional. When absent, the tracker applies its configured default.
bool has_shape
uint8 SPHERE=0
uint8 BOX=1
uint8 CAPSULE=2
uint8 MULTI_SPHERE=3
uint8 shape_type
geometry_msgs/Vector3 dimensions
float64 radius
