std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration

# Row-major covariance for [px, py, pz, vx, vy, vz, ax, ay, az].
bool has_state_covariance
float64[81] state_covariance
