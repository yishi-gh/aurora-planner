std_msgs/Header header
UnassociatedObstacleDetection[] detections

# True means the sensor cannot provide a complete detection set. It is
# distinct from a valid empty batch and enters the existing stale-information
# safety policy.
bool occlusion_active
