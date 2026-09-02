std_msgs/Header header
DynamicObstacleTrack[] tracks

# An explicit occlusion indication is distinct from a valid empty heartbeat.
# When true, the dynamic-obstacle set must not be used for new trajectories.
bool occlusion_active
# Optional IDs affected by the occlusion. An empty list with occlusion_active
# true means that the affected set is not known.
uint64[] occluded_track_ids
