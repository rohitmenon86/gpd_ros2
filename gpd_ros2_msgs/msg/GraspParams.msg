# Approach direction vector (x, y, z)
geometry_msgs/Vector3 approach_direction
# Camera positions (flattened: [x1, y1, z1, x2, y2, z2, ...])
float64[] camera_position
# Transform from camera to base frame
geometry_msgs/Transform transform_camera2base
# Workspace limits [x_min, x_max, y_min, y_max, z_min, z_max]
float64[] workspace
# Whether approach filtering can be applied
bool enable_approach_dir_filtering
# Threshold in degrees
float64 approach_dir_threshold