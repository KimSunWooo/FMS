# One planned map point. orientation_is_planned is false when the planner
# has no heading. The quaternion is then identity and is not a planned yaw.
string waypoint_id
geometry_msgs/PoseStamped pose
bool orientation_is_planned
