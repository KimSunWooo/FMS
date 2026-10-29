# Receive-only task. execute must be false. This message does not start Nav2.
string task_id
string robot_id
string task_type
string map_id
string map_revision
string goal_label
geometry_msgs/Point start
geometry_msgs/Point goal
fms_msgs/Waypoint[] waypoints
string payload_hash
string command_id
# Exact canonical JSON used for payload_hash. Structured fields must match it.
string canonical_payload
bool execute
