string robot_id
uint32 ros_domain_id
bool receiver_ready
bool task_execution_enabled
string active_task_id
string last_received_task_id
builtin_interfaces/Time stamp

# Reported map pose from TF. pose_valid=false means pose must not be used.
geometry_msgs/PoseStamped pose
bool pose_valid
string pose_source

# Battery SOC. battery_percentage is normalized 0.0..1.0 when battery_valid.
# battery_valid=false means the numeric field must not be treated as SOC.
bool battery_valid
float32 battery_percentage
builtin_interfaces/Time battery_stamp
string battery_source
