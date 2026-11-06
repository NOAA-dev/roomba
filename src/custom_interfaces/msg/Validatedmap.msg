int32 counter

bool valid
string fail_reason         # empty when valid

# map->odom change over this snapshot window (max single-tick step).
# A large step is a SLAM correction (e.g. loop closure).
float32 map_odom_jump_m
float32 map_odom_jump_rad
bool loop_closure_suspected

builtin_interfaces/Time validation_time

nav_msgs/OccupancyGrid map
