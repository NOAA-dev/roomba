int32 counter
float32 map_score          # 0-100

bool valid

bool fragmented
int32 fragments
bool is_dominant
float32 reachable_ratio

bool drift_detected
float32 drift_score
float32 drift_distance

builtin_interfaces/Time validation_time

nav_msgs/OccupancyGrid map