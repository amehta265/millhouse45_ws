# Where the ball is relative to the robot. Used on both /obj/bearing and /obj/range topics
# detect_object    -> fills found + bearing, distance = NaN (camera can't tell how far)
# get_object_range -> fills found + bearing + distance (from the lidar)
# bearing in radians, local/robot/ego-centric frame: 0 = straight ahead, + = left. distance in meters
bool found
float32 bearing
float32 distance
