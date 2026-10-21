# The closest obstacle point the lidar can see. Published by get_object_range on /obstacle.
# In the robot's LOCAL frame (robot-centric): +x = straight ahead, +y = to the robot's left.
# The vector (x, y) points FROM the robot's look-ahead point TO the nearest point on the obstacle. 

bool found # is something in the detection range
float32 x # only populated if found = true
float32 y # only populated if found = true
float32 distance # length of x, y
float32 angle # direction in radians: 0 = straight ahead, + = left
