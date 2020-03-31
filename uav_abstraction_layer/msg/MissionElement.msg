
uint8 TAKEOFF_POSE = 0
uint8 TAKEOFF_AUX = 1
uint8 PASS = 2
uint8 LOITER_UNLIMITED = 3
uint8 LOITER_TURNS = 4
uint8 LOITER_TIME = 5
uint8 LOITER_HEIGHT = 6
uint8 LAND_POSE = 7
uint8 LAND_AUX = 8

uint8 type
geometry_msgs/PoseStamped[] waypoints
ParamFloat[] params