# ROS Message to Send Via MQTT

std_msgs/Header header

uint8 control_state

# Control States
uint8 PATH_PLAN = 0
uint8 SAFETY_ANALYSIS = 1
uint8 OPTIMIZING_CONTROL = 2
uint8 SEARCHING_SOLUTION = 3
uint8 SIMULATING = 4