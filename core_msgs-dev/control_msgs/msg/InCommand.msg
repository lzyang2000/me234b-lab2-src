# ROS message to publish who is in command

std_msgs/Header header

uint8 inCommand

# In Command Enum
uint8 NONE = 0
uint8 TELEOP = 1
uint8 CONTROLLER = 2
uint8 JOY_TIMEOUT = 3
uint8 TELEOP_TIMEOUT = 4
uint8 CONTROLLER_TIMEOUT = 5
uint8 ENABLED_TIMEOUT = 6
uint8 DISABLED = 7
