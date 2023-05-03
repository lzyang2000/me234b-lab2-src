#!/usr/bin/env python3

import rospy

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped

rospy.init_node("move_base_demo")

# publisher to change the arm group

goal_pub = rospy.Publisher("move_base_simple/goal",  PoseStamped,queue_size=1)

goal = PoseStamped()
goal.header.frame_id = "flipper/map"
goal.header.stamp = rospy.Time.now()
goal.pose.position.z = 0.0
goal.pose.position.x = -2
goal.pose.position.y = 1
goal.pose.orientation.w = 0
goal.pose.orientation.x = 0
goal.pose.orientation.y = 0
goal.pose.orientation.z = -1
while rospy.is_shutdown() == False:
    goal_pub.publish(goal)
    rospy.sleep(2.0)