#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan

rospy.init_node("move_base_demo")

# publisher to change the arm group

goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
start_pub = rospy.Publisher("curr_state", PoseStamped, queue_size=1)
path_pub = rospy.Publisher("path", Path, queue_size=1)
goal = PoseStamped()
goal.header.frame_id = "flipper/map"
goal.header.stamp = rospy.Time.now()
goal.pose.position.z = 0.0
goal.pose.position.x = 1.25
goal.pose.position.y = 5.25
goal.pose.orientation.w = 1
goal.pose.orientation.x = 0
goal.pose.orientation.y = 0
goal.pose.orientation.z = 0

start = PoseStamped()
start.header.frame_id = "flipper/map"
start.header.stamp = rospy.Time.now()
start.pose.position.z = 0.0
start.pose.position.x = 5.25
start.pose.position.y = 0.75
start.pose.orientation.w = 1
start.pose.orientation.x = 0
start.pose.orientation.y = 0
start.pose.orientation.z = 0

rospy.wait_for_service("/global_planner/planner/make_plan")
while not rospy.is_shutdown():
    try:
        get_plan = rospy.ServiceProxy("/global_planner/planner/make_plan", GetPlan)
        resp1 = get_plan(start, goal, 0.01)
        goal_pub.publish(goal)
        start_pub.publish(start)
        path_pub.publish(resp1.plan)
        # print(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    rospy.sleep(1)
