#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path



if __name__ == '__main__':
    map_array = np.zeros((13,13))
    # all boarder cells are obstacles
    map_array[0,:] = 1
    map_array[-1,:] = 1
    map_array[:,0] = 1
    map_array[:,-1] = 1
    # add obstacles
    # map_array[1,1] = 1
    # map_array[4:11,1] = 1
    # map_array[1:9,4:6] = 1
    # map_array[1:3,8:11] = 1
    # map_array[6:8,8:10] = 1
    # map_array[8:10,9] = 1
    # add 5x5 obstacle
    map_array[4:9,4:9] = 1
    # print(map_array)
    # convert map_array to float32multiarray
    map_array_msg = Float32MultiArray()
    map_array_msg.data = map_array.flatten().tolist()
    map_array_msg.layout.dim.append(MultiArrayDimension())
    map_array_msg.layout.dim[0].label = "height"
    map_array_msg.layout.dim[0].size = map_array.shape[0]
    map_array_msg.layout.dim[0].stride = map_array.shape[1]
    map_array_msg.layout.dim.append(MultiArrayDimension())
    map_array_msg.layout.dim[1].label = "width"
    map_array_msg.layout.dim[1].size = map_array.shape[1]
    map_array_msg.layout.dim[1].stride = map_array.shape[1]
    # print(map_array_msg)
    grid_pub = rospy.Publisher("/mce234b/grid_published", Float32MultiArray, queue_size=1)
    rospy.init_node("map_sim")
    

    # rospy.wait_for_service("/global_planner/planner/make_plan")
    rate = rospy.Rate(10)
    while rospy.is_shutdown() == False:
        grid_pub.publish(map_array_msg)
        # nav.publish()
        # # nav.plan()
        rate.sleep()