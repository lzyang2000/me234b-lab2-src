#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path, Odometry

class navigation():
    def __init__(self):
        self.map_pub = rospy.Publisher("map", OccupancyGrid, queue_size=1)
        self.map_sub = rospy.Subscriber("/mce234b/grid_published", Float32MultiArray, self.map_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber("/mce234b/goal_pose_published", PoseStamped, self.goal_callback, queue_size=1)
        self.curr_sub = rospy.Subscriber("/optitrack/vrpn_client_node/mce234b_bot/pose", PoseStamped, self.curr_callback, queue_size=1)
        self.curr_sub_sim = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.curr_callback_sim, queue_size=1)
        self.global_path_pub = rospy.Publisher("global_path", Path, queue_size=1)
        self.map = OccupancyGrid()
        self.has_map = False
        self.goal = PoseStamped()
        self.curr = PoseStamped()
        self.has_goal = False
        self.has_curr = False

    def map_callback(self, msg):
        # convert msg to numpy array
        map_np = np.array(msg.data).reshape((msg.layout.dim[0].size, msg.layout.dim[1].size))
        # set map
        self.set_map(map_np, 0.5, (0,0))
        # publish map
    
    def goal_callback(self, msg):
        # set goal
        self.goal = msg
        self.has_goal = True
        # publish path
    
    def curr_callback(self, msg):
        # set curr
        self.curr = msg
        self.has_curr = True
        # publish path
    
    def curr_callback_sim(self, msg):
        # set curr
        self.curr.pose = msg.pose.pose
        self.curr.header.frame_id = "map"
        self.has_curr = True
        # publish path

    def set_map(self, map_np, reso, origin):
        # map is 2d numpy array
        # reso is float
        # width is int
        # height is int
        # origin is tuple of floats
        #increase resolution
        width = map_np.shape[1]
        height = map_np.shape[0]
        i = 0
        while reso > 0.1:
            reso /= 2
            width *= 2
            height *= 2
            i += 1
        for _ in range(0,i):
            map_np = np.repeat(map_np, 2, axis=0).repeat(2, axis=1)
        map_np = np.flipud(map_np)
        map_np[map_np == 0] = 0
        map_np[map_np == 1] = 100
        self.map.header.frame_id = "map"
        self.map.header.stamp = rospy.Time.now()
        self.map.info.resolution = reso
        self.map.info.width = width
        self.map.info.height = height
        self.map.info.origin.position.x = origin[0]
        self.map.info.origin.position.y = origin[1]
        self.map.info.origin.position.z = 0
        self.map.info.origin.orientation.w = 1
        self.map.info.origin.orientation.x = 0
        self.map.info.origin.orientation.y = 0
        self.map.info.origin.orientation.z = 0
        # convert map_np to flatten numpy int array
        self.map.data = map_np.flatten().astype(int).tolist()
        self.has_map = True

    
    def publish(self):
        if self.has_map:
            self.map_pub.publish(self.map)

    def plan(self):
        if self.has_map and self.has_goal and self.has_curr:
            get_plan = rospy.ServiceProxy("/global_planner/planner/make_plan", GetPlan)
            resp1 = get_plan(self.curr, self.goal, 0.01)
            self.global_path_pub.publish(resp1.plan)

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

    rospy.init_node("map_pub")
    nav = navigation()
    

    # rospy.wait_for_service("/global_planner/planner/make_plan")
    rate = rospy.Rate(1)
    while rospy.is_shutdown() == False:
        # nav.map_pub.publish(map_array_msg)
        nav.publish()
        nav.plan()
        rate.sleep()