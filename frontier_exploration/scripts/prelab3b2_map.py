#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid


class MapPub():
    def __init__(self):
        self.map_pub = rospy.Publisher("map", OccupancyGrid, queue_size=1)
        self.map = OccupancyGrid()

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
        print(map_np)
        map_np[map_np == 0] = 0
        map_np[map_np == 1] = 100
        self.map.header.frame_id = "flipper/map"
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

    
    def publish(self):
        self.map_pub.publish(self.map)

if __name__ == '__main__':
    map_array = np.zeros((12,12))
    # all boarder cells are obstacles
    map_array[0,:] = 1
    map_array[-1,:] = 1
    map_array[:,0] = 1
    map_array[:,-1] = 1
    # add obstacles
    map_array[1,1] = 1
    map_array[4:11,1] = 1
    map_array[1:9,4:6] = 1
    map_array[1:3,8:11] = 1
    map_array[6:8,8:10] = 1
    map_array[8:10,9] = 1
    # print(map_array)

    rospy.init_node("map_pub")
    map_pub = MapPub()
    map_pub.set_map(map_array, 0.5, (0,0))
    rate = rospy.Rate(1)
    while rospy.is_shutdown() == False:
        map_pub.publish()
        rate.sleep()