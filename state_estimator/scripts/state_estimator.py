#!/usr/bin/python3
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class IMU_process:
    def __init__(self):
        self.gt_path = Path()
        self.imu_path = Path()
        # self.path.header.frame_id = "world"
        # self.path.header.stamp = rospy.Time.now()

        self.gt_sub = rospy.Subscriber('/optitrack/vrpn_client_node/mce234b_bot/pose', PoseStamped, self.gt_cb)
        self.gt_pub = rospy.Publisher('/gt_path', Path, queue_size=10)
        self.imu_elevation_sub = rospy.Subscriber('/imu_elevation', PoseStamped, self.imu_elevation_cb)

    def gt_cb(self, data):
        self.gt_path.header = data.header
        self.gt_path.poses.append(data)
        self.gt_pub.publish(self.gt_path)

if __name__ == '__main__':
    rospy.init_node('gt_node')
    imu_process = IMU_process()
    rospy.spin()