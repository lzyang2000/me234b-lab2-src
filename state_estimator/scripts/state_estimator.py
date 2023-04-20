#!/usr/bin/python3
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
class state_estimator:
    def __init__(self):
        self.gt_path = Path()
        self.imu_path = Path()
        # self.path.header.frame_id = "world"
        # self.path.header.stamp = rospy.Time.now()

        self.gt_sub = rospy.Subscriber('/optitrack/vrpn_client_node/mce234b_bot/pose', PoseStamped, self.gt_cb)
        self.gt_pub = rospy.Publisher('/gt_path', Path, queue_size=10)
        self.imu_sub = rospy.Subscriber('/vn100/imu/imu', Imu, self.imu_cb)
        self.imu_ori_offset = [0, 0, 0]
        self.angular_velocity_offset = [0, 0, 0]
        self.imu_acc_offset = [0, 0, 0]
        # self.imu_init = True
        self.init_step = 0
        self.init_max = 100
        self.orientation_imu = [0, 0, 0]
        self.angular_velocity_imu = [0, 0, 0]
        self.linear_acceleration_imu = [0, 0, 0]
        # self.elevation = 0
        # self.elevation_dot = 0
        # self.elevation_dot_dot = 0
        self.pos_list = []
        self.gt_elevation_list = [0]
        self.pos = [0, 0, 0]
        self.vel = [0, 0, 0]
        self.acc = [0, 0, 0]
        self.init = True
        self.imu_time = 0
        self.imu_old_time = 0
        self.gt_pos_list = []
        self.orientation_imu_list = []
        self.orientation_gt_list = []
        

    def gt_cb(self, data):
        self.gt_path.header = data.header
        self.gt_path.poses.append(data)
        self.gt_pos_list.append([data.pose.position.x, data.pose.position.y])
        if self.init:
            self.pos = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
            self.init = False
        self.gt_elevation_list.append(data.pose.position.z)
        self.gt_ori = tf.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        self.gt_pub.publish(self.gt_path)
        self.orientation_gt_list.append(self.gt_ori)
    
    def imu_cb(self, data):
        time = data.header.stamp.to_sec()
        orientation = data.orientation
        angular_velocity = data.angular_velocity
        linear_acceleration = data.linear_acceleration
        linear_acceleration = [linear_acceleration.x, -linear_acceleration.y, -linear_acceleration.z]
        self.orientation_imu_raw = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        if self.init_step < self.init_max: # init for 0.5 second
            self.imu_acc_offset = [linear_acceleration[0] + self.imu_acc_offset[0], linear_acceleration[1] + self.imu_acc_offset[1], linear_acceleration[2] + self.imu_acc_offset[2]]
            imu_ori_offset_curr = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            self.imu_ori_offset = [imu_ori_offset_curr[0] + self.imu_ori_offset[0], imu_ori_offset_curr[1] + self.imu_ori_offset[1], imu_ori_offset_curr[2] + self.imu_ori_offset[2]]
            self.angular_velocity_offset = [angular_velocity.x + self.angular_velocity_offset[0], angular_velocity.y + self.angular_velocity_offset[1], angular_velocity.z + self.angular_velocity_offset[2]]
            self.init_step += 1
        elif self.init_step == self.init_max:
            self.imu_acc_offset = [self.imu_acc_offset[0]/self.init_max, self.imu_acc_offset[1]/self.init_max, self.imu_acc_offset[2]/self.init_max]
            self.imu_ori_offset = [self.imu_ori_offset[0]/self.init_max, self.imu_ori_offset[1]/self.init_max, self.imu_ori_offset[2]/self.init_max]
            self.angular_velocity_offset = [self.angular_velocity_offset[0]/self.init_max, self.angular_velocity_offset[1]/self.init_max, self.angular_velocity_offset[2]/self.init_max]
            self.imu_time_list = [time]
            self.init_step += 1 # init finished
            self.yaw = self.gt_ori[2]
        elif not self.init:
            self.imu_time = time
            print("gt yaw:", self.gt_ori[2], " imu yaw:", self.orientation_imu[2])
            self.linear_acceleration_imu = [linear_acceleration[0] - self.imu_acc_offset[0], linear_acceleration[1] - self.imu_acc_offset[1], linear_acceleration[2] - self.imu_acc_offset[2]]
            # self.orientation_imu = [self.orientation_imu_raw[0] - self.imu_ori_offset[0], self.orientation_imu_raw[1] - self.imu_ori_offset[1], self.orientation_imu_raw[2] - self.imu_ori_offset[2]]
            self.orientation_imu = self.orientation_imu_raw
            self.angular_velocity_imu = [angular_velocity.x - self.angular_velocity_offset[0], angular_velocity.y - self.angular_velocity_offset[1], angular_velocity.z - self.angular_velocity_offset[2]]
            # self.elevation_dot_dot = self.linear_acceleration_imu[2]
            if self.imu_old_time == 0: # first time
                self.imu_old_time = self.imu_time
            # transform self.acc to world frame with self.orientation_imu[2]
            self.acc = [self.linear_acceleration_imu[0]*np.cos(self.orientation_imu[2]) + self.linear_acceleration_imu[1]*np.sin(self.orientation_imu[2]), -self.linear_acceleration_imu[0]*np.sin(self.orientation_imu[2]) + self.linear_acceleration_imu[1]*np.cos(self.orientation_imu[2]), self.linear_acceleration_imu[2]]
            
            self.vel = [self.vel[0] + self.acc[0]*(self.imu_time - self.imu_old_time), self.vel[1] + self.acc[1]*(self.imu_time - self.imu_old_time), self.vel[2] + self.acc[2]*(self.imu_time - self.imu_old_time)]
            self.pos = [self.pos[0] + self.vel[0]*(self.imu_time - self.imu_old_time), self.pos[1] + self.vel[1]*(self.imu_time - self.imu_old_time), self.pos[2] + self.vel[2]*(self.imu_time - self.imu_old_time)]
            # self.yaw = self.yaw + self.angular_velocity_imu[2]*(self.imu_time - self.imu_old_time)
            self.pos_list.append(self.pos)

            # self.elevation_dot = self.elevation_dot + self.elevation_dot_dot * (self.imu_time - self.imu_old_time)
            # self.elevation = self.elevation + self.elevation_dot * (self.imu_time - self.imu_old_time)
            self.imu_old_time = self.imu_time
            self.imu_time_list.append(self.imu_time-self.imu_time_list[0])
            self.orientation_imu_list.append(self.orientation_imu)
            
            
            

    
    def plot(self):
        self.imu_time_list.pop(0)
        self.pos_list = np.array(self.pos_list).T
        self.gt_pos_list = np.array(self.gt_pos_list).T
        self.orientation_imu_list = np.array(self.orientation_imu_list).T
        self.orientation_gt_list = np.array(self.orientation_gt_list).T
        plt.plot(self.imu_time_list, self.pos_list[2])
        plt.xlabel('time (s)')
        plt.ylabel('elevation (m)')
        plt.title('IMU elevation vs time')
        plt.savefig('../figs/imu_elevation.png')
        plt.close()
        plt.plot(self.gt_pos_list[0], self.gt_pos_list[1])
        plt.plot(self.pos_list[0], self.pos_list[1])
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.title('path')
        plt.legend(['ground truth', 'state estimator'])
        plt.savefig('../figs/path.png')
        plt.close()
        plt.plot(self.imu_time_list, self.orientation_imu_list[2])
        plt.plot()
        plt.xlabel('time (s)')
        plt.ylabel('yaw (rad)')
        plt.title('IMU yaw vs time')
        plt.savefig('../figs/imu_yaw.png')
        plt.close()
        plt.plot(self.orientation_gt_list[2])
        plt.xlabel('idx (s)')
        plt.ylabel('yaw (rad)')
        plt.title('ground truth yaw vs time')
        plt.savefig('../figs/gt_yaw.png')

        # plt.show()

        



if __name__ == '__main__':
    rospy.init_node('se_node')
    state_estimator = state_estimator()
    rospy.spin()
    rospy.on_shutdown(state_estimator.plot())