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

        self.gt_sub = rospy.Subscriber(
            '/optitrack/vrpn_client_node/mce234b_bot/pose', PoseStamped, self.gt_cb)
        self.gt_pub = rospy.Publisher('/gt_path', Path, queue_size=10)
        self.imu_sub = rospy.Subscriber('/vn100/imu/imu', Imu, self.imu_cb)
        self.imu_ori_offset = [0, 0, 0]
        self.angular_velocity_offset = [0, 0, 0]
        self.imu_acc_offset = [0, 0, 0]
        # self.imu_init = True
        self.init_step = 0
        self.init_max = 10
        self.orientation_imu = [0, 0, 0]
        self.angular_velocity_imu = [0, 0, 0]
        self.linear_acceleration_imu = [0, 0, 0]
        # self.elevation = 0
        # self.elevation_dot = 0
        # self.elevation_dot_dot = 0
        self.pos_list = []
        self.pos = [0, 0, 0]
        self.vel = [0, 0, 0]
        self.acc = [0, 0, 0]
        self.init = True
        self.imu_time = 0
        self.imu_old_time = 0
        self.gt_pos_list = []
        self.orientation_imu_list = []
        self.orientation_gt_list = []
        self.velocity_imu_list = []
        self.velocity_gt_list = []
        self.gt_time_list = []
        self.init_ori = [0, 0, 0]
        self.imu_time_list = None
        self.start_pos = True
        self.old_pos = [0, 0, 0]
        self.gt_old_time = 0

    def gt_cb(self, data):
        self.gt_path.header = data.header
        self.gt_path.poses.append(data)
        self.gt_ori = tf.transformations.euler_from_quaternion(
            [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        self.gt_pub.publish(self.gt_path)
        if self.init:
            self.pos = [data.pose.position.x,
                        data.pose.position.y, data.pose.position.z]
            self.init_ori = self.gt_ori
            self.old_pos = self.pos
            self.init = False
        if self.imu_time_list:
            if self.start_pos:
                self.pos = [data.pose.position.x,
                            data.pose.position.y, data.pose.position.z]
            self.start_pos = False
            self.gt_time_list.append(self.imu_time-self.imu_time_list[0])
            self.gt_pos_list.append(
                [data.pose.position.x, data.pose.position.y, data.pose.position.z])
            if self.gt_old_time == 0:
                self.gt_old_time = self.imu_time
            self.orientation_gt_list.append(self.gt_ori)
            if(self.imu_time-self.gt_old_time == 0):
                self.velocity_gt_list.append([0, 0, 0])
            else:
                self.velocity_gt_list.append(
                [(data.pose.position.x-self.old_pos[0])/(self.imu_time-self.gt_old_time), (data.pose.position.y-self.old_pos[1])/(self.imu_time-self.gt_old_time), (data.pose.position.z-self.old_pos[2])/(self.imu_time-self.gt_old_time)])
            self.old_pos = [data.pose.position.x,
                            data.pose.position.y, data.pose.position.z]
            self.gt_old_time = self.imu_time

    def imu_cb(self, data):
        time = data.header.stamp.to_sec()
        orientation = data.orientation
        angular_velocity = data.angular_velocity
        linear_acceleration = data.linear_acceleration
        linear_acceleration = [linear_acceleration.x,
                               -linear_acceleration.y, -linear_acceleration.z]
        orientation_imu_raw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        orientation_imu_raw = [orientation_imu_raw[0], orientation_imu_raw[1],
                               -orientation_imu_raw[2]]  # flip the yaw
        angular_velocity = [angular_velocity.x, angular_velocity.y,
                            -angular_velocity.z]
        if self.init_step < self.init_max and not self.init:  # init for 0.5 second
            self.imu_acc_offset = [linear_acceleration[0] + self.imu_acc_offset[0], linear_acceleration[1] +
                                   self.imu_acc_offset[1], linear_acceleration[2] + self.imu_acc_offset[2]]
            self.imu_ori_offset = [orientation_imu_raw[0] - self.init_ori[0] + self.imu_ori_offset[0], orientation_imu_raw[1] -
                                   self.init_ori[1] + self.imu_ori_offset[1], orientation_imu_raw[2] - self.init_ori[2] + self.imu_ori_offset[2]]
            self.angular_velocity_offset = [angular_velocity[0] + self.angular_velocity_offset[0], angular_velocity[1] +
                                            self.angular_velocity_offset[1], angular_velocity[2] + self.angular_velocity_offset[2]]
            self.init_step += 1
        elif self.init_step == self.init_max and not self.init:
            self.imu_acc_offset = [self.imu_acc_offset[0]/self.init_max,
                                   self.imu_acc_offset[1]/self.init_max, self.imu_acc_offset[2]/self.init_max-0.12]
            self.imu_ori_offset = [self.imu_ori_offset[0]/self.init_max,
                                   self.imu_ori_offset[1]/self.init_max, self.imu_ori_offset[2]/self.init_max+0.3]
            self.angular_velocity_offset = [self.angular_velocity_offset[0]/self.init_max,
                                            self.angular_velocity_offset[1]/self.init_max, self.angular_velocity_offset[2]/self.init_max]
            self.imu_time_list = [time]
            self.init_step += 1  # init finished
        elif not self.init:
            self.imu_time = time
            self.linear_acceleration_imu = [linear_acceleration[0] - self.imu_acc_offset[0],
                                            linear_acceleration[1] - self.imu_acc_offset[1], linear_acceleration[2] - self.imu_acc_offset[2]]
            self.orientation_imu = [orientation_imu_raw[0] - self.imu_ori_offset[0], orientation_imu_raw[1] -
                                    self.imu_ori_offset[1], orientation_imu_raw[2] - self.imu_ori_offset[2]]  # roll pitch still wrong I think
            # clip self.orientation_imu to -pi to pi
            self.orientation_imu = [np.clip(self.orientation_imu[0], -np.pi, np.pi), np.clip(
                self.orientation_imu[1], -np.pi, np.pi), np.clip(self.orientation_imu[2], -np.pi, np.pi)]
            self.angular_velocity_imu = [angular_velocity[0] - self.angular_velocity_offset[0], angular_velocity[1] -
                                         self.angular_velocity_offset[1], angular_velocity[2] - self.angular_velocity_offset[2]]
            # self.elevation_dot_dot = self.linear_acceleration_imu[2]
            if self.imu_old_time == 0:  # first time
                self.imu_old_time = self.imu_time
            # transform self.acc to world frame with self.orientation_imu[2]
            self.acc = [self.linear_acceleration_imu[0]*np.cos(self.orientation_imu[2]) - self.linear_acceleration_imu[1]*np.sin(self.orientation_imu[2]), self.linear_acceleration_imu[0]*np.sin(
                self.orientation_imu[2]) + self.linear_acceleration_imu[1]*np.cos(self.orientation_imu[2]), self.linear_acceleration_imu[2]]

            self.vel = [self.vel[0] + self.acc[0]*(self.imu_time - self.imu_old_time), self.vel[1] + self.acc[1]*(
                self.imu_time - self.imu_old_time), self.vel[2] + self.acc[2]*(self.imu_time - self.imu_old_time)]
            self.pos = [self.pos[0] + self.vel[0]*(self.imu_time - self.imu_old_time), self.pos[1] + self.vel[1]*(
                self.imu_time - self.imu_old_time), self.pos[2] + self.vel[2]*(self.imu_time - self.imu_old_time)]
            # self.yaw = self.yaw + self.angular_velocity_imu[2]*(self.imu_time - self.imu_old_time)
            self.pos_list.append(self.pos)

            # self.elevation_dot = self.elevation_dot + self.elevation_dot_dot * (self.imu_time - self.imu_old_time)
            # self.elevation = self.elevation + self.elevation_dot * (self.imu_time - self.imu_old_time)
            self.imu_old_time = self.imu_time
            self.imu_time_list.append(self.imu_time-self.imu_time_list[0])
            self.orientation_imu_list.append(self.orientation_imu)
            self.velocity_imu_list.append(self.vel)

    def plot(self):
        self.imu_time_list.pop(0)
        self.pos_list = np.array(self.pos_list).T
        self.gt_pos_list = np.array(self.gt_pos_list).T
        self.orientation_imu_list = np.array(self.orientation_imu_list).T
        self.orientation_gt_list = np.array(self.orientation_gt_list).T
        self.velocity_gt_list = np.array(self.velocity_gt_list).T
        self.velocity_imu_list = np.array(self.velocity_imu_list).T
        self.gt_time_list = np.array(self.gt_time_list)
        self.imu_time_list = np.array(self.imu_time_list)

        plt.plot(self.gt_time_list, self.gt_pos_list[2])
        plt.plot(self.imu_time_list, self.pos_list[2])
        plt.xlabel('time (s)')
        plt.ylabel('elevation (m)')
        plt.title('Elevation vs time')
        plt.legend(['OptiTrack', 'IMU'])
        plt.savefig('../figs/imu_elevation.png')
        plt.close()

        plt.plot(self.gt_pos_list[0], self.gt_pos_list[1])
        plt.plot(self.pos_list[0], self.pos_list[1])
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.title('path')
        plt.legend(['OptiTrack', 'IMU'])
        plt.savefig('../figs/path.png')
        plt.close()

        plt.plot(self.gt_time_list, self.orientation_gt_list[2])
        plt.plot(self.imu_time_list, self.orientation_imu_list[2])
        plt.xlabel('time (s)')
        plt.ylabel('yaw (rad)')
        plt.title('Yaw vs time')
        plt.legend(['OptiTrack', 'IMU'])
        plt.savefig('../figs/yaw.png')
        plt.close()

        plt.plot(self.gt_time_list[np.where(self.velocity_gt_list[0]>0)], self.velocity_gt_list[0][np.where(self.velocity_gt_list[0]>0)])
        plt.plot(self.gt_time_list[np.where(self.velocity_gt_list[1]>0)], self.velocity_gt_list[1][np.where(self.velocity_gt_list[1]>0)])
        plt.plot(self.imu_time_list, self.velocity_imu_list[0])
        plt.plot(self.imu_time_list, self.velocity_imu_list[1])
        plt.xlabel('time (s)')
        plt.ylabel('velocity (m/s)')
        plt.title('Velocity vs time')
        plt.legend(['OptiTrack x', 'OptiTrack y', 'IMU x', 'IMU y'])
        plt.savefig('../figs/velocity.png')




        # plt.show()


if __name__ == '__main__':
    rospy.init_node('se_node')
    state_estimator = state_estimator()
    rospy.spin()
    rospy.on_shutdown(state_estimator.plot())
