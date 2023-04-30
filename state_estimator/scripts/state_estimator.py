#!/usr/bin/python3
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
from gvrbot.msg import GvrbotMobilityData
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class state_estimator:
    def __init__(self):
        self.gt_path = Path()
        self.imu_path = Path()
        # self.path.header.frame_id = "world"
        # self.path.header.stamp = rospy.Time.now()

        self.gt_sub = rospy.Subscriber(
            "/optitrack/vrpn_client_node/mce234b_bot/pose", PoseStamped, self.gt_cb
        )
        self.gt_pub = rospy.Publisher("/gt_path", Path, queue_size=10)
        self.imu_sub = rospy.Subscriber("/vn100/imu/imu", Imu, self.imu_cb)
        self.imu_repub_sub = rospy.Subscriber(
            "/vn100/imu/imu", Imu, self.imu_repub_cb
        )
        self.imu_pub = rospy.Publisher("/imu_data", Imu, queue_size=10)
        self.track_sub = rospy.Subscriber(
            "/gvrbot_mobility_data", GvrbotMobilityData, self.track_cb
        )
        self.track_odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.track_path_pub = rospy.Publisher("/track_path", Path, queue_size=10)
        self.ekf_sub = rospy.Subscriber(
            "/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.ekf_cb
        )
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
        self.imu_pos_list = []
        self.imu_pos = [0, 0, 0]
        self.track_pos = [0, 0, 0]
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
        self.track_old_time = 0
        self.track_odom = Odometry()
        self.seq = 0
        self.track_width = 0.305
        self.track_yaw = 0
        self.track_time_list = []
        self.track_yaw_list = []
        self.track_pos_list = []
        self.imu_header_stamp = None
        self.ekf_pos_list = []
        self.ekf_error_list = []
        self.ekf_cov_list = []
        self.ekf_time_list = []

    def gt_cb(self, data):
        self.gt_path.header = data.header
        self.gt_path.poses.append(data)
        self.gt_ori = tf.transformations.euler_from_quaternion(
            [
                data.pose.orientation.x,
                data.pose.orientation.y,
                data.pose.orientation.z,
                data.pose.orientation.w,
            ]
        )
        self.gt_pub.publish(self.gt_path)
        if self.init:
            self.imu_pos = [
                data.pose.position.x,
                data.pose.position.y,
                data.pose.position.z,
            ]
            self.track_pos = self.imu_pos.copy()
            self.track_yaw = self.gt_ori[2]
            self.init_ori = self.gt_ori
            self.old_pos = self.imu_pos
            self.init = False
        if self.imu_time_list:
            if self.start_pos:
                self.imu_pos = [
                    data.pose.position.x,
                    data.pose.position.y,
                    data.pose.position.z,
                ]
                self.track_pos = self.imu_pos.copy()
                self.track_yaw = self.gt_ori[2]
            self.start_pos = False
            self.gt_time_list.append(self.imu_time - self.imu_time_list[0])
            self.gt_pos_list.append(
                [data.pose.position.x, data.pose.position.y, data.pose.position.z]
            )
            if self.gt_old_time == 0:
                self.gt_old_time = self.imu_time
            self.orientation_gt_list.append(self.gt_ori)
            if self.imu_time - self.gt_old_time == 0:
                self.velocity_gt_list.append([0, 0, 0])
            else:
                self.velocity_gt_list.append(
                    [
                        (data.pose.position.x - self.old_pos[0])
                        / (self.imu_time - self.gt_old_time),
                        (data.pose.position.y - self.old_pos[1])
                        / (self.imu_time - self.gt_old_time),
                        (data.pose.position.z - self.old_pos[2])
                        / (self.imu_time - self.gt_old_time),
                    ]
                )
            self.old_pos = [
                data.pose.position.x,
                data.pose.position.y,
                data.pose.position.z,
            ]
            self.gt_old_time = self.imu_time

    def imu_repub_cb(self, data):
        data.header.frame_id = "vn100"
        linear_acceleration = data.linear_acceleration
        linear_acceleration = [
            linear_acceleration.x,
            -linear_acceleration.y,
            -linear_acceleration.z,
        ]
        data.linear_acceleration.x = linear_acceleration[0]
        data.linear_acceleration.y = linear_acceleration[1]
        data.linear_acceleration.z = linear_acceleration[2]
        orientation_imu_raw = tf.transformations.euler_from_quaternion(
            [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        )
        orientation_imu_raw = [
            orientation_imu_raw[0],
            orientation_imu_raw[1],
            -orientation_imu_raw[2],
        ]  # flip the yaw
        data_or = tf.transformations.quaternion_from_euler(
            orientation_imu_raw[0],
            orientation_imu_raw[1],
            orientation_imu_raw[2],
            axes="sxyz",
        )
        data.orientation.x = data_or[0]
        data.orientation.y = data_or[1]
        data.orientation.z = data_or[2]
        data.orientation.w = data_or[3]
        self.imu_pub.publish(data)

    def imu_cb(self, data):
        time = data.header.stamp.to_sec()
        self.imu_header_stamp = data.header.stamp
        orientation = data.orientation
        angular_velocity = data.angular_velocity
        linear_acceleration = data.linear_acceleration
        linear_acceleration = [
            linear_acceleration.x,
            -linear_acceleration.y,
            -linear_acceleration.z,
        ]
        orientation_imu_raw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        orientation_imu_raw = [
            orientation_imu_raw[0],
            orientation_imu_raw[1],
            -orientation_imu_raw[2],
        ]  # flip the yaw
        angular_velocity = [angular_velocity.x, angular_velocity.y, -angular_velocity.z]
        if self.init_step < self.init_max and not self.init:  # init for 0.05 second
            self.imu_acc_offset = [
                linear_acceleration[0] + self.imu_acc_offset[0],
                linear_acceleration[1] + self.imu_acc_offset[1],
                linear_acceleration[2] + self.imu_acc_offset[2],
            ]
            self.imu_ori_offset = [
                orientation_imu_raw[0] - self.init_ori[0] + self.imu_ori_offset[0],
                orientation_imu_raw[1] - self.init_ori[1] + self.imu_ori_offset[1],
                orientation_imu_raw[2] - self.init_ori[2] + self.imu_ori_offset[2],
            ]
            self.angular_velocity_offset = [
                angular_velocity[0] + self.angular_velocity_offset[0],
                angular_velocity[1] + self.angular_velocity_offset[1],
                angular_velocity[2] + self.angular_velocity_offset[2],
            ]
            self.init_step += 1
        elif self.init_step == self.init_max and not self.init:
            self.imu_acc_offset = [
                self.imu_acc_offset[0] / self.init_max,
                self.imu_acc_offset[1] / self.init_max,
                self.imu_acc_offset[2] / self.init_max - 0.12,  # 0
            ]
            self.imu_ori_offset = [
                self.imu_ori_offset[0] / self.init_max,
                self.imu_ori_offset[1] / self.init_max,
                self.imu_ori_offset[2] / self.init_max + 0.3,  # 0.55
            ]
            self.angular_velocity_offset = [
                self.angular_velocity_offset[0] / self.init_max,
                self.angular_velocity_offset[1] / self.init_max,
                self.angular_velocity_offset[2] / self.init_max,
            ]
            self.imu_time_list = [time]
            self.init_step += 1  # init finished
        elif not self.init:
            self.imu_time = time
            self.linear_acceleration_imu = [
                linear_acceleration[0] - self.imu_acc_offset[0],
                linear_acceleration[1] - self.imu_acc_offset[1],
                linear_acceleration[2] - self.imu_acc_offset[2],
            ]
            self.orientation_imu = [
                orientation_imu_raw[0] - self.imu_ori_offset[0],
                orientation_imu_raw[1] - self.imu_ori_offset[1],
                orientation_imu_raw[2] - self.imu_ori_offset[2],
            ]  # roll pitch still wrong I think
            # clip self.orientation_imu to -pi to pi
            self.orientation_imu = [
                np.clip(self.orientation_imu[0], -np.pi, np.pi),
                np.clip(self.orientation_imu[1], -np.pi, np.pi),
                np.clip(self.orientation_imu[2], -np.pi, np.pi),
            ]
            self.angular_velocity_imu = [
                angular_velocity[0] - self.angular_velocity_offset[0],
                angular_velocity[1] - self.angular_velocity_offset[1],
                angular_velocity[2] - self.angular_velocity_offset[2],
            ]
            # self.elevation_dot_dot = self.linear_acceleration_imu[2]
            if self.imu_old_time == 0:  # first time
                self.imu_old_time = self.imu_time
            # transform self.acc to world frame with self.orientation_imu[2]
            self.acc = [
                self.linear_acceleration_imu[0] * np.cos(self.orientation_imu[2])
                - self.linear_acceleration_imu[1] * np.sin(self.orientation_imu[2]),
                self.linear_acceleration_imu[0] * np.sin(self.orientation_imu[2])
                + self.linear_acceleration_imu[1] * np.cos(self.orientation_imu[2]),
                self.linear_acceleration_imu[2],
            ]

            self.vel = [
                self.vel[0] + self.acc[0] * (self.imu_time - self.imu_old_time),
                self.vel[1] + self.acc[1] * (self.imu_time - self.imu_old_time),
                self.vel[2] + self.acc[2] * (self.imu_time - self.imu_old_time),
            ]
            self.imu_pos = [
                self.imu_pos[0] + self.vel[0] * (self.imu_time - self.imu_old_time),
                self.imu_pos[1] + self.vel[1] * (self.imu_time - self.imu_old_time),
                self.imu_pos[2] + self.vel[2] * (self.imu_time - self.imu_old_time),
            ]
            # self.yaw = self.yaw + self.angular_velocity_imu[2]*(self.imu_time - self.imu_old_time)
            self.imu_pos_list.append(self.imu_pos)

            # self.elevation_dot = self.elevation_dot + self.elevation_dot_dot * (self.imu_time - self.imu_old_time)
            # self.elevation = self.elevation + self.elevation_dot * (self.imu_time - self.imu_old_time)
            self.imu_old_time = self.imu_time
            self.imu_time_list.append(self.imu_time - self.imu_time_list[0])
            self.orientation_imu_list.append(self.orientation_imu)
            self.velocity_imu_list.append(self.vel)

    def track_cb(self, data):
        self.track_odom.header = Header()
        self.track_odom.header.stamp = self.imu_header_stamp
        self.track_odom.header.frame_id = "odom"
        self.track_odom.header.seq = self.seq
        left_trac_vel = data.left_track_velocity
        right_trac_vel = data.right_track_velocity
        if not self.init and self.imu_time_list:
            if self.track_old_time == 0:
                self.track_old_time = self.imu_time
            self.track_time_list.append(self.imu_time - self.imu_time_list[0])
            velocity = (left_trac_vel - right_trac_vel) / 2
            theta_dot = -(right_trac_vel + left_trac_vel) / 2 / self.track_width
            self.track_yaw = self.track_yaw + theta_dot * (
                self.imu_time - self.track_old_time
            )
            self.track_pos = [
                self.track_pos[0]
                + velocity
                * np.cos(self.track_yaw)
                * (self.imu_time - self.track_old_time),
                self.track_pos[1]
                + velocity
                * np.sin(self.track_yaw)
                * (self.imu_time - self.track_old_time),
                0,
            ]
            self.track_pos_list.append(self.track_pos)
            self.track_yaw_list.append(self.track_yaw)

            self.track_old_time = self.imu_time
            self.track_odom.pose.pose.position.x = self.track_pos[0]
            self.track_odom.pose.pose.position.y = self.track_pos[1]
            self.track_odom.pose.pose.position.z = self.track_pos[2]
            x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, self.track_yaw)
            self.track_odom.pose.pose.orientation.x = x
            self.track_odom.pose.pose.orientation.y = y
            self.track_odom.pose.pose.orientation.z = z
            self.track_odom.pose.pose.orientation.w = w
            self.track_odom.twist.twist.linear.x = velocity * np.cos(self.track_yaw)
            self.track_odom.twist.twist.linear.y = velocity * np.sin(self.track_yaw)
            self.track_odom.twist.twist.linear.z = 0
            self.track_odom.twist.twist.angular.x = 0
            self.track_odom.twist.twist.angular.y = 0
            self.track_odom.twist.twist.angular.z = theta_dot
            self.track_odom.pose.covariance = [
                0.01,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.01,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.01,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1,
            ]
            self.track_odom_pub.publish(self.track_odom)
        self.seq += 1

    def ekf_cb(self, data):
        x_pos = data.pose.pose.position.x
        y_pos = data.pose.pose.position.y
        self.ekf_pos_list.append([x_pos, y_pos])
        if self.imu_time_list and not self.init:
            self.ekf_error_list.append(
                [(x_pos - self.gt_pos_list[-1][0]) ** 2, (y_pos - self.gt_pos_list[-1][1]) ** 2]
            )
            self.ekf_cov_list.append([data.pose.covariance[0], data.pose.covariance[7]])
            self.ekf_time_list.append(self.imu_time - self.imu_time_list[0])


    def plot(self):
        self.imu_time_list.pop(0)
        self.imu_pos_list = np.array(self.imu_pos_list).T
        self.gt_pos_list = np.array(self.gt_pos_list).T
        self.track_pos_list = np.array(self.track_pos_list).T
        self.ekf_pos_list = np.array(self.ekf_pos_list).T
        self.orientation_imu_list = np.array(self.orientation_imu_list).T
        self.orientation_gt_list = np.array(self.orientation_gt_list).T
        self.velocity_gt_list = np.array(self.velocity_gt_list).T
        self.velocity_imu_list = np.array(self.velocity_imu_list).T
        self.gt_time_list = np.array(self.gt_time_list)
        self.imu_time_list = np.array(self.imu_time_list)
        self.track_time_list = np.array(self.track_time_list)
        self.ekf_error_list = np.array(self.ekf_error_list).T
        self.ekf_cov_list = np.array(self.ekf_cov_list).T
        self.ekf_cov_list = np.clip(self.ekf_cov_list, -1, 1)
        self.ekf_time_list = np.array(self.ekf_time_list)

        plt.plot(self.gt_time_list, self.gt_pos_list[2])
        plt.plot(self.imu_time_list, self.imu_pos_list[2])
        plt.xlabel("time (s)")
        plt.ylabel("elevation (m)")
        plt.title("Elevation vs time")
        plt.legend(["OptiTrack", "IMU"])
        plt.savefig("../figs/imu_elevation.png")
        plt.close()

        plt.plot(self.gt_pos_list[0], self.gt_pos_list[1])
        plt.plot(self.imu_pos_list[0], self.imu_pos_list[1])
        plt.plot(self.track_pos_list[0], self.track_pos_list[1])
        plt.plot(self.ekf_pos_list[0], self.ekf_pos_list[1])
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.title("path")
        plt.legend(["OptiTrack", "IMU", "Track", "EKF"])
        plt.savefig("../figs/path.png")
        plt.close()

        plt.plot(self.gt_pos_list[0], self.gt_pos_list[1])
        # plt.plot(self.imu_pos_list[0], self.imu_pos_list[1])
        # plt.plot(self.track_pos_list[0], self.trzack_pos_list[1])
        plt.plot(self.ekf_pos_list[0], self.ekf_pos_list[1])
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.title("path")
        plt.legend(["OptiTrack", "EKF"])
        plt.savefig("../figs/path_ekf.png")
        plt.close()

        plt.plot(self.gt_time_list, self.orientation_gt_list[2])
        plt.plot(self.imu_time_list, self.orientation_imu_list[2])
        plt.xlabel("time (s)")
        plt.ylabel("yaw (rad)")
        plt.title("Yaw vs time")
        plt.legend(["OptiTrack", "IMU"])
        plt.savefig("../figs/yaw.png")
        plt.close()

        plt.plot(self.gt_time_list, self.velocity_gt_list[0])
        plt.plot(self.gt_time_list, self.velocity_gt_list[1])
        plt.plot(self.imu_time_list, self.velocity_imu_list[0])
        plt.plot(self.imu_time_list, self.velocity_imu_list[1])
        plt.xlabel("time (s)")
        plt.ylabel("velocity (m/s)")
        plt.title("Velocity vs time")
        plt.legend(["OptiTrack x", "OptiTrack y", "IMU x", "IMU y"])
        plt.savefig("../figs/velocity.png")
        plt.close()

        plt.plot(self.ekf_time_list, self.ekf_error_list[0])
        plt.plot(self.ekf_time_list, self.ekf_error_list[1])
        plt.plot(self.ekf_time_list, self.ekf_cov_list[0])
        plt.plot(self.ekf_time_list, self.ekf_cov_list[1])
        print(max(self.ekf_error_list[0]))
        print(max(self.ekf_error_list[1]))
        plt.xlabel("time (s)")
        plt.ylabel("error (m^2)")
        plt.title("EKF error vs time")
        plt.legend(["x error^2", "y error^2", "x sd^2", "y sd^2"])
        plt.savefig("../figs/ekf_error.png")
        plt.close()

        plt.plot(self.gt_pos_list[0], self.gt_pos_list[1])
        plt.plot(self.imu_pos_list[0], self.imu_pos_list[1])
        plt.plot(self.track_pos_list[0], self.track_pos_list[1])
        # plt.plot(self.ekf_pos_list[0], self.ekf_pos_list[1])
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.title("path")
        plt.legend(["OptiTrack", "IMU", "Track"])
        plt.savefig("../figs/track_path.png")
        plt.close()
        



        # plt.show()


if __name__ == "__main__":
    rospy.init_node("se_node")
    state_estimator = state_estimator()
    rospy.spin()
    rospy.on_shutdown(state_estimator.plot())
