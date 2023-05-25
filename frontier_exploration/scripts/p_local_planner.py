#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PD_local_planner:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/cmd_vel/teleopDrive", Twist, queue_size=1)
        self.path_sub = rospy.Subscriber(
            "global_path", Path, self.path_callback, queue_size=1
        )
        self.curr_sub = rospy.Subscriber(
            "/optitrack/vrpn_client_node/mce234b_bot/pose",
            PoseStamped,
            self.curr_callback,
            queue_size=1,
        )
        self.goal_sub = rospy.Subscriber(
            "/mce234b/goal_pose_published",
            PoseStamped,
            self.goal_callback,
            queue_size=1,
        )
        self.map_sub = rospy.Subscriber(
            "map", OccupancyGrid, self.map_callback, queue_size=1
        )
        self.path = Path()
        self.curr = PoseStamped()
        self.goal = PoseStamped()
        self.map = OccupancyGrid()
        self.has_path = False
        self.has_curr = False
        self.has_goal = False
        self.has_map = False
        self.kp = 1
        self.kd = 1
        self.kp_ang = 1
        self.kd_ang = 1
        self.max_vel = 1
        self.max_ang_vel = 1
        self.max_acc = 1
        self.max_ang_acc = 1
        self.cmd = Twist()

    def path_callback(self, msg):
        self.path = msg.poses
        self.has_path = True

    def curr_callback(self, msg):
        self.curr = msg
        self.has_curr = True
        if self.has_goal and self.has_map and self.has_path:
            self.getUProportional(0.1, 0.1, 0.1, 0.1)
            self.cmd_pub.publish(self.cmd)

    def goal_callback(self, msg):
        self.goal = msg
        self.has_goal = True

    def map_callback(self, msg):
        self.map = msg
        self.has_map = True

    def get_next_waypoint(self):
        if self.has_goal:
            if len(self.path) > 1:
                return self.path[1]
            else:
                return self.goal

    def getUProportional(
        self,
        xy_tol,
        theta_tol,
        xy_err_l2,
        theta_err_abs,
        K_p=-0.9,
        K_h=-0.9,
        debug=False,
    ):
        """
        vecx_0 = np.array([x_0, y_0, theta_0])
        vecx_f = np.array([x_f, y_f, theta_f])
        xy_tol
        theta_tol
        xy_err_l2
        theta_err_abs

        Returns u = np.array([V, omega])
        """

        x_0, y_0, theta_0 = (
            self.curr.pose.position.x,
            self.curr.pose.position.y,
            euler_from_quaternion(
                [
                    self.curr.pose.orientation.x,
                    self.curr.pose.orientation.y,
                    self.curr.pose.orientation.z,
                    self.curr.pose.orientation.w,
                ]
            )[2],
        )
        nextw = self.get_next_waypoint()
        x_f, y_f, theta_f = (
            nextw.pose.position.x,
            nextw.pose.position.y,
            euler_from_quaternion(
                [
                    nextw.pose.orientation.x,
                    nextw.pose.orientation.y,
                    nextw.pose.orientation.z,
                    nextw.pose.orientation.w,
                ]
            )[2],
        )
        V = 0
        omega = 0
        # Are we within tolerance of position?
        if xy_err_l2 < xy_tol:
            # Yes: adjust heading
            # Are we within tolerance of heading?
            if debug:
                print("\t\t Within pos tol")
            if theta_err_abs < theta_tol:
                # Yes: do nothing
                pass
            else:
                # No: fix heading to match heading target
                omega = K_h * (theta_0 - theta_f)
        else:
            # No: adjust position
            # Compute the heading needed to point to target xy
            if debug:
                print("\t\t Not within pos tol")
            delta_y = y_0 - y_f
            delta_x = x_0 - x_f
            theta_p = np.arctan(delta_y / delta_x)

            # Are we pointing towards the goal?
            delta_theta_p = theta_0 - theta_p
            if debug:
                print("\t\t heading err:", delta_theta_p, "of ", theta_tol)
            if abs(delta_theta_p) < theta_tol:
                # Yes: move towards target
                if debug:
                    print("\t\t Within heading tol -> make V")
                L = np.sqrt(delta_x**2 + delta_y**2) * np.sign(delta_y / delta_x)
                V = -K_p * L
            else:
                # No: adjust heading to point towards target
                if debug:
                    print("\t\t Not in heading tol -> make omega_p")
                omega = K_h * (theta_0 - theta_p)

        u = np.array([V, omega])
        self.cmd.linear.x = u[0]
        self.cmd.angular.z = u[1]
        
    
    
