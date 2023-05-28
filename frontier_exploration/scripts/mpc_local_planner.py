#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from MPC import getMPC

class PD_local_planner:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/cmd_vel/teleopDrive", Twist, queue_size=1)
        self.path_sub = rospy.Subscriber(
            "global_path", Path, self.path_callback, queue_size=1
        )
        self.curr_goal_pub = rospy.Publisher("curr_goal", PoseStamped, queue_size=1)
        self.curr_sub = rospy.Subscriber(
            "/optitrack/vrpn_client_node/mce234b_bot/pose",
            PoseStamped,
            self.curr_callback,
            queue_size=1,
        )
        self.curr_sub_sim = rospy.Subscriber(
            "/base_pose_ground_truth", Odometry, self.curr_callback_sim, queue_size=1
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
        self.u = np.array([0, 0])

    def path_callback(self, msg):
        self.path = msg.poses
        self.has_path = True

    def curr_callback(self, msg):
        self.curr = msg
        self.has_curr = True
        if self.has_goal and self.has_map and self.has_path:
            self.getUProportional(0.1, 0.1, 0.1, 0.1)
            self.cmd_pub.publish(self.cmd)

    def curr_callback_sim(self, msg):
        # set curr
        self.curr.pose = msg.pose.pose
        self.curr.header.frame_id = "map"
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
            for p in range(len(self.path)):
                dist_to_robot = np.sqrt(pow(self.curr.pose.position.x - self.path[p].pose.position.x, 2) + pow(self.curr.pose.position.y - self.path[p].pose.position.y, 2))
                if dist_to_robot>0.3:
                    return self.path[p]
            else:
                return self.goal

    def getUProportional(
        self,
        xy_tol,
        theta_tol,
        xy_err_l2,
        theta_err_abs,
        K_p=-0.5,
        K_h=-0.5,
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
        self.curr_goal_pub.publish(nextw)
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
        vecx_0 = np.array([x_0, y_0, theta_0])
        vecx_f = np.array([x_f, y_f, theta_f])
        self.u = getMPC(vecx_0=vecx_0,vecx_ref=vecx_f,N_h=20,N_mpc=6)
        self.cmd.linear.x = self.u[0]
        self.cmd.angular.z = self.u[1]
        if np.allclose(np.array([0, 0]), np.array([self.curr.pose.position.x-self.goal.pose.position.x, self.curr.pose.position.y-self.goal.pose.position.y]), atol=0.05):
            self.cmd.linear.x = 0
            self.cmd.angular.z = 0
        


if __name__ == "__main__":
    rospy.init_node("pd_controller")
    pd = PD_local_planner()
    rospy.spin()
