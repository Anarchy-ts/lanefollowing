#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav_msgs.msg import Odometry
from robot_navigator import BasicNavigator, NavigationResult
import tf_transformations
import numpy as np
import time

# Global variables
global pos, modo, modoflag
pos = Point()
modo = Odometry()
modoflag = False

class Subscriber(Node):
    def __init__(self):
        super().__init__('lf')
        self.create_subscription(Odometry, '/vidhyut/odom', self.odom, 10)

    def odom(self, msg):
        global modo, modoflag
        if modoflag == False:
            modo = msg
            modoflag = True
            self.create_subscription(Point, '/flane/lanegoal/start', self.goal, 10)

    def goal(self, msg):
        global pos
        pos = msg
        LaneFollower().run()

class LaneFollower:
    def __init__(self):
        self.navigator = BasicNavigator()

    def run(self):
        global pos, modo, modoflag
        self.navigator.waitUntilNav2Active()

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'odom'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        self.goal_pose.pose.position.x = pos.x 
        self.goal_pose.pose.position.y = pos.y
        self.goal_pose.pose.position.z = pos.z
        self.goal_pose.pose.orientation.x = modo.pose.pose.orientation.x
        self.goal_pose.pose.orientation.y = modo.pose.pose.orientation.y
        self.goal_pose.pose.orientation.z = modo.pose.pose.orientation.z
        self.goal_pose.pose.orientation.w = modo.pose.pose.orientation.w

        self.navigator.goToPose(self.goal_pose)

        while rclpy.ok() and not self.navigator.isNavComplete():
            rclpy.spin_once(self.navigator)

        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('Goal succeeded!\nAligner will start')
            modoflag = False
            time.sleep(2)
            Aligner().run()
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

class Aligner(Node):
    def __init__(self):
        super().__init__('aligner')
        self.lane_vec = None
        self.create_subscription(Point, '/flane/lanevec', self.vec_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/vidhyut/cmd_vel', 10)
        self.count = 0
        self.angular_velocity = 0.2  # Angular velocity to adjust orientation

    def vec_callback(self, msg):
        if self.lane_vec is None:
            self.lane_vec = np.array([msg.x, msg.y])
        else:
            self.create_subscription(Odometry, '/vidhyut/odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        odom_ang_x = msg.pose.pose.orientation.x
        odom_ang_y = msg.pose.pose.orientation.y
        odom_ang_z = msg.pose.pose.orientation.z
        odom_w = msg.pose.pose.orientation.w
        quaternion = [odom_ang_x, odom_ang_y, odom_ang_z, odom_w]
        r, p, y = tf_transformations.euler_from_quaternion(quaternion)
        yaw = np.rad2deg(y)

        self.bot_head = np.array([np.cos(y), np.sin(y)])

        dot_product = np.dot(self.lane_vec, self.bot_head)
        cross_product = np.cross(np.append(self.lane_vec, 0), np.append(self.bot_head, 0))
        cross_product_z = cross_product[2]

        lane_vec_mag = np.linalg.norm(self.lane_vec)
        bot_head_mag = np.linalg.norm(self.bot_head)

        cosine = dot_product / (lane_vec_mag * bot_head_mag)
        angle = np.arccos(cosine)
        angle = np.rad2deg(angle)

        if angle > 1:
            cmd_vel_msg = Twist()
            print(f"Current angle: {angle}")
            if cross_product_z < 0:
                cmd_vel_msg.angular.z = self.angular_velocity
            else:
                cmd_vel_msg.angular.z = -self.angular_velocity
            self.cmd_vel_pub.publish(cmd_vel_msg)
        else:
            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel_msg)
            self.count += 1
            if self.count == 1:
                print(f"Current angle: {angle}")
                self.get_logger().info("Aligned to Lanes")
            if self.count == 1000:
                print("Switching back to LaneFollower")
                Subscriber()
                rclpy.spin(Subscriber())
                self.destroy_node()

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    flane = Subscriber()
    rclpy.spin(flane)
    flane.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
