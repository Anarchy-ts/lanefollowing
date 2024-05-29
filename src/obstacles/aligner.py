#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf_transformations
import numpy as np

class OdomAligner(Node):
    def __init__(self):
        super().__init__('aligner')

        self.initial_orientation = 0.0
        self.vidhyut_orientation = 0.0

        self.subscription_initial = self.create_subscription(
            Odometry,
            '/initial/odom',
            self.initial_odom_callback,
            10)
        self.subscription_vidhyut = self.create_subscription(
            Odometry,
            '/vidhyut/odom',
            self.vidhyut_odom_callback,
            10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.stop_threshold = 5  # Threshold to consider orientations equal
        self.angular_velocity = 0.2  # Angular velocity to adjust orientation

    def initial_odom_callback(self, msg):
        odom_ang_x = msg.pose.pose.orientation.x
        odom_ang_y = msg.pose.pose.orientation.y
        odom_ang_z = msg.pose.pose.orientation.z
        odom_w = msg.pose.pose.orientation.w
        quaternion = [odom_ang_x, odom_ang_y, odom_ang_z, odom_w]
        # q = tf_transformations.quaternion_from_euler(r, p, y)
        r, p, y = tf_transformations.euler_from_quaternion(quaternion)
        self.roll = np.rad2deg(r)
        self.pitch = np.rad2deg(p)
        self.yaw = np.rad2deg(y)
        self.initial_orientation = self.yaw

    def vidhyut_odom_callback(self, msg):
        odom_ang_x = msg.pose.pose.orientation.x
        odom_ang_y = msg.pose.pose.orientation.y
        odom_ang_z = msg.pose.pose.orientation.z
        odom_w= msg.pose.pose.orientation.w
        quaternion = [odom_ang_x, odom_ang_y, odom_ang_z, odom_w]
        # q = tf_transformations.quaternion_from_euler(r, p, y)
        r, p, y = tf_transformations.euler_from_quaternion(quaternion)
        roll = np.rad2deg(r)
        pitch = np.rad2deg(p)
        yaw = np.rad2deg(y)
        self.vidhyut_orientation = yaw
        if abs(self.initial_orientation - self.vidhyut_orientation) > self.stop_threshold:
            # print("HI")
            cmd_vel_msg = Twist()
            print(f"current : {self.vidhyut_orientation} & initial : {self.initial_orientation}")
            cmd_vel_msg.angular.z = self.angular_velocity
            self.cmd_vel_pub.publish(cmd_vel_msg)
        else :
            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel_msg)
            exit(0)
       

def main(args=None):
    rclpy.init(args=args)
    odom_aligner = OdomAligner()
    rclpy.spin(odom_aligner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
