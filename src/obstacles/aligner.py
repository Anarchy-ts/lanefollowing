#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

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

        self.stop_threshold = 0.1  # Threshold to consider orientations equal
        self.angular_velocity = 0.5  # Angular velocity to adjust orientation

    def initial_odom_callback(self, msg):
        self.initial_orientation = msg.pose.pose.orientation.z

    def vidhyut_odom_callback(self, msg):
        self.vidhyut_orientation = msg.pose.pose.orientation.z
        if abs(self.initial_orientation - self.vidhyut_orientation) > self.stop_threshold:
            # print("HI")
            cmd_vel_msg = Twist()
            print(f"current : {self.vidhyut_orientation} & initial : {self.initial_orientation}")
            cmd_vel_msg.angular.z = self.angular_velocity
            self.cmd_vel_pub.publish(cmd_vel_msg)
        else :
            exit(0)
       

def main(args=None):
    rclpy.init(args=args)
    odom_aligner = OdomAligner()
    rclpy.spin(odom_aligner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
