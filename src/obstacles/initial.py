#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

global initial
initial = None
class InitialOdometryPublisher(Node):

    def __init__(self):
        super().__init__('initial_angz')
        self.subscription = self.create_subscription(
            Odometry,
            '/vidhyut/odom',  # Change 'input_odometry_topic' to your odometry input topic name
            self.odometry_callback,
            10)
        self.publisher_ = self.create_publisher(
            Odometry,
            '/initial/odom',  # Change 'output_odometry_topic' to your odometry output topic name
            10)

    def odometry_callback(self, msg):
        global initial
        if initial == None : 
            initial = msg
    
        # # Modify the received odometry message
        # modified_odometry = Odometry()
        # modified_odometry.header = msg.header
        # modified_odometry.child_frame_id = msg.child_frame_id
        # modified_odometry.pose.pose.position.x = 0.0
        # modified_odometry.pose.pose.position.y = 0.0
        # modified_odometry.pose.pose.position.z = 0.0
        # modified_odometry.pose.pose.orientation.x = 0.0
        # modified_odometry.pose.pose.orientation.y = 0.0
        # modified_odometry.pose.pose.orientation.z = 0.0
        # modified_odometry.pose.pose.orientation.w = 1.0
        # modified_odometry.twist.twist.linear.x = 0.0
        # modified_odometry.twist.twist.linear.y = 0.0
        # modified_odometry.twist.twist.linear.z = 0.0
        # modified_odometry.twist.twist.angular.x = 0.0
        # modified_odometry.twist.twist.angular.y = 0.0
        # modified_odometry.twist.twist.angular.z = 0.0

        self.publisher_.publish(initial)

def main(args=None):
    rclpy.init(args=args)
    node = InitialOdometryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
