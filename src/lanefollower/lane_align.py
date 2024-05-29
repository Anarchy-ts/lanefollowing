#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf_transformations
import numpy as np
from geometry_msgs.msg import Point

class OdomAligner(Node):
    def __init__(self):
        super().__init__('aligner')
        self.lane_vec = None
        self.create_subscription(Point, '/flane/lanevec', self.vec_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.count = 0
        self.angular_velocity = 0.2  # Angular velocity to adjust orientation

    def vec_callback(self, msg):
        # print("HI1")
        if self.lane_vec is None:
            self.lane_vec = np.array([msg.x,msg.y])
            # print("HI2")
        else:
            self.create_subscription(Odometry, '/vidhyut/odom', self.odom_callback, 10)
            

    def odom_callback(self, msg):
        # self.create_subscription(Odometry, '/vidhyut/odom', self.vec_callback, 10)
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
        
        self.bot_head = np.array([np.cos(y),np.sin(y)])

        dot_product = np.dot(self.lane_vec, self.bot_head)
        cross_product = np.cross(np.append(self.lane_vec, 0), np.append(self.bot_head, 0))
        cross_product_z = cross_product[2]

        lane_vec_mag = np.linalg.norm(self.lane_vec)
        bot_head_mag = np.linalg.norm(self.bot_head)
        
        cosine = dot_product / (lane_vec_mag * bot_head_mag)
        angle = np.arccos(cosine)
        angle = np.rad2deg(angle)
        
        # print(cross_product_z)
        if angle>1:
            # print("HI")
            cmd_vel_msg = Twist()
            print(f"current angle : {angle}")
            if cross_product_z<0:
                cmd_vel_msg.angular.z = self.angular_velocity
            else:
                cmd_vel_msg.angular.z = -self.angular_velocity
            self.cmd_vel_pub.publish(cmd_vel_msg)
        else :
            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = 0.0
            
            self.cmd_vel_pub.publish(cmd_vel_msg)
            self.count = self.count+1
            if self.count == 1:
                print(f"current angle : {angle}")
                self.get_logger().info("Aligned to Lanes")
            if self.count == 1000:
                exit(0)
       

def main(args=None):
    rclpy.init(args=args)
    odom_aligner = OdomAligner()
    rclpy.spin(odom_aligner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
