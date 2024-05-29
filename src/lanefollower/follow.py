#!/usr/bin/python3
import subprocess
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import tf_transformations
import numpy as np

def run_background_command(command):
    process = subprocess.Popen(command, shell=True)
    return process

def run_command(command):
    process = subprocess.Popen(command, shell=True)
    process.wait()
    return process.returncode

def calculate_angle(vec1, vec2):
    dot_product = np.dot(vec1, vec2)
    cross_product = np.cross(np.append(vec1, 0), np.append(vec2, 0))
    cross_product_z = cross_product[2]

    lane_vec_mag = np.linalg.norm(vec1)
    bot_head_mag = np.linalg.norm(vec2)
        
    cosine = dot_product / (lane_vec_mag * bot_head_mag)
    angle = np.arccos(cosine)
    angle = np.rad2deg(angle)
    return angle

class LaneFollowingNode(Node):
    def __init__(self):
        super().__init__('lane_following_node')
        self.odom_subscriber = self.create_subscription(Odometry, '/vidhyut/odom', self.odom_callback, 10)
        self.lanevec_subscriber = self.create_subscription(Point, '/flane/lanevec', self.lanevec_callback, 10)
        self.bot_head = None
        self.lane_vec = None
        self.booalign = False
        self.boofollow =  False

    def odom_callback(self, msg):

        odom_ang_x = msg.pose.pose.orientation.x
        odom_ang_y = msg.pose.pose.orientation.y
        odom_ang_z = msg.pose.pose.orientation.z
        odom_w = msg.pose.pose.orientation.w
        quaternion = [odom_ang_x, odom_ang_y, odom_ang_z, odom_w]
        # q = tf_transformations.quaternion_from_euler(r, p, y)
        r, p, y = tf_transformations.euler_from_quaternion(quaternion)
        self.bot_head = np.array([np.cos(y),np.sin(y)])
        self.check_and_run()

    def lanevec_callback(self, msg):
        self.lane_vec = np.array([msg.x,msg.y])
        self.check_and_run()

    def check_and_run(self):
        if self.bot_head is not None and self.lane_vec is not None:
            angle = calculate_angle(self.bot_head, self.lane_vec)
            print(f"Angle : {angle}")
            if angle > 2:
                self.get_logger().info(f'Angle {angle} > 1 degrees, running lane_align.py')
                run_command("ros2 run lanefollowing lane_align.py")
                print("HI")
            else:
                self.get_logger().info(f'Angle {angle} <= 1 degrees, running lanegoal.py')
                run_command("ros2 run lanefollowing lanegoal.py")

def main(args=None):
    rclpy.init(args=args)
    
    node = LaneFollowingNode()
    # Initial ROS 2 commands that run indefinitely
    background_commands = [
        "ros2 run lanefollowing midpoint",
        "ros2 run lanefollowing mid_ipm.py",
    ]

    # Run background commands
    background_processes = []
    for cmd in background_commands:
        print(f"Running background command: {cmd}")
        process = run_background_command(cmd)
        background_processes.append(process)
        time.sleep(5)  # Add a small delay to ensure the command starts properly
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
