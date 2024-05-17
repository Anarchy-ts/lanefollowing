#!/usr/bin/python3
import rclpy
from geometry_msgs.msg import PoseStamped
from robot_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import Point
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time

global pos, modo, modoflag
pos = Point()
modo = Odometry()
modoflag = False

class Subscriber(Node):
    def __init__(self):
        super().__init__('lf')
        self.create_subscription(Odometry, '/vidhyut/odom', self.odom, 10)

    def odom(self,msg):
        global modo,modoflag
        if modoflag == False:
            modo = msg
            modoflag = True
            self.create_subscription(Point, '/flane/lanegoal', self.goal, 10)

    def goal(self, msg):
        global pos
        pos = msg
        
        LaneFollower().run()

class LaneFollower:
    def __init__(self):
        self.navigator = BasicNavigator()

    def run(self):
        global pos,modo,modoflag
        self.navigator.waitUntilNav2Active()

        # while rclpy.ok() and self.current_x is None:
        #     rclpy.spin_once(self.navigator)

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
            print('Goal succeeded!\n Taking new goal!')
            modoflag = False
            time.sleep(2)
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        
        # self.navigator.lifecycleShutdown()
        # exit(0)

def main(args=None):
    rclpy.init(args=args)
    flane = Subscriber()
    rclpy.spin(flane)
    flane.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
