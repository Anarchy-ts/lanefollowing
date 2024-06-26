#!/usr/bin/python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from robot_navigator import BasicNavigator, NavigationResult

'''
Basic navigation demo to go to pose.
'''
global flag
flag = True
class NavigatorDemo:
    def __init__(self):
        self.navigator = BasicNavigator()
        self.odom_subscriber = self.navigator.create_subscription(Odometry, '/vidhyut/odom', self.odom_callback, 10)
        self.current_x = None
        self.current_y = None
        self.ang_z = None
        # self.flag = True

    def odom_callback(self, msg):
        global flag
        if flag :
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            self.ang_x = msg.pose.pose.orientation.x
            self.ang_y = msg.pose.pose.orientation.y
            self.ang_z = msg.pose.pose.orientation.z
            flag = False

    def run(self):
        self.navigator.waitUntilNav2Active()

        # while rclpy.ok() and self.current_x is None:
        #     rclpy.spin_once(self.navigator)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'odom'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        if abs(self.ang_z) < 0.4 : 
            goal_pose.pose.position.x = self.current_x + 3.0
            goal_pose.pose.position.y = self.current_y + 0.0
            goal_pose.pose.orientation.x = self.ang_x
            goal_pose.pose.orientation.y = self.ang_y
            goal_pose.pose.orientation.z = self.ang_z
            goal_pose.pose.orientation.w = 1.0
            # goal_pose.pose.orientation.z
        else : 
            goal_pose.pose.position.x = self.current_x + 0.0
            goal_pose.pose.position.y = self.current_y + 3.0
            goal_pose.pose.orientation.x = self.ang_x
            goal_pose.pose.orientation.y = self.ang_y
            goal_pose.pose.orientation.z = self.ang_z
            goal_pose.pose.orientation.w = 1.0

        self.navigator.goToPose(goal_pose)

        while rclpy.ok() and not self.navigator.isNavComplete():
            rclpy.spin_once(self.navigator)

        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        # self.navigator.lifecycleShutdown()
        exit(0)

def main():
    rclpy.init()
    demo = NavigatorDemo()
    demo.run()

if __name__ == '__main__':
    main()
