#!/usr/bin/python3
import rclpy
from geometry_msgs.msg import PoseStamped
from robot_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import Point

class NavigatorDemo:
    def __init__(self):
        self.navigator = BasicNavigator()
        self.odom_subscriber = self.navigator.create_subscription(Point, '/flane/lanegoal', self.odom_callback, 10)
  
    def odom_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.ang_x = 0.0
        self.ang_y = 0.0
        self.ang_z = 0.0

    def run(self):
        self.navigator.waitUntilNav2Active()

        # while rclpy.ok() and self.current_x is None:
        #     rclpy.spin_once(self.navigator)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'odom'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
  
        goal_pose.pose.position.x = self.x 
        goal_pose.pose.position.y = self.y
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
