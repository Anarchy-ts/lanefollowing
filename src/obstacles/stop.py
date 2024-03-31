#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class LaneFollowingAvoidance(Node):

    def __init__(self):
        super().__init__('lane_stop')
        self.publisher_ = self.create_publisher(Twist, '/lane/stop', 10)
        self.twist_msg = Twist()

    def publish_zero_twist(self):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0
        self.publisher_.publish(self.twist_msg)
        # self.get_logger().info("Published zero twist message.")

def main(args=None):
    rclpy.init(args=args)

    lane_following_avoidance = LaneFollowingAvoidance()

    while rclpy.ok():
        lane_following_avoidance.publish_zero_twist()

    rclpy.shutdown()

if __name__ == '__main__':
    main()



# #!/usr/bin/python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool
# from geometry_msgs.msg import Twist
# import time

# class LaneFollowingAvoidance(Node):

#     def __init__(self):
#         super().__init__('lane_following_avoidance_node')
#         self.subscription = self.create_subscription(
#             Bool,
#             '/lanefollowing/avoid',
#             self.avoid_callback,
#             10)
#         self.publisher_ = self.create_publisher(Twist, '/lane/stop', 10)
#         self.twist_msg = Twist()

#     def avoid_callback(self, msg):
#         msg
#         # if msg.data:  # If message is True
#         self.twist_msg.linear.x = 0.0
#         self.twist_msg.linear.x = 0.0
#         self.twist_msg.linear.y = 0.0
#         self.twist_msg.linear.z = 0.0
#         self.twist_msg.angular.x = 0.0
#         self.twist_msg.angular.y = 0.0
#         self.twist_msg.angular.z = 0.0
#         self.publisher_.publish(self.twist_msg)
#         # self.get_logger().info("Avoidance triggered. Stopping the robot.")
#         # time.sleep(2)  # Wait for 2 seconds
#         # rclpy.shutdown()  # Shut down the node after publishing for 2 seconds

# def main(args=None):
#     rclpy.init(args=args)

#     lane_following_avoidance = LaneFollowingAvoidance()

#     rclpy.spin(lane_following_avoidance)

# if __name__ == '__main__':
#     main()
