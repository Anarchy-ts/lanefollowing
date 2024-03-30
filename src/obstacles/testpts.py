#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np

class PointCloudSubscriber(Node):

    def __init__(self):
        super().__init__('pointcloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/zed2i/zed_node/points',  # Change this to your actual topic name
            self.pointcloud_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.ground_height = 0.0  # Adjust this value based on your ground plane height
        self.closest_point = None
        self.min_distance = float('inf')

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received a pointcloud message')

        # Parse point cloud data
        point_step = int(msg.point_step)  # Convert point_step to integer
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, point_step // 4)

        for i in range(points.shape[0]):
            x = points[i][0]
            y = points[i][1]
            z = points[i][2]

            # Filter points above the ground height + 20 cm
            if z > self.ground_height + 0.20 and x<10 and y<10 and z<10:
                distance = np.sqrt(x**2 + y**2 + z**2)
                if distance < self.min_distance:
                    self.min_distance = distance
                    self.closest_point = (x, y, z)

        if self.closest_point is not None:
            x, y, z = self.closest_point
            self.get_logger().info('Closest point at x=%f, y=%f, z=%f, distance=%f' % (x, y, z, self.min_distance))
        else:
            self.get_logger().info('No points found above the ground.')

def main(args=None):
    rclpy.init(args=args)
    pointcloud_subscriber = PointCloudSubscriber()
    rclpy.spin(pointcloud_subscriber)
    pointcloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
