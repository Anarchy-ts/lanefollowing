#!/usr/bin/python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter')
        self.subscription = self.create_subscription(
            Image,
            '/zed2i/zed_node/image_raw',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Image, '/gray_image', 10)
        self.binary_publisher = self.create_publisher(Image, '/binary_image', 10)
        self.midpoint_publisher = self.create_publisher(Image, '/middlelane', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to a NumPy array
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert the image to grayscale
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Thresholding: Binarize the image
            # _, binary_image = cv2.threshold(gray_image, 120, 140, cv2.THRESH_BINARY)
            binary_image = cv2.inRange(gray_image, 120, 140)
            
            # Find contours in the binary image
            contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Extract midpoints of each lane
            midpoints = []
            for contour in contours:
                moments = cv2.moments(contour)
                if moments["m00"] != 0:
                    cx = int(moments["m10"] / moments["m00"])
                    cy = int(moments["m01"] / moments["m00"])
                    midpoints.append((cx, cy))
                    print(f"Lane midpoint: ({cx}, {cy})")

            # Create a new image with a white lane between the midpoints
            midpoint_image = binary_image.copy()
            for midpoint in midpoints:
                cv2.circle(midpoint_image, midpoint, 3, 255, -1)

            # Convert the binary image back to a ROS Image message
            binary_msg = self.bridge.cv2_to_imgmsg(binary_image, encoding='mono8')
            midpoint_msg = self.bridge.cv2_to_imgmsg(midpoint_image, encoding='mono8')

            # Publish the grayscale and binary images
            self.publisher.publish(msg)
            self.binary_publisher.publish(binary_msg)
            self.midpoint_publisher.publish(midpoint_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)
    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
