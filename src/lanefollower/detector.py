#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from sklearn.cluster import DBSCAN
import numpy as np

class ImageConverter(Node):
    def __init__(self):
        super().__init__('detector')

        # Subscribe to the RGB image topic
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.process_image,
            10)

        # Publish the grayscale image
        self.publisher_ = self.create_publisher(Image, '/igvc/lanes_binary', 10)

    def process_image(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Convert RGB image to grayscale
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            binary_image = cv2.inRange(gray_image, 125, 140)
            binary_image[:binary_image.shape[0]//2 + 100, :] = 0

            # Find the indices of white pixels
            white_pixel_indices = np.argwhere(binary_image == 255)

            # Run DBSCAN on white pixel indices
            db = DBSCAN(eps=1, min_samples=5).fit(white_pixel_indices)
            labels = db.labels_
            no_clusters = len(np.unique(labels))
            no_noise = np.sum(labels == -1)

            print('Estimated no. of clusters: %d' % no_clusters)
            print('Estimated no. of noise points: %d' % no_noise)

            # Determine the size of each cluster
            cluster_sizes = [np.sum(labels == label) for label in np.unique(labels) if label != -1]

            # Keep the indices of the largest two clusters
            largest_clusters_indices = np.argsort(cluster_sizes)[-2:]

            # Create a blank image to draw clusters
            clustered_image = np.zeros_like(cv_image)

            for label in np.unique(labels):
                
                if label == -1 or label not in largest_clusters_indices:
                    continue
                
                cluster_indices = white_pixel_indices[labels == label]
                for point in cluster_indices:
                    clustered_image[point[0], point[1]] = 255

            clustered_msg = bridge.cv2_to_imgmsg(clustered_image, encoding='rgb8')


            # Publish the clustered image
            self.publisher_.publish(clustered_msg)
        except Exception as e:
            self.get_logger().error('cv_bridge exception: %s' % e)

def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)
    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
