import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from image_processing import image_segment, image_centroid_horizontal
import cv2
import numpy as np
import os


class ImageSegmenter(Node):
    def __init__(self):
        super().__init__('image_segmenter')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Define color thresholds (adjust these values based on your line color)
        self.threshold_low = (0, 40, 0)    # BGR lower bound (for green line)
        self.threshold_high = (110, 255, 90)  # BGR upper bound
        
        # Publishers
        self.seg_pub = self.create_publisher(Image, '/image/segmented', 10)
        self.centroid_pub = self.create_publisher(
            PointStamped, '/image/centroid', 10)
        
        # Subscriber
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        self.get_logger().info("Line segmentation node initialized")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 1. Segment the image
            segmented = image_segment(cv_image, self.threshold_low, self.threshold_high)
            
            # 2. Publish segmented image
            seg_msg = self.bridge.cv2_to_imgmsg(segmented, encoding='mono8')
            seg_msg.header = msg.header
            self.seg_pub.publish(seg_msg)
            
            # 3. Compute and publish centroid
            centroid_x = image_centroid_horizontal(segmented)
            
            centroid_msg = PointStamped()
            centroid_msg.header.stamp = self.get_clock().now().to_msg()
            centroid_msg.header.frame_id = msg.header.frame_id
            centroid_msg.point.x = float(centroid_x)
            centroid_msg.point.y = 0.0  # Vertical center (assuming horizontal line)
            centroid_msg.point.z = 0.0  # No depth information
            
            self.centroid_pub.publish(centroid_msg)
            
            self.get_logger().debug(
                f"Published centroid at x={centroid_x}",
                throttle_duration_sec=1)
                
        except Exception as e:
            self.get_logger().error(f"Image processing failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSegmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
