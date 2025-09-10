import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# This is a simple line follower node that processes images 
# from a camera and publishes velocity commands to follow a line.

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        self.bridge = CvBridge()
        # Subscriber to camera
        self.subscription = self.create_subscription(
            Image,
            '/camera_image',  # Make sure this matches your bridge remap
            self.image_callback,
            10
        )

        # Publisher to cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Crop the bottom part of the image
        height, width, _ = cv_image.shape
        crop_height = 60
        crop_img = cv_image[height-crop_height:height, 0:width]

        # Convert to grayscale and threshold
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        twist = Twist()
        if contours:
            # Find the largest contour (assume it's the line)
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                # Calculate error from center
                error = cx - width // 2
                # Proportional controller for steering
                twist.linear.x = 0.15
                twist.angular.z = -float(error) / 100.0
            else:
                # If no centroid, stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        else:
            # If no line detected, stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
