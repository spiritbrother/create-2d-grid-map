import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        self.cv_bridge = CvBridge()

        # Subscriptions to camera topics
        self.subscriber_cam2 = self.create_subscription(
            Image,
            '/overhead_camera/overhead_camera2/image_raw',
            self.callback_cam2,
            10)
        
        self.subscriber_cam4 = self.create_subscription(
            Image,
            '/overhead_camera/overhead_camera4/image_raw',
            self.callback_cam4,
            10)

        # Publisher to send velocity commands
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variables to store images
        self.image_cam2 = None  # Bottom left (camera 2)
        self.image_cam4 = None  # Top left (camera 4)

        # Timer to periodically process images and control robot
        self.control_timer = self.create_timer(1.0, self.control_robot)

        # Initialize variables for controlling robot
        self.map_image = None

    def callback_cam2(self, msg):
        self.image_cam2 = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

    def callback_cam4(self, msg):
        self.image_cam4 = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

    def control_robot(self):
        if self.image_cam2 is not None and self.image_cam4 is not None:
            self.merge_images()
            self.navigate_wall()

    def merge_images(self):
        # Get individual image shapes
        h2, w2, _ = self.image_cam2.shape
        h4, w4, _ = self.image_cam4.shape

        # Determine the total dimensions for the combined image
        total_height = h2 + h4
        total_width = max(w2, w4)

        # Initialize the combined image with a black background
        combined_img = np.zeros((total_height, total_width, 3), dtype=np.uint8)

        # Paste images into the combined image with appropriate shifts
        combined_img[0:h4, 0:w4] = self.image_cam4  # Top-left (image 4)
        combined_img[h4:h4+h2, 0:w2] = self.image_cam2  # Bottom-left (image 2)

        # Convert to grayscale
        grayscale_img = cv2.cvtColor(combined_img, cv2.COLOR_BGR2GRAY)

        # Thresholding to binary image
        _, binary_img = cv2.threshold(grayscale_img, 128, 255, cv2.THRESH_BINARY)

        # Create or initialize the map image with white background
        if self.map_image is None or self.map_image.shape != grayscale_img.shape:
            self.map_image = np.ones_like(grayscale_img) * 255  # White background

        # Use binary image to highlight walls in the map image
        self.map_image[binary_img == 0] = 0  # Black pixels for detected walls

        # Save the map image periodically (e.g., every 10 seconds)
        if int(time.time()) % 10 == 0:
            cv2.imwrite('map.pgm', self.map_image)
            self.get_logger().info('Map saved as map.pgm')

    def navigate_wall(self):
        # Example wall-following logic
        # Adjust linear and angular velocity based on sensor readings
        velocity_cmd = Twist()

        # Example logic to turn left if the robot is too close to the wall
        if self.image_cam4 is not None:
            left_wall_dist = np.mean(self.image_cam4[:, -1])  # Simplified example
            if left_wall_dist < 100:  # Distance threshold
                velocity_cmd.angular.z = 0.3  # Turn right
            else:
                velocity_cmd.angular.z = 0.0  # Go straight

        velocity_cmd.linear.x = 0.2  # Move forward

        # Publish velocity commands
        self.velocity_publisher.publish(velocity_cmd)

        self.get_logger().info(f'Published velocity command: Linear x: {velocity_cmd.linear.x}, Angular z: {velocity_cmd.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
