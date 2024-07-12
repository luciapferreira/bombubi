import cv2

# ROS2 package modules and libraries
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node 
from cv_bridge import CvBridge 

# To make sure the file can be execute:
# chmod +x cameraSubscriber.py

# To run subscriber:
# ros2 run camera_subscriber subscriber_node

class SubscriberNodeClass(Node):

    # Constructor
    def __init__(self):
        super().__init__('cam_sub')

        # CvBridge converts ROS2 messages to OpenCV images to be sent through topics
        self.bridgeObject = CvBridge()

        # Name of the topic used to transfer the camera images
        # It should match the topic name in the publisher node
        self.topicNameFrames = '/camera/image_raw'
        # Queue size for messages
        self.queueSize = 20
        
        self.subscription = self.create_subscription(Image, self.topicNameFrames, self.listener_callbackFunction, self.queueSize)
        self.subscription # Prevent unused variable warning

    def listener_callbackFunction(self, imageMessage):

        self.get_logger().info('Image frame received')

        # Convert ROS2 image message to OpenCV image
        openCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage)

        # Show the image on the screen
        cv2.imshow("Camera video", openCVImage)
        cv2.waitKey(1)

def main(args=None):

    # Initialize rclpy
    rclpy.init(args=args)

    subscriberNode = SubscriberNodeClass()

    # Callback timer function is called recursively
    rclpy.spin(subscriberNode)

    # Destroy
    subscriberNode.destroy_node()
    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()