import cv2
import os

# ROS2 package modules and libraries
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class SaveImageNodeClass(Node):

    num = 0
    save_directory = os.path.expanduser('~/projeto/calibration/images/img')

    # Constructor
    def __init__(self):
        super().__init__('save_image_node')

        # CvBridge converts ROS2 messages to OpenCV images to be sent through topics
        self.bridgeObject = CvBridge()

        # Name of the topic used to transfer the camera images
        # It should match the topic name in the publisher node
        self.topicNameFrames = 'camera'
        # Queue size for messages
        self.queueSize = 20

        self.subscription = self.create_subscription(Image, self.topicNameFrames, self.listener_callbackFunction, self.queueSize)
        self.subscription # Prevent unused variable warning

    def listener_callbackFunction(self, imageMessage):
        #self.get_logger().info('Image frame received')

        # Convert ROS2 image message to OpenCV image
        openCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage)

        k = cv2.waitKey(5)

        # Wait for 's' key press to save image
        if k == ord('s'):

            cv2.imwrite(self.save_directory + str(self.num) + '.png', openCVImage)
            print("Saved")
            self.num += 1

        # Show the image on the screen
        cv2.imshow('Camera video', openCVImage)
        #cv2.waitKey(1)

def main(args=None):

    # Initialize rclpy
    rclpy.init(args=args)

    saveimageNode = SaveImageNodeClass()

    # Callback timer function is called recursively
    rclpy.spin(saveimageNode)

    # Destroy
    saveimageNode.destroy_node()
    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
