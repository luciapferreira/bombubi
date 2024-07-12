import cv2

# ROS2 package modules and libraries
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node 
from cv_bridge import CvBridge 

# To make sure the file can be execute:
# chmod +x cameraPublisher.py

# To run publisher:
# ros2 run camera_publisher publisher

class PublisherNodeClass(Node):
    # Constructor
    def __init__(self):
        super().__init__('cam_pub')

        self.cameraDeviceNumber=0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)

        # CvBridge converts ROS2 messages to OpenCV images to be sent through topics
        self.bridgeObject = CvBridge()

        # Name of the topic used to transfer the camera images
        # It should match the topic name in the publisher node
        self.topicNameFrames = '/camera/image_raw'
        # Queue size for messages
        self.queueSize = 20

        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)

        self.periodCommunication = 0.02
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        #self.i = 0

    def timer_callbackFunction(self):
        success, frame = self.camera.read()
        #rot = cv2.rotate(frame, cv2.ROTATE_180)
        frame = cv2.resize(frame, (820,640), interpolation=cv2.INTER_CUBIC)

        if success == True:
            ROS2ImageMessage = self.bridgeObject.cv2_to_imgmsg(frame)
            self.publisher.publish(ROS2ImageMessage)


        #self.get_logger().info('Publishing image number %d' % self.i)
        #self.i += 1

def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    publisherObject = PublisherNodeClass()

    # Callback timer function is called recursively
    rclpy.spin(publisherObject)

    # Destroy
    publisherObject.destroy_node()
    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()