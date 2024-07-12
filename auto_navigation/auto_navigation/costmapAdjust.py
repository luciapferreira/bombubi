import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class CostmapPositionAdjuster(Node):
    def __init__(self):
        super().__init__('costmap_position_adjuster')

        # Set up QoS profiles to allow subscription and publication on the same topic
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.listener_callback,
            qos_profile)
        self.publisher = self.create_publisher(OccupancyGrid, '/global_costmap/costmap', qos_profile)

    def listener_callback(self, msg):
        adjusted_msg = msg

        #adjusted_msg.info.origin.position.x = -8.0
        adjusted_msg.info.origin.position.y = -8.0
        
        #adjusted_msg.info.origin.orientation.x = -0.7071068
        
        adjusted_msg.info.origin.orientation.z = 0.7071068
        adjusted_msg.info.origin.orientation.w = 0.7071068

        self.publisher.publish(adjusted_msg)

def main(args=None):
    rclpy.init(args=args)
    costmap_position_adjuster = CostmapPositionAdjuster()
    rclpy.spin(costmap_position_adjuster)
    costmap_position_adjuster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
