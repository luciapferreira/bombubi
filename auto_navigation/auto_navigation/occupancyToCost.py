import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

def occupancy_to_costmap(occupancy_grid: OccupancyGrid, inflation_radius: int = 5):
    # Extract the width, height, and resolution of the occupancy grid
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    resolution = occupancy_grid.info.resolution

    # Reshape the occupancy grid data to a 2D numpy array
    costmap = np.array(occupancy_grid.data).reshape((height, width))

    # Inflate obstacles: For each occupied cell, increase the cost of surrounding cells
    for y in range(height):
        for x in range(width):
            if costmap[y, x] == 100:  # If the cell is occupied
                for dy in range(-inflation_radius, inflation_radius + 1):
                    for dx in range(-inflation_radius, inflation_radius + 1):
                        # Ensure the neighbor cell is within the grid bounds
                        if 0 <= x + dx < width and 0 <= y + dy < height:
                            distance = np.sqrt(dx**2 + dy**2) * resolution  # Calculate the distance from the obstacle
                            if distance <= inflation_radius * resolution:
                                # Set cost based on the distance from the obstacle
                                cost = 100 - int((distance / (inflation_radius * resolution)) * 100)
                                costmap[y + dy, x + dx] = max(costmap[y + dy, x + dx], cost)
    
    return costmap  # Return the inflated cost map

class OccupancyToCostmapNode(Node):

    def __init__(self):
        super().__init__('occupancy_to_costmap')

        self.subscription = self.create_subscription(
            OccupancyGrid,
            'orboccupancygrid',
            self.listener_callback,
            10)

        self.publisher_ = self.create_publisher(
            OccupancyGrid,
            'costmap',  
            10)

    def listener_callback(self, msg):
        # Set the inflation radius for obstacle inflation
        inflation_radius = self.get_parameter('inflation_radius').value

        costmap = occupancy_to_costmap(msg, inflation_radius)
        costmap_msg = OccupancyGrid()

        # Copy the header and info from the received occupancy grid
        costmap_msg.header = msg.header
        costmap_msg.info = msg.info

        # Flatten the cost map data and assign it to the message
        costmap_msg.data = costmap.flatten().tolist()

        self.publisher_.publish(costmap_msg)

def main(args=None):
    rclpy.init(args=args)

    node = OccupancyToCostmapNode()
    node.declare_parameter('inflation_radius', 5)
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
