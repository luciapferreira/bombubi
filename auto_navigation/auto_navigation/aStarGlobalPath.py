import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.srv import GetPlan
import numpy as np
import heapq

def heuristic(a, b):
    """Calculate the heuristic distance (Manhattan distance) between two points."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal, costmap, obstacle_threshold):
    """Perform A* search to find the path from start to goal on the costmap."""
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    open_list = []
    heapq.heappush(open_list, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    open_set = {start}

    while open_list:
        _, current_g, current = heapq.heappop(open_list)
        open_set.remove(current)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            if not (0 <= neighbor[0] < costmap.shape[1] and 0 <= neighbor[1] < costmap.shape[0]):
                continue  # Skip out of bounds

            tentative_g_score = current_g + costmap[neighbor[1], neighbor[0]]
            if costmap[neighbor[1], neighbor[0]] >= obstacle_threshold:  # Skip obstacles
                continue

            if neighbor in g_score and tentative_g_score >= g_score[neighbor]:
                continue

            came_from[neighbor] = current
            g_score[neighbor] = tentative_g_score
            f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
            if neighbor not in open_set:
                heapq.heappush(open_list, (f_score[neighbor], tentative_g_score, neighbor))
                open_set.add(neighbor)

    return None  # No path found

class AStarPathPlanner(Node):

    def __init__(self):
        super().__init__('a_star_path_planner')
        self.subscription_costmap = self.create_subscription(
            OccupancyGrid,
            'costmap',
            self.costmap_callback,
            10)
        self.subscription_pose = self.create_subscription(
            PoseStamped,
            'bombubi_pose',
            self.pose_callback,
            10)
        self.costmap = None
        self.current_pose = None

        self.declare_parameter('obstacle_threshold', 100)

        self.create_service(GetPlan, 'a_star_path', self.compute_path_callback)

    def compute_path_callback(self, request, response):
        if self.costmap is None:
            self.get_logger().error("No costmap received yet.")
            return response

        if self.current_pose is None:
            self.get_logger().error("No current pose received yet.")
            return response

        start = (int(self.current_pose.pose.position.x), int(self.current_pose.pose.position.y))
        goal = (int(request.goal.pose.position.x), int(request.goal.pose.position.y))

        if not (0 <= start[0] < self.costmap.shape[1] and 0 <= start[1] < self.costmap.shape[0]):
            self.get_logger().error("Start position is out of bounds.")
            return response

        if not (0 <= goal[0] < self.costmap.shape[1] and 0 <= goal[1] < self.costmap.shape[0]):
            self.get_logger().error("Goal position is out of bounds.")
            return response

        self.get_logger().info(f"Computing path from {start} to {goal}.")

        obstacle_threshold = self.get_parameter('obstacle_threshold').value
        path = a_star(start, goal, self.costmap, obstacle_threshold)
        if path:
            response.plan = self.create_path_message(path)
        else:
            self.get_logger().error("No path found.")
            response.plan = Path()

        return response

    def costmap_callback(self, msg):
        """Callback to handle incoming map data."""
        try:
            self.costmap = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        except Exception as e:
            self.get_logger().error(f"Failed to reshape costmap: {e}")

    def pose_callback(self, msg):
        """Callback to handle incoming robot pose data."""
        self.current_pose = msg

    def create_path_message(self, path):
        """Create a Path message from the calculated path."""
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for (x, y) in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        return path_msg

def main(args=None):
    rclpy.init(args=args)
    node = AStarPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
