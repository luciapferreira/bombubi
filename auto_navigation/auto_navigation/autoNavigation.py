import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from nav_msgs.srv import GetPlan

class DWALocalPlanner(Node):

    def __init__(self):
        super().__init__('dwa_local_planner')

        # Declare parameters for DWA
        self.declare_parameter('max_accel', 0.2)
        self.declare_parameter('max_delta_yaw_rate', 0.2)
        self.declare_parameter('v_resolution', 0.1)
        self.declare_parameter('yaw_rate_resolution', 0.1)
        self.declare_parameter('predict_time', 3.0)
        self.declare_parameter('to_goal_cost_gain', 1.0)
        self.declare_parameter('speed_cost_gain', 1.0)
        self.declare_parameter('obstacle_cost_gain', 1.0)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_yaw_rate', 1.0)

        # Retrieve parameters
        self.max_accel = self.get_parameter('max_accel').get_parameter_value().double_value
        self.max_delta_yaw_rate = self.get_parameter('max_delta_yaw_rate').get_parameter_value().double_value
        self.v_resolution = self.get_parameter('v_resolution').get_parameter_value().double_value
        self.yaw_rate_resolution = self.get_parameter('yaw_rate_resolution').get_parameter_value().double_value
        self.predict_time = self.get_parameter('predict_time').get_parameter_value().double_value
        self.to_goal_cost_gain = self.get_parameter('to_goal_cost_gain').get_parameter_value().double_value
        self.speed_cost_gain = self.get_parameter('speed_cost_gain').get_parameter_value().double_value
        self.obstacle_cost_gain = self.get_parameter('obstacle_cost_gain').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').get_parameter_value().double_value

        # Subscribers to camera, map, and pose topics
        self.subscription_camera = self.create_subscription(
            Image,
            'camera',
            self.camera_callback,
            10)
        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            'costmap',
            self.map_callback,
            10)
        self.subscription_pose = self.create_subscription(
            PoseStamped,
            'bombubi_pose',
            self.pose_callback,
            10)
        self.subscription_goal = self.create_subscription(
            PoseStamped,
            'dwa_goal',
            self.goal_callback,
            10)
        self.publisher_cmd = self.create_publisher(
            Twist,
            'cmd_vel_nav',
            10)

        self.bridge = CvBridge()
        self.path = None
        self.obstacles = []
        self.costmap = None
        self.current_pose = None
        self.goal = None

        # Service client for path planning
        self.path_client = self.create_client(GetPlan, 'a_star_path')
        while not self.path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for A* path planner service...')

    def camera_callback(self, msg):
        # Callback to handle incoming camera data for obstacle detection
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.obstacles = self.detect_obstacles(cv_image)

    def map_callback(self, msg):
        # Callback to handle incoming costmap data
        self.costmap = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def pose_callback(self, msg):
        # Callback to handle incoming robot pose data
        self.current_pose = msg

    def goal_callback(self, msg):
        # Callback to handle incoming goals
        self.goal = [msg.pose.position.x, msg.pose.position.y]
        self.get_logger().info(f'Received new goal: {self.goal}')
        self.request_path_to_goal()

    def request_path_to_goal(self):
        # Request a path from the A* path planner
        if self.current_pose is None or self.goal is None:
            self.get_logger().error('Current pose or goal is not set.')
            return

        request = GetPlan.Request()
        start_pose = PoseStamped()
        start_pose.pose = self.current_pose.pose
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = self.goal[0]
        goal_pose.pose.position.y = self.goal[1]
        goal_pose.pose.position.z = 0.0

        request.start = start_pose
        request.goal = goal_pose

        # Call the path planning service and wait for the result
        path_future = self.path_client.call_async(request)
        path_future.add_done_callback(self.path_planned_callback)

    def path_planned_callback(self, path_future):
        # Callback to handle the planned path
        if path_future.result() is not None and path_future.result().plan.poses:
            self.path = path_future.result().plan.poses
            self.get_logger().info('Path received, starting DWA control loop.')
            self.dwa_control_loop()
        else:
            self.get_logger().error('Failed to get path or path is empty.')

    def detect_obstacles(self, cv_image):
        # Detect obstacles using camera data
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        obstacles = []
        for contour in contours:
            if cv2.contourArea(contour) > 500:
                x, y, w, h = cv2.boundingRect(contour)
                obstacles.append((x + w / 2, y + h / 2))
        return obstacles

    def dwa_control_loop(self):
        # DWA control loop to navigate to the goal
        if self.path is None or not self.path:
            self.get_logger().warn('No path set for DWA.')
            return

        rate = self.create_rate(10)
        path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in self.path]
        goal = path_points[-1]

        while rclpy.ok():
            if self.current_pose is None:
                continue

            current_position = self.current_pose.pose.position
            distance_to_goal = self.distance([current_position.x, current_position.y], goal)
            if distance_to_goal < 0.1:
                self.stop_robot()
                self.get_logger().info('Reached goal.')
                break

            # Current state of the robot: [x, y, yaw, velocity, yaw_rate]
            x = [
                current_position.x, 
                current_position.y, 
                self.quaternion_to_yaw(self.current_pose.pose.orientation),
                0.0,
                0.0
            ]

            # Calculate control inputs and trajectory using DWA
            u, trajectory = self.dwa_control(x, goal)
            cmd_vel = Twist()
            cmd_vel.linear.x = u[0]
            cmd_vel.angular.z = u[1]
            self.publisher_cmd.publish(cmd_vel)
            rate.sleep()

    def dwa_control(self, x, goal):
        # Compute control and trajectory using DWA
        dw = self.calc_dynamic_window(x)
        u, trajectory = self.calc_control_and_trajectory(x, dw, goal)
        return u, trajectory

    def calc_dynamic_window(self, x):
        # Calculate the dynamic window based on current state and parameters
        Vs = [0, self.max_speed, -self.max_yaw_rate, self.max_yaw_rate]
        Vd = [x[3] - self.max_accel * self.predict_time,
              x[3] + self.max_accel * self.predict_time,
              x[4] - self.max_delta_yaw_rate * self.predict_time,
              x[4] + self.max_delta_yaw_rate * self.predict_time]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        return dw

    def calc_control_and_trajectory(self, x, dw, goal):
        # Calculate the best control and trajectory to follow
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array(x)
        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for y in np.arange(dw[2], dw[3], self.yaw_rate_resolution):
                trajectory = self.predict_trajectory(x_init, v, y)
                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal)
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])
                ob_cost = self.obstacle_cost_gain * self.calc_obstacle_cost(trajectory)
                final_cost = to_goal_cost + speed_cost + ob_cost
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
        return best_u, best_trajectory

    def predict_trajectory(self, x_init, v, y):
        # Predict the trajectory based on the control inputs
        trajectory = np.array(x_init)
        time = 0
        x = np.array(x_init)
        while time <= self.predict_time:
            x = self.motion(x, [v, y])
            trajectory = np.vstack((trajectory, x))
            time += 0.1
        return trajectory

    def motion(self, x, u):
        # Update the robot's state based on the control inputs
        x[0] += u[0] * math.cos(x[2]) * 0.1
        x[1] += u[0] * math.sin(x[2]) * 0.1
        x[2] += u[1] * 0.1
        x[3] = u[0]
        x[4] = u[1]
        return x

    def calc_to_goal_cost(self, trajectory, goal):
        # Calculate the cost to the goal
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        return math.sqrt(dx ** 2 + dy ** 2)

    def calc_obstacle_cost(self, trajectory):
        # Calculate the cost associated with obstacles
        if not self.obstacles and self.costmap is None:
            return 0.0

        min_r = float("inf")
        for (ox, oy) in self.obstacles:
            dx = trajectory[:, 0] - ox
            dy = trajectory[:, 1] - oy
            r = np.hypot(dx, dy)
            if min(r) <= self.max_speed:
                min_r = min(r)

        if self.costmap is not None:
            for y in range(self.costmap.shape[0]):
                for x in range(self.costmap.shape[1]):
                    if self.costmap[y, x] > 0:
                        cx = x * 0.1  # Assuming costmap resolution is 0.1m
                        cy = y * 0.1
                        dx = trajectory[:, 0] - cx
                        dy = trajectory[:, 1] - cy
                        r = np.hypot(dx, dy)
                        if min(r) <= self.max_speed:
                            min_r = min(r)
        return 1.0 / min_r if min_r != float("inf") else 0.0

    def quaternion_to_yaw(self, orientation):
        # Convert quaternion orientation to yaw angle
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def distance(self, point1, point2):
        # Calculate the Euclidean distance between two points
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def stop_robot(self):
        # Stop the robot by publishing zero velocities
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.publisher_cmd.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = DWALocalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
