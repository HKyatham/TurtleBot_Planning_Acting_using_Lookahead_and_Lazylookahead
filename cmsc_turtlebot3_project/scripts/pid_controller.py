import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
import math
import numpy as np
from cv_bridge import CvBridge
from path_finder import Path_Finder
import time


class PIDController(Node):
    def __init__(self, start_index, goal_index, step, clerance, path = None):
        super().__init__('pid_controller')

        # PID parameters
        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.1
        self.kp_angular = 3.0
        self.ki_angular = 0.0
        self.kd_angular = 0.2
        self.path_finder = Path_Finder()
        self.goal_index = goal_index
        # # Path and waypoint tracking
        self.path = path
        self.current_waypoint_index = 0
        # x, y = self.path[self.current_waypoint_index]
        self.target_x, self.target_y = 0, 0 
        self.lazy_or_not = True

        # Error terms
        self.prev_linear_error = 0.0
        self.sum_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.sum_angular_error = 0.0

        # Obstacle detection
        self.obstacle_detected = False
        self.lidar_threshold = 0.75  # Threshold distance in meters for LiDAR
        self.depth_threshold = 1.0  # Threshold distance in meters for depth camera

        # Depth camera processing
        self.bridge = CvBridge()

        # Publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.depth_subscriber = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Current robot pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.start = time.time()

    def odom_callback(self, msg):
        """Updates the current robot pose based on odometry data."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw angle from quaternion
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1.0 - 2.0 * (orientation_q.y**2 + orientation_q.z**2)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        """Checks if there's an obstacle within the threshold in front of the robot using LiDAR."""
        front_ranges = msg.ranges[len(msg.ranges)//2 - 10: len(msg.ranges)//2 + 10]
        self.obstacle_detected = any(r < self.lidar_threshold for r in front_ranges if r > 0)

    def depth_callback(self, msg):
        """Checks if there's an obstacle within the threshold in front of the robot using depth camera."""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            center_depth = depth_image[depth_image.shape[0] // 2 - 10:depth_image.shape[0] // 2 + 10,
                                        depth_image.shape[1] // 2 - 10:depth_image.shape[1] // 2 + 10]
            min_depth = np.nanmin(center_depth)
            if min_depth < self.depth_threshold:
                self.obstacle_detected = True
        except Exception as e:
            self.get_logger().error(f"Depth camera processing error: {e}")

    def control_loop(self):
        """PID control loop to move towards the current waypoint."""
        # if self.obstacle_detected:
        #     # Stop the robot if an obstacle is detected
        #     cmd_msg = Twist()
        #     cmd_msg.linear.x = 0.0
        #     cmd_msg.angular.z = 0.0
        #     self.cmd_vel_publisher.publish(cmd_msg)
        #     self.get_logger().info("Obstacle detected! Stopping the robot.")
        #     return
        # check for the goal
        if math.sqrt((self.goal_index[0]/100 -1 - self.current_x)**2 + (self.goal_index[1]/100 -1 - self.current_y)**2) < 0.1:
            # Stop the robot if all waypoints are reached
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd_msg)
            
            self.get_logger().info("Goal Reached!")
            self.get_logger().info("Lazy_Lookahead: "+str(time.time() - self.start))
            return

        # Compute errors
        distance_error = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
        target_angle = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        angular_error = target_angle - self.current_yaw

        # Normalize angular error to [-pi, pi]
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))

        # Linear PID
        linear_p = self.kp_linear * distance_error
        self.sum_linear_error += distance_error
        linear_i = self.ki_linear * self.sum_linear_error
        linear_d = self.kd_linear * (distance_error - self.prev_linear_error)
        linear_velocity = linear_p + linear_i + linear_d
        self.prev_linear_error = distance_error

        # Angular PID
        angular_p = self.kp_angular * angular_error
        self.sum_angular_error += angular_error
        angular_i = self.ki_angular * self.sum_angular_error
        angular_d = self.kd_angular * (angular_error - self.prev_angular_error)
        angular_velocity = angular_p + angular_i + angular_d
        self.prev_angular_error = angular_error

        # Adjust speeds for sharp turns
        if abs(angular_error) > 0.2:  # Turn in place if angular error is large
            linear_velocity = 0.0

        # Stop conditions
        if distance_error < 0.01:
            self.current_waypoint_index += 1
            x, y = (100 * (self.current_x + 1)) // 5 * 5, (100 * (self.current_y + 1)) // 5 * 5
            if not self.lazy_or_not:
                cmd_msg = Twist()
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self.cmd_vel_publisher.publish(cmd_msg)
                # self.get_logger().info(str((x,y)))
                # path_finder = self.path_finder()
                self.get_logger().info(str(self.goal_index))
                path = self.path_finder.Astar((x,y),self.goal_index)
                # self.get_logger().info(str(path))
                if path != 0:
                    if len(path) > 2:
                        x, y = path[2]
                        self.target_x, self.target_y = x / 100 - 1, y / 100 - 1
            else: # Lazy look ahead version
                if self.path  == None:
                    self.get_logger().info("Fetching Plan")
                    path_finder = Path_Finder()
                    self.path = path_finder.Astar((x,y),self.goal_index)
                    self.get_logger().info(str(self.path))
                if len(self.path) > self.current_waypoint_index:
                    x, y = self.path[self.current_waypoint_index]
                    self.target_x, self.target_y = x / 100 - 1, y / 100 - 1
                else:
                    self.target_x, self.target_y = self.goal_index[0] / 100 - 1, self.goal_index[1] / 100 - 1
            return

        # Publish velocities
        cmd_msg = Twist()
        cmd_msg.linear.x = max(min(linear_velocity, 0.22), -0.22)  # Limit max linear speed
        cmd_msg.angular.z = max(min(angular_velocity, 2.84), -2.84)  # Limit max angular speed
        self.cmd_vel_publisher.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)

    start_index = (100, 100)
    goal_index = (650, 100)
    Step = 5
    clerance = 5
    Robot_radius = 20
    # path_finder = Path_Finder()
    # path_finder.Astar(start_index, goal_index)
    # # print("Generated graph: ", path_finder.graph)
    # path_finder.shortest_path()
    # path = path_finder.path
    # print("Generated Path: ", path)

    # Pass the path to the PID controller
    pid_controller = PIDController(start_index, goal_index, Step, clerance=clerance + Robot_radius)
    rclpy.spin(pid_controller)

    pid_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
