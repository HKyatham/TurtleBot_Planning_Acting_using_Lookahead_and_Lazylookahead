import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from tf_transformations import euler_from_quaternion

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        time.sleep(2)  # Allow some time for the publisher to connect
        self.yaw = 0.0
        self.action_timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0
        
    def timer_callback(self):
        self.i += 1
        if(self.i == 10):
            self.rotate_right(1.0)
        
        self.get_logger().info('Timer callback executed!: ' + str(self.yaw))
        
    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.get_logger().info('Odom callback executed!: ' + str(self.yaw))

    def move_forward(self, duration=1.0):
        msg = Twist()
        msg.linear.x = 0.2  # Move forward with a speed of 0.2 m/s
        self._publish_command(msg, duration)

    def move_back(self, duration=1.0):
        msg = Twist()
        msg.linear.x = -0.2  # Move backward with a speed of 0.2 m/s
        self._publish_command(msg, duration)

    def rotate_left(self, duration=1.0):
        target_yaw = self.yaw + math.radians(90)
        msg = Twist()
        msg.angular.z = 0.2
        if target_yaw > math.pi:
            target_yaw -= 2 * math.pi
        self._rotate_to_target(msg,target_yaw)
    
    def _is_yaw_reached(self, target_yaw):
    # Check if the current yaw is close enough to the target yaw
        return abs(target_yaw - self.yaw) < math.radians(5)

    def rotate_right(self, duration=1.0):
        target_yaw = self.yaw - math.radians(90)
        msg = Twist()
        msg.angular.z = -0.2
        if target_yaw < -math.pi:
            target_yaw += 2 * math.pi
        self._rotate_to_target(msg,target_yaw)
        
    def _rotate_to_target(self, msg, target_yaw):
        # msg = Twist()
        # msg.angular.z = 0.5  # Rotate with a speed of 0.5 rad/s
        while not self._is_yaw_reached(target_yaw):
            self.publisher_.publish(msg)
            rclpy.spin_once(self)  # Allow callbacks to update while rotating
        # Stop the robot after reaching the target yaw
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        
    def stop(self, duration=1.0):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = 0.0 # Setting linear to 0.0
        self._publish_command(msg, duration)

    def _publish_command(self, msg, duration):
        end_time = time.time() + duration
        for i in range(4):
            self.publisher_.publish(msg)
            time.sleep(0.25)  # Publish at 10 Hz
        # Stop the robot after the movement
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtleBotController()
    # executor = MultiThreadedExecutor()
    # executor.add_node(turtlebot_controller)

    # def run_movements():
    try:
        # turtlebot_controller.move_back(2.0)     # Move back for 2 seconds
        # turtlebot_controller.move_forward(2.0)  # Move forward for 2 seconds
        turtlebot_controller.rotate_left(1.0)     # Rotate left for 1.5 seconds
        # turtlebot_controller.rotate_right(1.0)    # Rotate right for 1.5 seconds
        # turtlebot_controller.stop(1.0)
        # executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        turtlebot_controller.destroy_node()
        rclpy.shutdown()

    

    # try:
        
    # finally:
    #     executor.shutdown()
    #     movement_thread.join()

if __name__ == '__main__':
    main()
