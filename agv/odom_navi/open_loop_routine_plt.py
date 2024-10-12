import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from math import pi, sin, cos
import math
import datetime
import time
import subprocess
class Vector2:
    def __init__(self, x:float, y:float, z:float) -> None:
        self.x = x
        self.y = y
class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'airsbot_cmd_vel', 10)
        # Subscribe to both left and right ultrasonic sensors
        self.left_subscription = self.create_subscription(
            Range,
            '/airsbot_ultrasonic_front_left',
            self.left_listener_callback,
            10
        )
        
        self.distance_list = [695, 200, 700, 795, 615]
        self.angle_list = [-17, 70, 52, 52, 52]

        self.right_subscription = self.create_subscription(
            Range,
            '/airsbot_ultrasonic_front_right',
            self.right_listener_callback,
            10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            'aisbot/odom',
            self.odom_callback,
            10
        )
        # Flags and counters for open-loop control
        self.go_forward_flag = 1
        self.adjust_yaw_flag = 0
        self.go_left_flag = 0
        self.stop_flag_left = 0
        self.stop_flag_right = 0
        self.forward_cnt = 0
        self.left_cnt = 0
        self.line_cnt = 0
        self.angle_cnt = 0

        # Timer to call open_loop_routine every 0.1 seconds
        self.routine_timer = self.create_timer(0.1, self.open_loop_routine)

        # List to store the path
        self.path = [(0, 0)]
        self.current_position = [0, 0]
        self.current_angle = 0  # In radians, 0 means facing right

    def left_listener_callback(self, msg):
        self.left_range = msg.range
        self.check_letf_range()
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        quaternion = msg.pose.pose.orientation
        self.current_position = Vector2(position.x, position.y)
        # Convert quaternion to yaw angle
        self.current_heading = self.quaternion_to_yaw(quaternion)
    def quaternion_to_yaw(self, quaternion):
        """Convert a quaternion to a yaw angle."""
        (roll, pitch, yaw) = self.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw

    def euler_from_quaternion(self, quat):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        x, y, z, w = quat
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z
    def right_listener_callback(self, msg):
        self.right_range = msg.range
        self.check_right_range()

    def check_letf_range(self):
        if self.left_range is not None:
            # the barrier is close to car
            if self.left_range < 0.4:
                self.stop_robot()
                self.stop_flag_left = 1
            # the barrier is far away from car
            elif self.left_range > 0.4:
                self.stop_flag_left = 0

    def check_right_range(self):
        if self.right_range is not None:
            # the barrier is close to car
            if self.right_range < 0.4:
                self.stop_robot()
                self.stop_flag_right = 1
            # the barrier is far away from car
            elif self.right_range > 0.4:
                self.stop_flag_right = 0

    def stop_robot(self):
        self.get_logger().info('Obstacle detected! Stopping the robot.')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def go_forward(self):
        msg = Twist()
        msg.linear.x = 0.45  # Adjust the speed as needed
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -0.0048
        self.cmd_vel_publisher.publish(msg)
        
        # Update position
        self.current_position[0] += 0.45 * 0.1 * cos(self.current_angle)  # delta_x
        self.current_position[1] += 0.45 * 0.1 * sin(self.current_angle)  # delta_y
        self.path.append(tuple(self.current_position))
        print(tuple(self.current_position))


    def go_left(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        temp = 1
        if self.angle_list[self.angle_cnt] < 0:
            temp = -1
        msg.angular.z = 0.3 * temp # Adjust the turn speed as needed
        self.cmd_vel_publisher.publish(msg)
        
        # Update angle
        self.current_angle += 0.3 * 0.1  # delta_theta
        self.current_angle %= 2 * pi  # Normalize the angle between 0 and 2*pi

    def open_loop_routine(self):
        if self.stop_flag_left == 1 or self.stop_flag_right == 1:
            self.stop_robot()
        if self.go_forward_flag and not self.go_left_flag:
            self.go_forward()
            self.forward_cnt += 1
        if self.forward_cnt == self.distance_list[self.line_cnt]:  # 200 * 0.1s = 20 seconds
            self.go_forward_flag = 0
            self.go_left_flag = 1
            self.forward_cnt = 0
            self.line_cnt += 1  # Next edge
            if self.line_cnt == 5:
                self.line_cnt = 0
                self.stop_robot()
                time.sleep(5)
                rclpy.shutdown()

        

        if self.go_left_flag == 1 and self.go_forward_flag == 0:
            self.go_left()
            self.left_cnt += 1
            
        if self.left_cnt == abs(self.angle_list[self.angle_cnt]):
            self.go_left_flag = 0
            self.go_forward_flag = 1
            self.left_cnt = 0 
            self.angle_cnt+=1
        

    # def plot_path(self):
    #     x_coords, y_coords = zip(*self.path)
    #     plt.plot(x_coords, y_coords, marker='o')
    #     plt.title("Robot Path")
    #     plt.xlabel("X Position")
    #     plt.ylabel("Y Position")
    #     plt.grid(True)
    #     plt.show()
    # def to_csv(self):
    #     current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    #     file_name = f"{current_time}.csv"

    #     with open(file_name, mode='w') as file:
    #         file.write(','.join(map(str, self.path) + '\n'))
    #     print(f"CSV has build, the name is: {file_nam
def main(args=None):
    rclpy.init(args=args)
    plt.ion()
    robot_controller = RobotController()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(robot_controller)
    # robot_controller.plot_path()  # Plot the path after execution
    # robot_controller.to_csv()
    try:
        executor.spin()
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()
    # rclpy.spin(robot_controller)
    
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
