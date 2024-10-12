import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math
import csv
import os


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist,
                                                       'airsbot_cmd_vel',
                                                       10)

        # Subscribe to both left and right ultrasonic sensors
        self.left_subscription = self.create_subscription(
            Range,
            '/airsbot_ultrasonic_front_left',
            self.left_listener_callback,
            10
        )

        self.right_subscription = self.create_subscription(
            Range,
            '/airsbot_ultrasonic_front_right',
            self.right_listener_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/airsbot_odom',  # Replace with your actual odometry topic name
            self.odom_callback,
            10)

        # Flags and counters for open-loop control
        self.go_forward_flag = 1
        self.go_left_flag = 0
        self.go_rect_edge = 0
        self.stop_flag = 0
        self.forward_cnt = 0
        self.left_cnt = 0

        # Initialize position variables
        self.current_position = {'x': 0.0, 'y': 0.0}

        # Timer to log position every second
        self.position_log_timer = self.create_timer(1.0, self.log_position)

        # Timer to call open_loop_routine every 0.1 seconds
        self.routine_timer = self.create_timer(0.1, self.open_loop_routine)
        # Register the shutdown event
        self.add_on_shutdown(self.save_trajectory_to_csv)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        quaternion = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(quaternion)

        # Update current position
        self.current_position['x'] = position.x
        self.current_position['y'] = position.y

        # Log the odometry data
        self.get_logger().info(f'Current Odm Position: x={position.x}, y={position.y}')

    def save_trajectory_to_csv(self):
        # Define the CSV file path
        csv_file_path = os.path.join(os.path.expanduser('~'), 'robot_trajectory.csv')

        # Write the trajectory to a CSV file
        with open(csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y'])  # Write the header
            writer.writerows(self.trajectory)

        self.get_logger().info(f'Trajectory saved to {csv_file_path}')

    def left_listener_callback(self, msg):
        self.left_range = msg.range
        self.check_ranges()

    def right_listener_callback(self, msg):
        self.right_range = msg.range
        self.check_ranges()

    def check_ranges(self):
        if self.left_range is not None and self.right_range is not None:
            # the barrier is close to car
            if self.left_range < 0.4 or self.right_range < 0.4:
                self.stop_robot()
                self.stop_flag = 1
            # the barrier is far away from car
            elif self.left_range > 0.4 and self.right_range > 0.4:
                self.stop_flag = 0

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

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        quaternion = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(quaternion)

        # Update current position and yaw
        self.current_position['x'] = position.x
        self.current_position['y'] = position.y
        self.current_position['yaw'] = yaw

    def save_trajectory_to_csv(self):
        # Define the CSV file path
        csv_file_path = os.path.join(os.path.expanduser('~'), 'robot_trajectory.csv')

        # Write the trajectory to a CSV file
        with open(csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y', 'yaw'])  # Write the header
            writer.writerows(self.trajectory)

        self.get_logger().info(f'Trajectory saved to {csv_file_path}')

    def log_position(self):
        # Log the current position and yaw to trajectory list
        self.trajectory.append((self.current_position['x'], self.current_position['y'], self.current_position['yaw']))
        self.get_logger().info(f'Logged Position: x={self.current_position["x"]}, y={self.current_position["y"]}, yaw={self.current_position["yaw"]}')

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
        msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg)

    def go_left(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.3  # Adjust the turn speed as needed
        self.cmd_vel_publisher.publish(msg)

    def open_loop_routine(self):
        if self.stop_flag == 1:
            self.stop_robot()

        if self.go_forward_flag:
            self.go_forward()
            self.forward_cnt += 1

        if self.forward_cnt == 1100 and self.go_left_flag == 0:  # 200 * 0.1s = 20 seconds
            self.go_forward_flag = 0 #
            self.go_left_flag = 1 # go left
            self.go_rect_edge = 1 # go next edge
            self.forward_cnt = 0

        # Forward movement for the shorter side (800 counts)
        elif self.forward_cnt == 900 and self.go_rect_edge == 1:
            self.go_forward_flag = 0
            self.go_left_flag = 1
            self.go_rect_edge = 0
            self.forward_cnt = 0

        if self.go_left_flag == 1 and self.go_forward_flag == 0:
            self.go_left()
            self.left_cnt += 1

        if self.left_cnt == 45:  # 30 * 0.1s = 3 seconds
            self.go_left_flag = 0
            self.go_forward_flag = 1
            self.left_cnt = 0

    def destroy_node(self):
        self.save_trajectory_to_csv()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
