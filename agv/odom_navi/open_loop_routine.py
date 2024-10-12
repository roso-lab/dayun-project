import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range11


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
        
        self.distance_list = [1100, 950, 950, 950]

        self.right_subscription = self.create_subscription(
            Range,
            '/airsbot_ultrasonic_front_right',
            self.right_listener_callback,
            10
        )
        # Flags and counters for open-loop control
        self.go_forward_flag = 1
        self.go_left_flag = 0
        self.stop_flag_left = 0
        self.stop_flag_right = 0
        self.forward_cnt = 0
        self.left_cnt = 0
        self.line_cnt = 0

        # Timer to call open_loop_routine every 0.1 seconds
        self.routine_timer = self.create_timer(0.1, self.open_loop_routine)

    def left_listener_callback(self, msg):
        self.left_range = msg.range
        self.check_letf_range()

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
        if self.stop_flag_left == 1 or self.stop_flag_right ==1  :
            self.stop_robot()
        if self.go_forward_flag:
            self.go_forward()
            self.forward_cnt += 1

        if self.forward_cnt == self.distance_list[self.line_cnt]:  # 200 * 0.1s = 20 seconds
            self.go_forward_flag = 0
            self.go_left_flag = 1
            self.forward_cnt = 0
            self.line_cnt += 1 # Next edge
            if self.line_cnt == 4:
                self.line_cnt = 0

        if self.go_left_flag == 1 and self.go_forward_flag == 0:
            self.go_left()
            self.left_cnt += 1

        if self.left_cnt == 43:  # 30 * 0.1s = 3 seconds
            self.go_left_flag = 0
            self.go_forward_flag = 1
            self.left_cnt = 0


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
