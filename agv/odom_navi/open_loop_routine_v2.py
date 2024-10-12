import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range  # Adjust the import based on your message definition
from nav_msgs.msg import Odometry
import math
import time

class Vector3:
    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


class Vector2:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Vector2(x={self.x}, y={self.y})"

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist,
                                                       '/airsbot_cmd_vel',
                                                       10)

        self.left_subscription = self.create_subscription(
            Range,
            '/airsbot_ultrasonic_front_left',
            self.left_listener_callback,
            10
        )
        
        self.distance_list = [950, 200, 1000, 1050, 830]
        self.angle_list = [-17, 70, 52, 52, 52]

        self.right_subscription = self.create_subscription(
            Range,
            '/airsbot_ultrasonic_front_right',
            self.right_listener_callback,
            10
        )

        self.odom_subscription = self.create_subscription(Odometry,
                                                          '/airsbot_odom',  # Replace with your actual odometry topic name
                                                          self.odom_callback,
                                                          10)

        # Flags and counters for open-loop control
        self.go_forward_flag = 1
        self.go_left_flag = 0
        self.forward_cnt = 0
        self.left_cnt = 0
        self.angle_cnt = 0
        self.stop_flag = 0
        self.stop_flag = 0
        self.stop_flag = 0
        self.yaw_adjustment_threshold = 0.25
        self.initial_position = None
        self.initial_yaw = None
        self.last_yaw = None
        self.last_position = None

        self.current_position = None
        self.current_heading = None

        # Timer to call open_loop_routine every 0.1 seconds
        self.routine_timer = self.create_timer(0.1, self.open_loop_routine)
        # self.direction_timer = self.create_timer(0.1, self.adjust_yaw)
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
    def is_stop(self):
        return self.stop_flag_left or self.stop_flag_right
    
    def open_loop_routine(self):
        # stop if obstacle is detected
        if self.is_stop():
            self.stop_robot()
            return

        # go forward
        if self.go_forward_flag:
            if self.initial_position is None:
                self.initial_position = self.current_position
                self.initial_yaw = self.current_heading
                return

            self.go_forward()
            self.forward_cnt += 1

        # before turning, adjust yaw
        # if self.forward_cnt > 1000 and self.forward_cnt < 1150:
        #     self.last_position = self.current_position
        #     self.last_yaw = self.current_heading
        #     # Log the distance to the target
        #     desired_yaw_adjustment = self.last_yaw - self.initial_yaw
        #     self.get_logger().info(f'desired_yaw_adjustment: {desired_yaw_adjustment:.2f}')
        #     if abs(desired_yaw_adjustment) < self.yaw_adjustment_threshold:
        #         self.go_forward()
        #         self.forward_cnt += 1

        #     else:
        #         if desired_yaw_adjustment > 0:
        #             self.go_left()

        #         else:
        #             self.go_right()

        # if self.forward_cnt == 1150:  # 200 * 0.1s = 20 seconds
        #     self.go_forward_flag = 0
        #     self.go_left_flag =1
        #     self.forward_cnt = 0

        # if self.go_left_flag == 1 and self.go_forward_flag == 0:
        #     self.go_left()
        #     self.left_cnt += 1

        # if self.left_cnt == 130:  # 30 * 0.1s = 3 seconds
        #     self.go_left_flag = 0
        #     self.go_forward_flag = 1
        #     self.left_cnt = 0
        if abs(0.8 * self.distance_list[self.line_cnt]) < self.forward_cnt < abs(self.distance_list[self.line_cnt]):
            
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
        
    def obstacle_callback(self, msg):
        self.obstacle_range = msg.range
        self.check_ranges()

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

    def stop_robot(self):
        self.get_logger().info('Obstacle detected! Stopping the robot.')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    # def check_ranges(self):
    #     if self.obstacle_range is not None:
    #         # the barrier is close to car
    #         if self.obstacle_range < 0.3:
    #             self.stop_robot()
    #             self.stop_flag = 1
    #         # the barrier is far away from car
    #         elif self.obstacle_range > 0.3:
    #             self.stop_flag = 0

    def go_forward(self):
        msg = Twist()
        msg.linear.x = 0.3  # Adjust the speed as needed
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -0.0022
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

    def go_right(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -0.3  # Adjust the turn speed as needed
        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
