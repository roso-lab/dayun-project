import rclpy
from geometry_msgs.msg import Twist
# from protocol.msg import HeadTofPayload  # Adjust this import based on your message definition
from sensor_msgs.msg import Range  # Adjust the import based on your message definition
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math
import serial
import threading  # To handle serial reading in a separate thread
from collections import deque
from nav_msgs.msg import Odometry
import geometry_msgs.msg
import tf2_ros
import time


# import math.pi


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


class ContinuousPublisher(Node):
    def __init__(self):
        super().__init__('continuous_publisher')

        # self.timer = self.create_timer(0.1, self.navigate)
        self.speed_lv = 3.0
        self.linear = Vector3(0.5, 0.0, 0.0)
        self.angular = Vector3(0.0, 0.0, 0.0)
        self.current_heading = 0.0

        # Flag zone
        self.init_odo_position_flag = 0
        self.turning_flag = 0
        self.turning_finish = 0
        self.update_target_flag = 0
        self.target_number = 0

        self.turning_yaw = 0.0
        self.turning_angle = 0.0
        self.yaw_init = 0.0
        self.yaw_last = 0.0
        self.MAX_SPEED = 16
        self.target_distance_threshold = 3.0
        # Subscribe to the distance sensor topic
        # self.create_subscription(HeadTofPayload, '/mi_desktop_48_b0_2d_5f_bf_41/head_tof_payload', self.obstacle_callback, 10)
        # self.create_subscription(Range, '/mi_desktop_48_b0_2d_5f_bf_41/ultrasonic_payload', self.obstacle_callback, 10)
        self.is_moving = True  # State to check if the robot should be moving
        self.max_capacity_position_queue = 100
        self.position_queue = deque(maxlen=self.max_capacity_position_queue)  # Fixed-size queue to store x, y positions
        self.current_position = Vector2(0.0, 0.0)  # Will be set by odm callback
        self.yaw_threshould = 1/ 180 * math.pi
        self.distance_to_target = 0.0
        self.distance_to_target_prev = 0.0
        self.prev_judgeangle = 0.0
        self.yaw_prev = 0.0

        self.target_position_queue = [Vector2(10.0, 10.0), Vector2(10.0, 0.0), Vector2(0.0,0.0)]  # Target coordinates (x_t, y_t)

        # self.target_position_queue = [Vector2(0.0, 5.0), Vector2(5.0, 5.0), Vector2(5.0, 0.0), Vector2(0.0, 0.0)] # Target coordinates (x_t, y_t)
        self.target_position = self.target_position_queue[0]  # Vector2(-2414588.30, 5369302.21)
        # self.target_position = Vector3(-2414588.30, 5369302.21, 0.0)

        # subscribe to ODOM
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  # Replace with your actual odometry topic name
            self.odom_callback,
            10)
        self.publisher_velocity = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.odom_navigation)



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

    def distance_to_goal(self):
        self.get_logger().info(f'Current position to target:{self.current_position}')
        self.get_logger().info(f'Target position:{self.target_position}')

        target_vector = Vector2(
            self.target_position.x - self.current_position.x,
            self.target_position.y - self.current_position.y,
        )
        self.distance_to_target = math.sqrt(target_vector.x ** 2 + target_vector.y ** 2)
        self.get_logger().info(f'Current distance to target:{self.distance_to_target}')
        return self.distance_to_target
    
    def update_target_position(self):
        self.target_number += 1
        self.target_position = self.target_position_queue[self.target_number]
        self.get_logger().info(f"Reached target No.{self.target_number}, heading to next target")
        self.update_target_flag = 0

    def odom_callback(self, msg):
        self.lastest_odom_msg = msg     # store the latest odometry message

    def ok_to_turn(self):
        return self.calculate_turning_yaw() >= 15/ 180 * math.pi or len(self.position_queue)==self.max_capacity_position_queue
    
    def odom_navigation(self):
        # read position from odometry
        if hasattr(self, 'lastest_odom_msg'):
            msg = self.lastest_odom_msg
            position = msg.pose.pose.position
            quaternion = msg.pose.pose.orientation
            # Convert gps position
            self.get_logger().info(f'Current Odm Position: x={position.x}, y={position.y}')

        # initialize odometry position
        if not self.init_odo_position_flag:
            self.init_odm_position = Vector2(position.x, position.y)
            self.init_odo_position_flag = 1
        
        # record and append current position from odometry to queue
        else:
            # TO DO: ADD ODOMETRY FOR CHECKING WHETHER ROBOT IS MOVING OR NOT
            self.current_position = Vector2(position.x - self.init_odm_position.x,
                                            position.y - self.init_odm_position.y)
            # self.current_position = Vector2(position.x, position.y)
            if self.turning_flag == 0:
                self.position_queue.append(self.current_position)


        # calculate distance to target
        self.distance_to_target = self.distance_to_goal()
        # self.get_logger().info(f'Current distance to target:{self.distance_to_target}')

        # Stop if close enough to the target
        if self.distance_to_target < self.target_distance_threshold:
            if self.target_number == len(self.target_position_queue) - 1:
                self.stop()
                self.get_logger().info('Reached the target position. Stopping the robot.')
                time.sleep(5)
                return
            else:
                self.update_target_flag = 1
                time.sleep(5)
                self.update_target_position()

        # to check go forward or turning
        # turning_flag = 0, go forward
        if self.turning_flag == 0:
            self.go_forward()
            self.get_logger().info(f"Current target is No.{self.target_number}")

        # if queue is full, turning flag = 1, then adjust yaw
        if self.turning_flag == 0 and self.ok_to_turn():
            
            # adjust yaw Turing

            # self.yaw_init = self.quaternion_to_yaw(quaternion)
            # self.yaw_prev = self.yaw_init
    
        
            self.turning_flag = 1


            # start turning, record current heading
            self.current_heading = self.quaternion_to_yaw(quaternion)

            # turning initialization
            self.yaw_init = self.current_heading
            self.get_logger().info(f"Starting yaw is: {self.yaw_init}")

            # calculate turning_yaw
            self.turning_yaw = self.calculate_turning_yaw()
            self.get_logger().info(f"Desired turning angle is: {self.turning_yaw}")

            # turing with initial angular velocity (+- 0.2)
            self.adjust_yaw(self.turning_yaw)

            # time.sleep(0.1)
            self.turning_angle = self.turning_yaw
        
        if self.turning_flag == 1:
            self.turning()

        # turning finish, clear queue
        if self.turning_finish == 1:
            self.position_queue = deque()
            self.turning_finish = 0
            self.turning_flag = 0

        self.distance_to_target_prev = self.distance_to_target


    def turning(self):
        self.yaw_last = self.current_heading
        angle_change = self.yaw_last - self.yaw_prev
        
        if abs(angle_change) > 2 * math.pi:
            if angle_change > 0: 
                
                angle_change -= 2 * math.pi
            else:
               
                angle_change += 2 * math.pi
            # angle_change = -angle_change
        angle_change -= self.turning_angle
        self.get_logger().info(f"The angle change will be: {angle_change}")
        if abs(angle_change) >= self.yaw_threshould:
            self.get_logger().info(f"Remain: {self.turning_angle}")
            self.adjust_yaw(self.turning_angle)
            temp = self.turning_angle * 0.05
            self.turning_angle -= temp
            
        else:
            self.turning_finish = 1
        self.yaw_prev = self.yaw_last
    # def turning(self):
    #     # turning started, from yaw_init + adjust_yaw(turning_yaw)
    #     self.yaw_last = self.current_heading
    #     # wtf Judge sumetimes very large

    #     #101315
    #     angle_change = self.yaw_last - self.yaw_prev
    #     self.get_logger().info(f"The angle change will be: {angle_change}")
    #     if abs(angle_change) > 2 * math.pi:
    #         if angle_change > 0: 
                
    #             angle_change -= 2 * math.pi
    #         else:
               
    #             angle_change += 2 * math.pi
    #         # angle_change = -angle_change
    #     judge_func = abs(angle_change - self.turning_yaw)
    #     self.get_logger().info(f"Yaw_last is: {self.yaw_last}")
    #     self.get_logger().info(f"Judget angle is: {judge_func}")

    #     if judge_func > self.yaw_threshould:
    #         if abs(judge_func) > abs(self.prev_judgeangle):
    #             self.turning_yaw = -self.turning_yaw
    #         # while judge_func >= self.yaw_threshould:
    #         self.adjust_yaw(self.turning_yaw)
    #         # time.sleep(0.1)

    #     if judge_func <= self.yaw_threshould:
    #         self.turning_finish = 1
    #     # Renew prev judge angle
    #     self.prev_judgeangle = judge_func
    #     self.yaw_prev = self.yaw_last
    def calculate_turning_yaw(self):
        # Calculate vector from the last position in the queue to the target
        last_position = self.position_queue[-1]
        vector_to_target = Vector2(
            self.target_position.x - last_position.x,
            self.target_position.y - last_position.y
        )

        # Calculate vector from the first position in the queue to the last position
        first_position = self.position_queue[int(self.max_capacity_position_queue / 2.0)]
        vector_start_to_end = Vector2(
            last_position.x - first_position.x,
            last_position.y - first_position.y
        )

        self.get_logger().info(f"Vector to target: {vector_to_target}")
        self.get_logger().info(f"Vector from start to end in queue: {vector_start_to_end}")

        # Calculate the angle (yaw) between the two vectors
        turning_yaw = self.calculate_angle(vector_start_to_end, vector_to_target)
        self.get_logger().info(f"Calculated yaw angle to target: {turning_yaw}")
        return turning_yaw

    def calculate_angle(self, vec1: Vector2, vec2: Vector2) -> float:
        # Calculate the dot product
        dot_product = vec1.x * vec2.x + vec1.y * vec2.y

        # Calculate the magnitudes
        magnitude_vec1 = math.sqrt(vec1.x ** 2 + vec1.y ** 2)
        magnitude_vec2 = math.sqrt(vec2.x ** 2 + vec2.y ** 2)

        # Avoid division by zero and invalid acos inputs
        if magnitude_vec1 == 0 or magnitude_vec2 == 0:
            return 0.0

        # Calculate cosine of the angle
        cos_theta = dot_product / (magnitude_vec1 * magnitude_vec2)

        # Ensure the value is within the valid range for acos
        cos_theta = min(1.0, max(-1.0, cos_theta))

        # Calculate the angle between the vectors in radians
        angle = math.acos(cos_theta)

        # Calculate the direction of the angle
        # Using atan2 to determine the angle with sign
        cross_product = vec1.x * vec2.y - vec1.y * vec2.x
        # if cross_product < 0:
        #     angle = -angle

        # return angle

        # Adj 871602
        # Forget to deal with the case where the vectors are collinear
        if cross_product < 0:
            if (vec1.x != 0 or vec2.x != 0) and (vec1.y != 0 or vec2.y != 0):
                angle = -angle
            elif vec1.x == vec2.x == 0:
                if vec1.y * vec2.y < 0:
                    angle = math.pi
                else:
                    angle = 0.0
            elif vec1.y == vec2.y == 0:
                if vec1.x * vec2.x < 0:
                    angle = math.pi
                else:
                    angle = 0.0
        return angle

    def adjust_yaw(self, angle: float):
        msg = Twist()
        msg.angular.x = 0.0  # Assuming no rotation around x and y axes
        msg.angular.y = 0.0
        # if angle > 0:
        #     sign = 1.0
        # else:
        #     sign = -1.0
        # msg.angular.z = 0.2 * sign

        # Adj 871632
        if angle != 0:
            if angle > 0:
                sign = 1.0
            else:
                sign = -1.0
            msg.angular.z = 0.2 * sign
        else:
            msg.angular.z = 0.0

        self.publisher_velocity.publish(msg)
        self.get_logger().info(f"Adjusting yaw: {msg.angular.z}")

    def go_forward(self):
        msg = Twist()
        msg.linear.x = 0.1 * self.speed_lv
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_velocity.publish(msg)
        self.get_logger().info('Go forward: "%s"' % msg)

    def go_back(self, event=None):
        self.linear.x = -0.1 * self.speed_lv
        self.linear.y = 0.0
        self.angular.z = 0.0

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_velocity.publish(msg)
        self.get_logger().info('Stop: "%s"' % msg)
        # self.update_target_flag = 1
        time.sleep(0.5)

    # def go_left(self, event=None):
    #     self.linear.x = 0.0
    #     self.linear.y = 0.1 * self.speed_lv
    #     self.angular.z = 0.0

    # def go_right(self, event=None):
    #     self.linear.x = 0.0
    #     self.linear.y = -0.1 * self.speed_lv
    #     self.angular.z = 0.0

    # def turn_left(self, event=None):
    #     self.linear.x = 0.0
    #     self.linear.y = 0.0
    #     self.angular.z = 0.1 * self.speed_lv

    # def turn_right(self, event=None):
    #     self.linear.x = 0.0
    #     self.linear.y = 0.0
    #     self.angular.z = -0.1 * self.speed_lv

    # def stop(self, event=None):
    #     self.linear.x = 0.0
    #     self.linear.y = 0.0
    #     self.angular.z = 0.0

    # def speed_up(self, event=None):
    #     self.speed_lv += 1
    #     self.speed_lv = min(self.speed_lv, self.MAX_SPEED)

    # def speed_down(self, event=None):
    #     self.speed_lv -= 1
    #     self.speed_lv = max(self.speed_lv, 1)


def main(args=None):
    rclpy.init(args=args)
    continuous_publisher = ContinuousPublisher()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(continuous_publisher)
    # executor.spin()

    # Start moving forward
    # continuous_publisher.go_forward()
    # Call back
    # message = continuous_publisher.msg
    '''
    continuous_publisher.odom_callback(message)
    continuous_publisher.gps_callback(message)
    '''

    try:
        executor.spin()
    finally:
        continuous_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
