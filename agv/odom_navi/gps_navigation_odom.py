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

def geodetic_to_cartesian(lat, lon, alt):
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = 2 * f - f ** 2

    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    N = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)

    x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
    z = ((1 - e2) * N + alt) * math.sin(lat_rad)

    return x, y, z

def parse_nmea_sentence(sentence):
    parts = sentence.split(',')
    if parts[0] == '$GNGGA':
        try:
            lat_deg = float(parts[2][:2])
            lat_min = float(parts[2][2:])
            lat = lat_deg + lat_min / 60
            if parts[3] == 'S':
                lat = -lat

            lon_deg = float(parts[4][:3])
            lon_min = float(parts[4][3:])
            lon = lon_deg + lon_min / 60
            if parts[5] == 'W':
                lon = -lon

            alt = float(parts[9])

            return lat, lon, alt
        except (IndexError, ValueError):
            return None
    return None


class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, '/diy_gps', 10)
        self.timer = self.create_timer(1.0, self.publish_gps_data)

        # Initialize serial connection
        self.serial_port = '/dev/ttyUSB2'  # Replace with your serial port
        self.baud_rate = 9600
        self.ser = serial.Serial(self.serial_port, baudrate=self.baud_rate, timeout=1)
        self.target_distance_threshold = 5.0

        # Start a separate thread for reading from the serial port
        self.thread = threading.Thread(target=self.read_serial_data)
        self.thread.start()

        self.latest_coordinates = None

        

    def read_serial_data(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='replace').strip()
                coordinates = parse_nmea_sentence(line)
                if coordinates:
                    self.latest_coordinates = coordinates
            except Exception as e:
                self.get_logger().error(f'Error reading serial data: {e}')


    def publish_gps_data(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_frame'

        if self.latest_coordinates:
            lat, lon, alt = self.latest_coordinates
            x, y, z = geodetic_to_cartesian(lat, lon, alt)

            # Assign parsed and converted data to the message
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt

            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing GPS data: Latitude={msg.latitude}, Longitude={msg.longitude}, Altitude={msg.altitude}, Cartesian Coordinates: x={x}, y={y}, z={z}')
        else:
            self.get_logger().warn('No GPS data available')


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
        #self.publisher_velocity = self.create_publisher(Twist, '/mi_desktop_48_b0_2d_5f_bf_41/cmd_vel', 10)
        self.publisher_velocity = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.gps_navigation)  # Adjust the timer for desired frequency
        # self.timer = self.create_timer(0.1, self.navigate)
        self.speed_lv = 3.0
        
        self.linear = Vector3(0.5, 0.0, 0.0)
        self.angular = Vector3(0.0, 0.0, 0.0)
        self.current_position = Vector2(0.0, 0.0)

        self.is_initial_position = Vector2(0.0,0.0)
        self.current_heading = 0.0
        # Flag zone
        self.turning_flag = 0
        self.turning_finish = 0
        self.update_target_flag = 0
        self.target_number = 0

        self.turning_yaw = 0.0
        self.yaw_init = 0.0
        self.yaw_last = 0.0
        self.MAX_SPEED = 16
        self.target_distance_threshold = 5.0
        # Subscribe to the distance sensor topic
        # self.create_subscription(HeadTofPayload, '/mi_desktop_48_b0_2d_5f_bf_41/head_tof_payload', self.obstacle_callback, 10)
        # self.create_subscription(Range, '/mi_desktop_48_b0_2d_5f_bf_41/ultrasonic_payload', self.obstacle_callback, 10)
        self.is_moving = True  # State to check if the robot should be moving

        self.max_capacity_position_queue = 20
        self.position_queue = deque(maxlen=self.max_capacity_position_queue)  # Fixed-size queue to store x, y positions
        self.current_position = None  # Will be set by GPS callback
        self.yaw_threshould = 15 / 180 * math.pi

        self.prev_judgeangle = 0.0

        
        self.target_position_queue = [Vector2(0.0, 10.0), Vector2(10.0,10.0 ), Vector2(10.0, 0), (0,0)] # Target coordinates (x_t, y_t)
        self.target_position = self.target_position_queue[0] #Vector2(-2414588.30, 5369302.21)  
        # self.target_position = Vector3(-2414588.30, 5369302.21, 0.0)

        
        # subscribe to ODOM
        self.odom_subscription = self.create_subscription(
            Odometry,

            #'/mi_desktop_48_b0_2d_5f_bf_41/odom_out',  # Replace with your actual odometry topic name # mi
            '/odom',
            self.odom_callback,
            10)

        # Subscribe to the GPS data
        # self.gps_subscription = self.create_subscription(
        #     NavSatFix,
        #     '/diy_gps',
        #     self.gps_callback,
        #     10)

    def update_target_position(self):
        self.target_number += 1
        self.target_position = self.target_position_queue[self.target_number]
        self.get_logger().info(f"Reached target No.{self.target_number}, heading to next target")
        self.update_target_flag = 0

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        quaternion = msg.pose.pose.orientation
        # self.current_position = (position.x, position.y)
        # Convert quaternion to yaw angle
        self.current_heading = self.quaternion_to_yaw(quaternion)
        self.get_logger().info(f'Received Odometry: current_heading - {self.current_heading}')
        

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


    # def gps_callback(self, msg):
    #     """Callback function to update the current position from GPS data."""
    #     lat = msg.latitude
    #     lon = msg.longitude
    #     alt = msg.altitude

    #     # Convert geodetic coordinates to Cartesian if needed
    #     x, y, z = geodetic_to_cartesian(lat, lon, alt)

    #     self.current_position = Vector2(x, y)

    #     # TO DO: ADD ODOMETRY FOR CHECKING WHETHER ROBOT IS MOVING OR NOT

    #     self.position_queue.append(self.current_position)
    #     self.get_logger().info(f'Current GPS Position: x={x}, y={y}, z={z}')

    def distance_to_goal(self):
        target_vector = Vector2(
            self.target_position.x - self.current_position.x,
            self.target_position.y - self.current_position.y,
        )

        distance_to_target = math.sqrt(target_vector.x ** 2 + target_vector.y ** 2)
        self.get_logger().info(f'Current distance to target:{distance_to_target}')
        return distance_to_target


    def gps_navigation(self):
        # check the GPS signal
        # if self.current_position is None:
        #     self.get_logger().warn('Current position not set. Waiting for GPS data...')
        #     return

        distance_to_target = self.distance_to_goal()
        
        # Stop if close enough to the target
        if distance_to_target < self.target_distance_threshold:
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
        if len(self.position_queue) == self.max_capacity_position_queue and self.turning_flag == 0:
            # adjust yaw Turing
            self.turning_flag = 1

            # turning initialization
            self.yaw_init = self.current_heading
            self.get_logger().info(f"Yaw_init is: {self.yaw_init}")

            # calculate turning_yaw
            self.turning_yaw = self.calculate_turning_yaw()

            # turing with initial angular velocity (+- 0.2)
            self.adjust_yaw(self.turning_yaw)
            # time.sleep(0.1)


        if self.turning_flag == 1:
            self.turning()

        # turning finish, clear queue
        if self.turning_finish == 1:
            self.position_queue = deque()
            self.turning_finish = 0
            self.turning_flag = 0



    def turning(self):
        

        # turning started, from yaw_init + adjust_yaw(turning_yaw)
        self.yaw_last = self.current_heading
        #wtf Judge sumetimes very large
        judge_func = abs((self.yaw_last - self.yaw_init) - self.turning_yaw)
        self.get_logger().info(f"Yaw_last is: {self.yaw_last}")
        self.get_logger().info(f"Judget angle is: {judge_func}")

        if judge_func > self.yaw_threshould:
            if judge_func > self.prev_judgeangle:
                self.turning_yaw = -self.turning_yaw
            #while judge_func >= self.yaw_threshould:
            self.adjust_yaw(self.turning_yaw)
            # time.sleep(0.1)

        if judge_func <= self.yaw_threshould:
            self.turning_finish = 1
        # Renew prev judge angle
        self.prev_judgeangle = judge_func


    def calculate_turning_yaw(self):
        # Calculate vector from the last position in the queue to the target
            last_position = self.position_queue[-1]
            vector_to_target = Vector2(
                self.target_position.x - last_position.x,
                self.target_position.y - last_position.y
            )

            # Calculate vector from the first position in the queue to the last position
            first_position = self.position_queue[0]
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
            if (vec1.x != 0 or vec2.x != 0) and (vec1.y != 0 or vec2.y!= 0):
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
            msg.angular.z = 0.4 * sign
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
        #self.update_target_flag = 1
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
    # gps_publisher = GPSPublisher()

    executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(gps_publisher)
    executor.add_node(continuous_publisher)
    #executor.spin()

    # Start moving forward
    #continuous_publisher.go_forward()
    # Call back
    #message = continuous_publisher.msg
    '''
    continuous_publisher.odom_callback(message)
    continuous_publisher.gps_callback(message)
    '''

    try:
        executor.spin()
    finally:
        # gps_publisher.destroy_node()
        # gps_subscriber.destroy_node()
        continuous_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
