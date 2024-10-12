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

        # subscribe to ODOM
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  # Replace with your actual odometry topic name
            self.odom_callback,
            10)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        quaternion = msg.pose.pose.orientation
        # Convert gps position
        self.get_logger().info(f'Current Odm Position: x={position.x}, y={position.y}, z={position.z}')
        self.current_heading = self.quaternion_to_yaw(quaternion)
        self.get_logger().info(f'Current yaw: x={self.current_heading}')

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
