import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UltrasonicSubscriber(Node):

    def __init__(self):
        super().__init__('ultrasonic_subscriber')
        self.subscription = self.create_subscription(
            Range,
            '/airsbot_ultrasonic_front_left',  # 这里的主题名称应该与ROS 2系统中发布的主题名称一致
            self.listener_callback,
            10)
        self.subscription  # 防止变量被垃圾回收

    def listener_callback(self, msg):
        self.get_logger().info(f"Received Range Data:")
        self.get_logger().info(f"  Timestamp: {msg.header.stamp.sec} seconds, {msg.header.stamp.nanosec} nanoseconds")
        self.get_logger().info(f"  Frame ID: {msg.header.frame_id}")
        self.get_logger().info(f"  Radiation Type: {msg.radiation_type}")
        self.get_logger().info(f"  Field of View: {msg.field_of_view} radians")
        self.get_logger().info(f"  Min Range: {msg.min_range} meters")
        self.get_logger().info(f"  Max Range: {msg.max_range} meters")
        self.get_logger().info(f"  Detected Range: {msg.range} meters")

def main(args=None):
    rclpy.init(args=args)
    ultrasonic_subscriber = UltrasonicSubscriber()
    rclpy.spin(ultrasonic_subscriber)
    ultrasonic_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()