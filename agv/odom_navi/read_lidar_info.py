import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
from sensor_msgs.msg import PointCloud2, PointField

def read_lidar_data(point_cloud):
    """
    解析PointCloud2消息中的LiDAR数据并打印出每个点的坐标和强度。
    """
    # 定义格式：'fffI'表示3个float和1个unsigned int，分别对应x, y, z和intensity
    fmt = 'fffI'
    
    # 每个点的字节数
    point_step = point_cloud.point_step
    
    # 总的点数量
    num_points = point_cloud.height * point_cloud.width
    
    # 解析所有的点
    for i in range(num_points):
        offset = i * point_step
        x, y, z, intensity = struct.unpack_from(fmt, point_cloud.data, offset)
        print(f"x: {x:.2f}, y: {y:.2f}, z: {z:.2f}, intensity: {intensity}")

# 示例：假设你从ROS2话题或者bag文件中得到了一个PointCloud2对象，命名为point_cloud
# point_cloud = ...

# 调用函数处理这个PointCloud2对象
# read_lidar_data(point_cloud)

class LidarListener(Node):

    def __init__(self):
        super().__init__('lidar_listener')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.lidar_callback,
            10)
        self.subscription  # 防止被垃圾回收

    def lidar_callback(self, msg):
        read_lidar_data(msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_listener = LidarListener()
    rclpy.spin(lidar_listener)
    lidar_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
