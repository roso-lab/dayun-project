import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 解析和打印LiDAR数据
def read_lidar_data(point_cloud):
    """
    解析PointCloud2消息中的LiDAR数据并返回x, y, z的坐标列表。
    """
    fmt = 'fffI'  # 对应x, y, z和intensity
    point_step = point_cloud.point_step
    num_points = point_cloud.height * point_cloud.width

    x_data = []
    y_data = []
    z_data = []

    for i in range(num_points):
        offset = i * point_step
        x, y, z, intensity = struct.unpack_from(fmt, point_cloud.data, offset)
        x_data.append(x)
        y_data.append(y)
        z_data.append(z)
    
    return x_data, y_data, z_data

# 订阅并处理LiDAR数据
class LidarListener(Node):

    def __init__(self):
        super().__init__('lidar_listener')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rslidar_points',  # 替换为你的LiDAR数据话题
            self.lidar_callback,
            10)
        self.subscription  # 防止被垃圾回收

        # 初始化图形
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion()  # 开启交互模式
        plt.show()

    def lidar_callback(self, msg):
        # 解析数据
        x_data, y_data, z_data = read_lidar_data(msg)

        # 清除旧数据
        self.ax.clear()

        # 绘制新的点云数据
        self.ax.scatter(x_data, y_data, z_data, c=z_data, cmap='viridis', marker='.')

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        # 更新图形
        plt.draw()
        plt.pause(0.001)  # 短暂暂停以更新图形

def main(args=None):
    rclpy.init(args=args)
    lidar_listener = LidarListener()
    try:
        rclpy.spin(lidar_listener)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
