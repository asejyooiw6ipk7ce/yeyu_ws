import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdomPrinter(Node):
    def __init__(self):
        super().__init__('odom_printer')
        self.create_subscription(Odometry, '/odom', self.cb, 10)

    def cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z))
        linear_x = msg.twist.twist.linear.x
        print(f"yaw: {yaw:.3f} rad | linear_x: {linear_x:.3f} m/s")
        self.get_logger().info(f"yaw: {yaw:.3f} rad | linear_x: {linear_x:.3f} m/s", throttle_duration_sec=1.0)  # 1초에 한 번만 출력

rclpy.init()
rclpy.spin(OdomPrinter())