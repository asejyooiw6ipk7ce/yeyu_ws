import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from yeyu_msgs.msg import FunctionStatus

class FunctionNode(Node):
    def __init__(self):
        super().__init__('function_node')
        self.lost_count = 0
        self.create_subscription(Image, '/camera/image_raw', self.on_camera, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(FunctionStatus, '/line_status', 10)

    def on_camera(self, msg):
        # TODO: HSV 마스크 + 컨투어 + PID
        center_found = True  # placeholder

        status = FunctionStatus()
        status.course = 'S_CURVE'  # 또는 'CRANK'
        if center_found:
            self.lost_count = 0
            status.status = 'TRACKING'
        else:
            self.lost_count += 1
            status.status = 'LOST' if self.lost_count >= 5 else 'RECOVERING'  # SR-F-009
        status.lost_frame_count = self.lost_count
        self.pub_status.publish(status)

def main(args=None):
    rclpy.init(args=args)
    node = FunctionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()