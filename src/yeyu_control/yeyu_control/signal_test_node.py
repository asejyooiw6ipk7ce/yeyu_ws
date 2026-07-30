# test_signal_color.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class SignalColorTest(Node):
    def __init__(self):
        super().__init__('signal_color_test')
        self.bridge = CvBridge()
        self.GREEN_LOWER = np.array([35, 40, 40])
        self.GREEN_UPPER = np.array([90, 255, 255])
        self.SIGNAL_PIXEL_THRESHOLD = 500
        self.create_subscription(Image, '/camera/image_raw', self.on_camera, 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_flipped', 10)  # 추가

    def on_camera(self, msg):
        try:
                # rgb8 / bgr8 등 cv_bridge가 지원하는 표준 포맷
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'변환 실패 (encoding={msg.encoding}): {e}')
            return


        cv_image = cv2.flip(cv_image, -1)

        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        out_msg.header = msg.header  # 원본 타임스탬프/frame_id 유지 (권장)
        self.image_pub.publish(out_msg)


        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        h, w, _ = cv_image.shape
        center_pixel = hsv[h//2, w//2]
        self.get_logger().info(f'중앙 픽셀 HSV: {center_pixel}')   # 임시 디버그용

        mask = cv2.inRange(hsv, self.GREEN_LOWER, self.GREEN_UPPER)
        count = cv2.countNonZero(mask)
        self.get_logger().info(f'green_count={count}')
        
def main():
    rclpy.init()
    rclpy.spin(SignalColorTest())

if __name__ == '__main__':
    main()