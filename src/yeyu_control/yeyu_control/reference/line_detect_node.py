# 라인 검출
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

# line_detect_node
class LineDetectNode(Node):
    def __init__(self):
        super().__init__('line_detect_node')

        self.bridge = CvBridge()
				
				# ros2 launch turtlebot3_bringup camera.launch.py 킬 때 발행되는 토픽
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            '/camera/line_detected',
            10
        )

        self.get_logger().info('Line Detect Node started.')
        self.get_logger().info('Subscribe: /camera/image_raw')
        self.get_logger().info('Publish  : /camera/line_detected')

		# OpenCV 영상 처리
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        frame = cv2.flip(frame, -1)

        height, width, _ = frame.shape  # 이미지의 높이,너비,채널 수(640,480,3) 반환 -> height,width,_

				# 이미지의 아래쪽 40%만 잘라서 씀 -> roi(관심영역)
        roi = frame[int(height * 0.6):height, 0:width]
						          #ㄴ세로방향60%~맨아래   ㄴ가로방향전체

				# 흑백 이미지로 변경 (컬러는 3채널,흑백은 1채널) -> gray
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

				# 흑백이미지를 이진 이미지로 변환(0 아니면 255)  -> binary
        _, binary = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
					 #ㄴthreshold(문턱값)=80 기준 두 그룹으로       ㄴ반전이진화] 픽셀<80(검은라인)->흰색 / 픽셀>80(흰바닥)->검은색

				# 이진 이미지에서 윤곽선 찾기(by findContours) -> contours
        contours, _ = cv2.findContours(     
            binary,                  # 입력 이진 이미지
            cv2.RETR_EXTERNAL,       # 가장 바깥쪽 윤곽선만 검출
            cv2.CHAIN_APPROX_SIMPLE  # 간단 압축 저장
        )

				# 윤곽선 하나 이상 있으면
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)   # 윤곽선들 중 가장 면적이 큰 윤곽선 선택

            area = cv2.contourArea(largest_contour)     # 선택된 윤곽선 면적 계

						# 작은 노이즈 제거 
            if area > 500:
                M = cv2.moments(largest_contour)   # 모멘트 계산 ; 라인의 중심점 계산 위해 

                if M['m00'] != 0:    # 윤곽선 면적 0 아닐때(나눌거기때문)
                    cx = int(M['m10'] / M['m00'])   # 중심점 x좌표 -> cx
                    cy = int(M['m01'] / M['m00'])   # 중심점 y좌표 -> cy

										# roi 영역에 윤곽선과 중심점 그리기
                    cv2.drawContours(roi, [largest_contour], -1, (0, 255, 0), 2)   # 검출된 라인 윤곽선 그리기
                    cv2.circle(roi, (cx, cy), 8, (0, 0, 255), -1)                   # 라인의 중심점 빨간 원 그리기
															                  # B  G  R

                    error = cx - int(width / 2)             # 라인의 중심과 화면 중심 차이 계산 -> error

                    self.get_logger().info(f'Line center: {cx}, Error: {error}')

        frame[int(height * 0.6):height, 0:width] = roi            # 수정된 roi를 원본이미지의 frame영역에 넣기

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_msg.header = msg.header

        self.image_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)

    node = LineDetectNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()