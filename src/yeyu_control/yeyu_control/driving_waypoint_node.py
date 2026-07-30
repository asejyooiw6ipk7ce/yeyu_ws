import math
import os
import yaml
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter , ParameterValue , ParameterType
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from yeyu_msgs.msg import DrivingStatus
from yeyu_control.driving_mode import DrivingMode
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import numpy as np

NAV_ARRIVAL_TRANSITIONS = {
    DrivingMode.NAV_TO_SIGNAL: DrivingMode.SIGNAL_WAIT,
    DrivingMode.NAV_TO_ACCEL: DrivingMode.ACCEL_ZONE,
    DrivingMode.NAV_TO_PARKING: DrivingMode.PARKING,
    
}


class DrivingNode(Node):
    def __init__(self):
        super().__init__('driving_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---  waypoints 로드 ---
        wp_path = os.path.join(
            get_package_share_directory('yeyu_waypoint_nav'),
            'waypoints',
            'waypoint2.yaml'
        )
        with open(wp_path) as f:
            self.waypoints = yaml.safe_load(f)['waypoints']

        self.pending_resume_wp = None
        self.green_count = 0
        self.blue_count = 0

        # --- Nav2 액션 클라이언트 ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None

        # --- 구독/발행 ---
        # self.create_subscription(LaserScan, '/scan', self.on_lidar, 10)
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.on_camera, 10)
        self.param_client = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.pub_led = self.create_publisher(String, '/led_command', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(DrivingStatus, '/driving_status', 10)
        self.image_pub = self.create_publisher(CompressedImage, '/camera/image_flipped', 10)

        # --- 초기 상태: 첫 웨이포인트(직각주차)로 출발 ---
        self.mode = DrivingMode.NAV_TO_START
        self.wp_index = 0
        self.set_speed(0.05)
        self.send_waypoint(self.waypoints[0])   # ①


        # --- CvBridge: ROS Image <-> OpenCV(np.ndarray) 변환기 ---
        self.bridge = CvBridge()

        # --- HSV 색상 범위 --- 

        self.GREEN_LOWER = np.array([35, 40, 40])
        self.GREEN_HIGHER = np.array([90, 255, 255])

        self.BLUE_LOWER = np.array([95, 40, 30])
        self.BLUE_HIGHER = np.array([130, 255, 255])     

        self.SIGNAL_PIXEL_THRESHOLD = 300
        self.SPEED_SIGN_PIXEL_THRESHOLD = 300

        # 선언하고 스레드 실행 

    # ================= Nav2 제어 =================
    def check_tf_and_start(self):
        try:
            self.tf_buffer.lookup_transform(
                'map','base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            self.get_logger().info('TF 안정화 확인됨 , 첫 웨이포인트 전송')
            self.startup_timer.cancel()
            self.send_waypoint(self.waypoints[0])
        except Exception as e:
            self.get_logger().info(f'TF 아직 준비 안 됨 재시도: {e}')

    def send_waypoint(self, wp):

        self.get_logger().info(f'[send_waypoint] target={wp}')
        self.nav_client.wait_for_server()
        self.get_logger().info('[send_waypoint] action server ready, sending goal')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp['x']
        goal.pose.pose.position.y = wp['y']
        qz = math.sin(wp['yaw'] / 2.0)
        qw = math.cos(wp['yaw'] / 2.0)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        future = self.nav_client.send_goal_async(goal)
        self.get_logger().info('[send_goal_async] 호출됨') # goal 보냄   

        future.add_done_callback(self.on_goal_response) # goal 보내자마자 콜백 호출함
        self.get_logger().info('[on_goal] 호출됨') 


    def on_goal_response(self, future):
        goal_handle = future.result()
        self.get_logger().info('[on_goal_response] callback 호출됨')   # goal 받아서 콜백 호출됨 

        if not goal_handle.accepted:
            self.get_logger().warn('경로 목표가 거부됨')
            return

        
        self.get_logger().info('[on_goal_response] goal accepted!')    # nav2가 goal을 받음 
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)
        self.get_logger().info('[on_goal_response] result callback 등록 완료')  # goal 받아서 결과 콜백 호출함 

    def pause_nav(self):
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()

    def resume_nav(self, wp):
        self.send_waypoint(wp)

    def on_nav_result(self, future):
        self.get_logger().info('[on_nav_result] callback 호출됨')   # 결과 콜백 호출됨
        result = future.result()
        status = result.status
        self.get_logger().info(f'[on_nav_result] status={status}') # 결과에 대한 값 호출 (4는 성공)

        if status == GoalStatus.STATUS_SUCCEEDED:

            if self.wp_index == 0:   # wp1에 막 도착한 경우 (wp_index는 0-based, wp1=index 0)
                self.wp_index = 1
                self.mode = DrivingMode.NAV_TO_SIGNAL
                self.send_waypoint(self.waypoints[1])   # 자동으로 wp2까지 이어서 이동
                return
            if self.wp_index == 3:   # wp4에 막 도착한 경우 
                self.wp_index = 4
                self.mode = DrivingMode.NAV_TO_PARKING
                self.set_speed(0.15)
                self.send_waypoint(self.waypoints[4])   # 자동으로 wp5까지 이어서 이동
                return
            
            next_mode = NAV_ARRIVAL_TRANSITIONS.get(self.mode)
            if next_mode is not None:
                self.get_logger().info(f'{self.mode.name} -> {next_mode.name} 모드 전환')
                self.mode = next_mode

            else:
                
                self.get_logger().warn(f'예상치 못한 도착 콜백, 현재 mode={self.mode.name}')

        elif status == GoalStatus.STATUS_CANCELED:
            # 우리가 pause_nav()로 취소한 경우 (예: 장애물 감지 등)
            self.get_logger().info('경로 취소됨 (의도된 정지일 수 있음, mode는 다른 콜백이 관리)')

        elif status == GoalStatus.STATUS_ABORTED:
            # Nav2가 경로를 못 찾거나 중간에 포기한 경우
            self.get_logger().warn(f'주행 실패(ABORTED), 현재 mode={self.mode.name}')
            # TODO: 재시도 로직 필요 (SR-F-026 recovery behavior와 연계)

        else:
            # STATUS_UNKNOWN 등 예상 밖의 상태
            self.get_logger().warn(f'예상치 못한 nav 상태: {status}')                  # ④

    # # ================= LiDAR: 장애물 감지 =================

    # def on_lidar(self, msg):
    #     front_slice = msg.ranges[len(msg.ranges)//2 - 15 : len(msg.ranges)//2 + 15]
    #     if min(front_slice) < 0.20 and self.mode not in (
    #             DrivingMode.OBSTACLE_RESPONSE, DrivingMode.PARKING,
    #             DrivingMode.SIGNAL_WAIT, DrivingMode.ACCEL_ZONE):
    #         self.pending_resume_wp = self.waypoints[self.wp_index]     # 복귀 지점 기억
    #         self.pause_nav()                                            # ②
    #         self.mode = DrivingMode.OBSTACLE_RESPONSE
    #         self.pub_led.publish(String(data='ON'))
    #         self.create_timer(3.0, self.on_led_blink_ready)

    # def on_led_blink_ready(self):
    #     self.pub_led.publish(String(data='BLINK'))
    #     # TODO: LiDAR 재검사로 장애물이 실제로 치워졌는지 확인 후에만 resume하는 게 안전
    #     self.resume_nav(self.pending_resume_wp)                        # ③
    #     self.mode = DrivingMode.NAV_TO_SIGNAL

    # # ================= 카메라: 직각주차/신호/가속 =================

    def on_camera(self, msg):
        try: 
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg , desired_encoding= 'bgr8')
        except Exception as e :
            self.get_logger().warn(f'cv_bridge 변환 실패: {e}')

            return
        
        flipped = cv2.flip(cv_image, -1)

        try:
            out_msg = self.bridge.cv2_to_compressed_imgmsg(flipped, dst_format='jpg')
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f'republish 실패: {e}')

        if self.mode == DrivingMode.SIGNAL_WAIT: # wp2에서 신호 인식 
            color = self.detect_signal_color(flipped)
            if color == 'green':
                self.green_count += 1
                if self.green_count >= 3:
                    self.green_count =0
                    self.wp_index = 2
                    self.mode = DrivingMode.NAV_TO_ACCEL 
                    self.send_waypoint(self.waypoints[2]) #wp3로 감
            else:
                    self.green_count = 0


        elif self.mode == DrivingMode.ACCEL_ZONE: #wp3에서 표지판 인식
            color = self.detect_speed_sign(flipped)
            if color == 'blue':
                self.blue_count += 1
                if self.blue_count >= 3:
                    self.blue_count =0
                    self.wp_index = 3
                    self.mode = DrivingMode.NAV_TO_PARKING
                    self.set_speed(0.22) 
                    self.send_waypoint(self.waypoints[3]) #wp4로 감
            else:
                    self.blue_count = 0
            

        elif self.mode == DrivingMode.PARKING: #wp5에서 aruco탐색
            pose = self.detect_aruco_pose(flipped)
            if pose is not None and aligned(pose):
                self.wp_index = 5 
                self.send_waypoint(self.waypoints[5])  # wp6로 감
                self.mode = DrivingMode.NAV_TO_END 



    def detect_aruco_pose(self, cv_image):
        return
    
    def reached_stop_line(self):
            return

    def detect_signal_color(self, cv_image):   

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.GREEN_LOWER, self.GREEN_HIGHER)
        count = cv2.countNonZero(mask)
        self.get_logger().info(f'green_count={count}')

        if count > self.SIGNAL_PIXEL_THRESHOLD:
            return 'green'
        else:
            return 'unknown'

    def detect_speed_sign(self, cv_image):

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.BLUE_LOWER, self.BLUE_HIGHER)
        count = cv2.countNonZero(mask)
        self.get_logger().info(f'blue_count={count}')

        if count > self.SPEED_SIGN_PIXEL_THRESHOLD:
            return 'blue'
        else:
            return 'unknown'

    def set_speed(self, speed):
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('controller_server 파라미터 서비스 응답 없음')
            return

        param = Parameter()
        param.name = 'FollowPath.max_vel_x'
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value =speed
        )

        req = SetParameters.Request()
        req.parameters = [param]

        future = self.param_client.call_async(req)
        future.add_done_callback(self.on_accel_response)

    def on_accel_response(self, future):
        try:
            result = future.result()
            ok = result.results[0].successful
            self.get_logger().info(f'[accelerate_to] 속도 변경 {"성공" if ok else "실패"}')
        except Exception as e: 
            self.get_logger().warn(f'[accelerate_to] 응답 처리 실패: {e}')



            




def main(args=None):
    rclpy.init(args=args)
    node = DrivingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()