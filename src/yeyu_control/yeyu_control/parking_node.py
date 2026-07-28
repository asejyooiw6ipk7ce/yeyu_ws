import math
import os
import yaml
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from yeyu_msgs.msg import DrivingStatus
from sensor_msgs.msg import CompressedImage  
from yeyu_control.driving_mode import DrivingMode
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import numpy as np

from enum import Enum
from dataclasses import dataclass
from typing import Optional
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from cv_bridge import CvBridge, CvBridgeError


NAV_ARRIVAL_TRANSITIONS = {
    DrivingMode.NAV_TO_START: DrivingMode.NAV_TO_PARKING,
    DrivingMode.NAV_TO_PARKING: DrivingMode.PARKING,
    DrivingMode.NAV_TO_SIGNAL: DrivingMode.SIGNAL_WAIT,
    DrivingMode.NAV_TO_ACCEL: DrivingMode.ACCEL_ZONE,
}

class ParkingState(Enum):
    SEARCH_MARKER = 'SEARCH_MARKER'
    ALIGN_AXIS = 'ALIGN_AXIS'
    FINAL_APPROACH = 'FINAL_APPROACH'
    DONE = 'DONE'
    RECOVERY = 'RECOVERY'
    FAILED = 'FAILED'
 
 
@dataclass
class ArucoObservation:
    marker_id: int
    x_m: float
    y_m: float
    z_m: float
    bearing_rad: float
    center_x: float
    center_y: float

class DrivingNode(Node):
    def __init__(self):
        super().__init__('driving_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- 1. waypoints 로드 ---
        wp_path = os.path.join(
            get_package_share_directory('yeyu_waypoint_nav'),
            'waypoints',
            'waypoint1.yaml'
        )
        with open(wp_path) as f:
            self.waypoints = yaml.safe_load(f)['waypoints']
        self.wp_index = 0
        self.pending_resume_wp = None
        self.green_count = 0

        # --- 2. Nav2 액션 클라이언트 ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None

        # --- 3. T자 주차(ArUco) 파라미터 ---
        self._declare_parking_parameters()
        self._load_parking_parameters()
    
        self.bridge = CvBridge()

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

        self.latest_observation: Optional[ArucoObservation] = None
        self.last_marker_time = self.get_clock().now() - Duration(seconds=999.0)


    
        self.parking_state = ParkingState.SEARCH_MARKER
        self.parking_state_enter_time = self.get_clock().now()


        self.parking_retry_count = 0

        self.last_tracking_angular_z = 0.0
    
        self.aruco_dict, self.aruco_params, self.aruco_detector = self._create_aruco_detector(
            self.aruco_dictionary_name
        )

        # --- 4. 구독/발행 ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        #self.create_subscription(LaserScan, '/scan', self.on_lidar, 10)    #-> on_lidar 필요할 때 주석 해제
        self.create_subscription(Image, self.image_topic, self.on_camera, sensor_qos)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, sensor_qos)
        self.pub_led = self.create_publisher(String, '/led_command', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/parking_debug_image', 10)
        #self.debug_pub = self.create_publisher(CompressedImage, '/parking_debug_image', 10)
        self.pub_status = self.create_publisher(DrivingStatus, '/driving_status', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_flipped', 10)
 
        # 주차 제어 루프 (10Hz). mode가 PARKING일 때만 실제로 동작함.
        self.create_timer(1.0 / 10.0, self.parking_control_loop)
 
        # --- 5. 초기 상태: 첫 웨이포인트(직각주차)로 출발 ---
        self.mode = DrivingMode.NAV_TO_PARKING
        self.mode = DrivingMode.PARKING   #테스트
        #self.send_waypoint(self.waypoints[0])   # ①

       # ================= 파라미터 =================
    def _declare_parking_parameters(self):
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
 
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('target_marker_id', 4)          # 4번 칸 마커
        self.declare_parameter('marker_size_m', 0.10)
 
        # 정렬/정지 목표
        self.declare_parameter('parking_stop_distance_m', 0.18)     # 마커 벽에서 멈출 거리
        self.declare_parameter('lateral_tolerance_m', 0.035)
        self.declare_parameter('bearing_tolerance_rad', 0.060)
 
        # 속도/게인
        self.declare_parameter('max_approach_speed_mps', 0.070)
        self.declare_parameter('min_approach_speed_mps', 0.018)
        self.declare_parameter('final_approach_speed_mps', 0.025)
        self.declare_parameter('max_angular_speed_rps', 0.80)
        self.declare_parameter('max_reverse_speed_mps', 0.040)
        self.declare_parameter('k_z', 0.45)
        self.declare_parameter('k_bearing', 1.80)
 
        # 탐색/복구
        self.declare_parameter('search_angular_speed_rps', 0.28)
        self.declare_parameter('marker_lost_timeout_sec', 0.70)
        self.declare_parameter('stale_stop_timeout_sec', 0.25)
        self.declare_parameter('recovery_backup_time_sec', 1.20)
        self.declare_parameter('max_retry_count', 3)
        self.declare_parameter('max_parking_time_sec', 60.0)
        self.declare_parameter('enable_debug_image', True)
 
    def _load_parking_parameters(self):
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.aruco_dictionary_name = self.get_parameter('aruco_dictionary').value
        self.target_marker_id = int(self.get_parameter('target_marker_id').value)
        self.marker_size_m = float(self.get_parameter('marker_size_m').value)
 
        self.parking_stop_distance_m = float(self.get_parameter('parking_stop_distance_m').value)
        self.lateral_tolerance_m = float(self.get_parameter('lateral_tolerance_m').value)
        self.bearing_tolerance_rad = float(self.get_parameter('bearing_tolerance_rad').value)
 
        self.max_approach_speed_mps = float(self.get_parameter('max_approach_speed_mps').value)
        self.min_approach_speed_mps = float(self.get_parameter('min_approach_speed_mps').value)
        self.final_approach_speed_mps = float(self.get_parameter('final_approach_speed_mps').value)
        self.max_angular_speed_rps = float(self.get_parameter('max_angular_speed_rps').value)
        self.max_reverse_speed_mps = float(self.get_parameter('max_reverse_speed_mps').value)
        self.k_z = float(self.get_parameter('k_z').value)
        self.k_bearing = float(self.get_parameter('k_bearing').value)
 
        self.search_angular_speed_rps = float(self.get_parameter('search_angular_speed_rps').value)
        self.marker_lost_timeout_sec = float(self.get_parameter('marker_lost_timeout_sec').value)
        self.stale_stop_timeout_sec = float(self.get_parameter('stale_stop_timeout_sec').value)
        self.recovery_backup_time_sec = float(self.get_parameter('recovery_backup_time_sec').value)
        self.max_retry_count = int(self.get_parameter('max_retry_count').value)
        self.max_parking_time_sec = float(self.get_parameter('max_parking_time_sec').value)
        self.enable_debug_image = self._get_bool_parameter('enable_debug_image')

    def _get_bool_parameter(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ['true', '1', 'yes', 'on']
        return bool(value)

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
        self.nav_client.wait_for_server()
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
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        self.get_logger().info('[on_goal_response] callback 호출됨')   # 추가 ①
        if not goal_handle.accepted:
            self.get_logger().warn('경로 목표가 거부됨')
            return
        self.get_logger().info('[on_goal_response] goal accepted!')    # 추가 ②
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)
        self.get_logger().info('[on_goal_response] result callback 등록 완료')  # 추가 ③

    def pause_nav(self):
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()

    def resume_nav(self, wp):
        self.send_waypoint(wp)

    def on_nav_result(self, future):
        self.get_logger().info('[on_nav_result] callback 호출됨')   # 추가 ④
        result = future.result()
        status = result.status
        self.get_logger().info(f'[on_nav_result] status={status}') 

        if status == GoalStatus.STATUS_SUCCEEDED:
                next_mode = NAV_ARRIVAL_TRANSITIONS.get(self.mode)
                if next_mode is not None:
                    self.get_logger().info(f'{self.mode.name} -> {next_mode.name} 모드 전환')
                    self.mode = next_mode


                    if self.mode == DrivingMode.NAV_TO_PARKING:
                        self.wp_index = 1
                        self.send_waypoint(self.waypoints[self.wp_index])
                else:
                    # 지금 mode가 NAV_TO_* 계열이 아닌데 도착 콜백이 온 경우 (비정상 상황)
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
    def on_camera_info(self, msg: CameraInfo) -> None:
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('CameraInfo received. ArUco pose estimation enabled.')
 
    # 카메라가 실제로 어떤 인코딩(픽셀 포맷)으로 이미지를 보내든 bgr8로 바꾸기 위한 변환표.
    # 예전에는 cv_bridge에게 무조건 'bgr8'로 바꿔달라고 요청했는데,
    # 카메라(camera_ros)가 bgr8이 아닌 다른 포맷(bayer 등)으로 보내면
    # cv_bridge가 변환에 실패해서 예외를 던지고, on_camera가 바로 return 되어
    # 디버그 이미지 자체가 발행되지 않는 문제가 있었음.
    # 그래서 원본 그대로(passthrough) 받은 뒤, 실제 인코딩을 보고
    # 우리가 직접 bgr8로 변환하도록 바꿈.
    _BAYER_CODES = {
        'bayer_rggb8': cv2.COLOR_BayerRG2BGR,
        'bayer_bggr8': cv2.COLOR_BayerBG2BGR,
        'bayer_gbrg8': cv2.COLOR_BayerGB2BGR,
        'bayer_grbg8': cv2.COLOR_BayerGR2BGR,
    }

    def _to_bgr8(self, frame: np.ndarray, encoding: str) -> Optional[np.ndarray]:
        # msg.encoding에 적힌 실제 포맷 이름을 보고 bgr8로 변환하는 함수.
        # (passthrough로 받으면 cv_bridge가 변환을 안 해주기 때문에 여기서 우리가 직접 해줘야 함)
        enc = encoding.lower()

        if enc == 'bgr8':
            return frame  # 이미 원하는 포맷이면 그대로 반환
        if enc == 'bgra8':
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        if enc == 'rgb8':
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if enc == 'rgba8':
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        if enc == 'mono8':
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if enc in self._BAYER_CODES:
            return cv2.cvtColor(frame, self._BAYER_CODES[enc])

        # 위 목록에 없는 포맷이면 변환 방법을 모르니, 경고를 남기고 None을 반환해서
        # 호출한 쪽(on_camera)이 이번 프레임 처리를 건너뛰게 함
        self.get_logger().warn(f'지원하지 않는 image encoding: {encoding}')
        return None

    def on_camera(self, msg: Image) -> None:
        # -------------------------------------------------------------
        # TODO(향후 구현 예정): 신호등 대기 / 가속 구간 로직
        # 아래는 원래 스텁 코드에 있던 구현으로, 주차(PARKING) 이후
        # 미션이 이어질 경우 여기에 맞춰 구현 예정. 지금은 미구현 상태.
        #
        # elif self.mode == DrivingMode.NAV_TO_SIGNAL:
        #     if reached_stop_line():                                     # ⑥
        #         self.pause_nav()
        #         self.mode = DrivingMode.SIGNAL_WAIT
        #
        # elif self.mode == DrivingMode.SIGNAL_WAIT:
        #     color = detect_signal_color(frame)
        #     if color == 'green':
        #         self.green_count += 1
        #         if self.green_count >= 3:
        #             self.wp_index = 3
        #             self.resume_nav(self.waypoints[3])                  # ⑧
        #             self.mode = DrivingMode.NAV_TO_ACCEL
        #             self.green_count = 0
        #     else:
        #         self.green_count = 0
        #
        # elif self.mode == DrivingMode.NAV_TO_ACCEL:
        #     if detect_speed_sign(frame):                                 # ⑨
        #         self.mode = DrivingMode.ACCEL_ZONE
        #         self.accelerate_to(0.2)
        # -------------------------------------------------------------
 
        if self.mode != DrivingMode.PARKING:
            return  # 주차 모드가 아니면(신호등/가속 로직 미구현) 인식할 필요 없음
 
        try:
            # desired_encoding='bgr8'로 강제 변환을 요청하면, 카메라가 bgr8이 아닌
            # 포맷(bayer 등)으로 보낼 경우 cv_bridge가 변환을 못 해서 예외를 던짐.
            # 'passthrough'는 변환 없이 원본 그대로 받아오므로 여기서는 항상 성공하고,
            # 실제 bgr8 변환은 밑에서 우리가 직접 처리함
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as exc:
            self.get_logger().warn(f'cv_bridge conversion failed: {exc}')
            return

        # 원본 포맷(msg.encoding)을 보고 bgr8로 변환. 모르는 포맷이면 None이 반환되므로
        # 이번 프레임은 처리하지 않고 다음 프레임을 기다림
        frame = self._to_bgr8(frame, msg.encoding)
        if frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
 
        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

        observation = None
        selected_index = None       

        # 디버깅 이미지 - 마커 못 찾으면 observation = None인 상태로 _drawing_debug_image로 보냄
        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten().astype(int)
            target_indices = np.where(ids_flat == self.target_marker_id)[0]

            if len(target_indices) > 0 and self.camera_matrix is not None and self.dist_coeffs is not None:
                idx = int(target_indices[0])
                selected_index = idx  
                try:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[idx]], self.marker_size_m, self.camera_matrix, self.dist_coeffs
                    )
                    corner = corners[idx].reshape(4, 2)
                    center = corner.mean(axis=0)
                    tvec = tvecs[0][0]

                    x_m = float(tvec[0])
                    z_m = float(tvec[2])
                    bearing_rad = math.atan2(x_m, max(z_m, 1e-6))

                    observation = ArucoObservation(
                        marker_id=int(ids_flat[idx]),
                        x_m=x_m,
                        y_m=float(tvec[1]),
                        z_m=z_m,
                        bearing_rad=bearing_rad,
                        center_x=float(center[0]),
                        center_y=float(center[1]),
                    )
                    self.latest_observation = observation
                    self.last_marker_time = self.get_clock().now()
                except Exception as exc:
                    self.get_logger().warn(f'ArUco pose estimation failed: {exc}')
                    selected_index = None 

        if self.enable_debug_image:
            debug_frame = self._draw_debug_image(frame, corners, ids, observation, selected_index)
            try:
                debugout_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                debugout_msg.header = msg.header
                self.debug_pub.publish(debugout_msg)
            except CvBridgeError as exc:
                self.get_logger().warn(f'debug image publish failed: {exc}')
        
    def _create_aruco_detector(self, dictionary_name: str):
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('cv2.aruco 모듈이 없습니다. OpenCV 설치 상태를 확인하세요.')
        if not hasattr(cv2.aruco, dictionary_name):
            valid_names = [n for n in dir(cv2.aruco) if n.startswith('DICT_')]
            raise RuntimeError(
                f'지원하지 않는 ArUco dictionary: {dictionary_name}. 사용 가능 예: {valid_names[:10]}'
            )
 
        aruco_dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dictionary_name)
        )
 
        if hasattr(cv2.aruco, 'DetectorParameters'):
            aruco_params = cv2.aruco.DetectorParameters()
        else:
            aruco_params = cv2.aruco.DetectorParameters_create()
 
        aruco_detector = None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
 
        return aruco_dict, aruco_params, aruco_detector
 
    # ================= T자 주차 상태 기계 =================
    def _reset_parking_state(self):
        self.parking_state = ParkingState.SEARCH_MARKER
        self.parking_state_enter_time = self.get_clock().now()
        self.parking_start_time = self.get_clock().now()     #주차시간 제한용
        self.parking_retry_count = 0
        self.latest_observation = None
 
    def _transition_parking(self, new_state: ParkingState, reason: str = ''):
        if self.parking_state == new_state:
            return
        old = self.parking_state
        self.parking_state = new_state
        self.parking_state_enter_time = self.get_clock().now()
        self.get_logger().info(f'PARKING STATE: {old.value} -> {new_state.value}. reason={reason}')
 
    def _elapsed(self, start_time) -> float:
        return (self.get_clock().now() - start_time).nanoseconds * 1e-9
 
    def _clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, value))
 
    def _get_tracking_observation(
        self,
        lost_reason: str
    ) -> Optional[ArucoObservation]:
        if self.latest_observation is None:
            self.pub_cmd.publish(Twist())
            self._start_parking_recovery(lost_reason)
            return None

        observation_age = self._elapsed(self.last_marker_time)

        if observation_age > self.marker_lost_timeout_sec:
            self.pub_cmd.publish(Twist())
            self._start_parking_recovery(lost_reason)
            return None

        if observation_age > self.stale_stop_timeout_sec:
            # 완전히 놓친 건 아님 -> 일단 정지하고 재인식 대기
            self.pub_cmd.publish(Twist())
            return None

        return self.latest_observation
 
    def parking_control_loop(self):
        if self.mode != DrivingMode.PARKING:
            return

        self.publish_parking_state()   

        # 전체 주차 시간 제한
        if self._elapsed(self.parking_start_time) > self.max_parking_time_sec:
            if self.parking_state not in [ParkingState.DONE, ParkingState.FAILED]:
                self._transition_parking(ParkingState.FAILED, 'max parking time exceeded')

        if self.parking_state == ParkingState.DONE:
            return  # 이미 완료 처리됨 (on_parking_done에서 다음 웨이포인트로 이미 이동함)
 
        if self.parking_state == ParkingState.FAILED:
            self.pub_cmd.publish(Twist())
            return
 
        if self.parking_state == ParkingState.SEARCH_MARKER:
            self._handle_parking_search()
        elif self.parking_state == ParkingState.ALIGN_AXIS:
            self._handle_parking_align()
        elif self.parking_state == ParkingState.FINAL_APPROACH:
            self._handle_parking_final()
        elif self.parking_state == ParkingState.RECOVERY:
            self._handle_parking_recovery()
 
    def _handle_parking_search(self):
        obs = self._get_tracking_observation('re-check during search')
        if obs is not None:
            self._transition_parking(ParkingState.ALIGN_AXIS, 'marker acquired')
            return
            
        elapsed = self._elapsed(self.parking_state_enter_time)

        # 마지막으로 돌던 방향과 반대로 초기 탐색 방향을 잡음
        if self.last_tracking_angular_z > 0.0:
            initial_direction = -1.0
        elif self.last_tracking_angular_z < 0.0:
            initial_direction = 1.0
        else:
            initial_direction = 1.0

        if elapsed < 2.0:
            direction = initial_direction
        else:
            search_phase = int((elapsed - 2.0) / 3.0)
            direction = (
                -initial_direction
                if search_phase % 2 == 0
                else initial_direction
            )

        angular_z = direction * min(self.search_angular_speed_rps, 0.12)    
 
        cmd = Twist()
        cmd.angular.z = angular_z
        self.pub_cmd.publish(cmd)
 
    def _handle_parking_align(self):
        obs = self._get_tracking_observation('marker lost during align')
        if obs is None:
            # self._start_parking_recovery('marker lost during align')
            return
 
        aligned = (
            abs(obs.x_m) <= self.lateral_tolerance_m
            and abs(obs.bearing_rad) <= self.bearing_tolerance_rad
        )
        if aligned:
            self._transition_parking(ParkingState.FINAL_APPROACH, 'axis aligned')
            self.pub_cmd.publish(Twist())
            return
 
        angular_z = self._clamp(-self.k_bearing * obs.bearing_rad, -0.5, 0.5)
        linear_x = 0.012 if abs(obs.bearing_rad) <= 0.12 else 0.0
 
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.pub_cmd.publish(cmd)
        self.last_tracking_angular_z = angular_z
 
    def _handle_parking_final(self):
        obs = self._get_tracking_observation('marker lost during final approach')
        if obs is None:
            # self._start_parking_recovery('marker lost during final approach')
            return
 
        if abs(obs.x_m) > self.lateral_tolerance_m * 2 or abs(obs.bearing_rad) > self.bearing_tolerance_rad * 2:
            self._transition_parking(ParkingState.ALIGN_AXIS, 'drifted out of alignment')
            return
 
        if obs.z_m <= self.parking_stop_distance_m:
            self.pub_cmd.publish(Twist())
            self._transition_parking(ParkingState.DONE, 'parking distance reached')
            self._on_parking_done()
            return
 
        distance_error = obs.z_m - self.parking_stop_distance_m
        linear_x = max(self.min_approach_speed_mps, self.k_z * distance_error)
        linear_x = self._clamp(linear_x, 0.0, self.final_approach_speed_mps)
        angular_z = self._clamp(-self.k_bearing * obs.bearing_rad, -0.3, 0.3)
 
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.pub_cmd.publish(cmd)
 
    def _start_parking_recovery(self, reason: str):
        self.parking_retry_count += 1
        self._transition_parking(ParkingState.RECOVERY, reason)
 
    def _handle_parking_recovery(self):
        elapsed = self._elapsed(self.parking_state_enter_time)
        if elapsed < self.recovery_backup_time_sec:
            cmd = Twist()
            cmd.linear.x = -self.max_reverse_speed_mps
            self.pub_cmd.publish(cmd)
            return
 
        self.pub_cmd.publish(Twist())
 
        if self.parking_retry_count >= self.max_retry_count:
            self._transition_parking(ParkingState.FAILED, 'retry count exceeded')
            return
 
        self._transition_parking(ParkingState.SEARCH_MARKER, 'recovery finished')

    def publish_parking_state(self) -> None:
        msg = DrivingStatus()
        msg.mode = self.mode.name

        if self.parking_state == ParkingState.DONE:
            msg.result = 'PASS'
        elif self.parking_state == ParkingState.FAILED:
            msg.result = 'FAIL'
        else:
            msg.result = 'IN_PROGRESS'

        obs = self.latest_observation
        msg.error_lateral = float(obs.x_m) if obs is not None else 0.0
        msg.error_distance = (
            float(obs.z_m - self.parking_stop_distance_m) if obs is not None else 0.0
        )
        msg.retry_count = self.parking_retry_count

        self.pub_status.publish(msg)
    
    def _on_parking_done(self):
        """
        주차 완료 시 호출. 다음 웨이포인트(경유점2)로 이동.
        NOTE: 원래 on_camera 함수 안에, if aligned(pose): 조건문 밑에 끼워져 있었음
        상태 머신을 나누면서 진짜 완료 시점에만 실행되도록 함
        """
        self.wp_index = 1
        self.send_waypoint(self.waypoints[1])
        self.mode = DrivingMode.NAV_TO_SIGNAL

    def _draw_debug_image(
        self,
        frame,
        corners,
        ids,
        observation: Optional[ArucoObservation],
        selected_index: Optional[int],
    ):
        debug = frame.copy()

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)

        h, w = debug.shape[:2]

        cv2.line(debug, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)

        state_text = f'state={self.parking_state.value}'
        cv2.putText(
            debug,
            state_text,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

        if observation is not None:
            cx = int(observation.center_x)
            cy = int(observation.center_y)

            cv2.circle(debug, (cx, cy), 5, (0, 0, 255), -1)

            info_1 = (
                f'id={observation.marker_id} '
                f'z={observation.z_m:.2f}m '
                f'x={observation.x_m:.2f}m'
            )

            info_2 = (
                f'bearing={observation.bearing_rad:.2f} '
                f'retry={self.parking_retry_count}/{self.max_retry_count}'
            )

            cv2.putText(
                debug,
                info_1,
                (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                debug,
                info_2,
                (10, 85),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            if (
                selected_index is not None
                and self.camera_matrix is not None
                and self.dist_coeffs is not None
            ):
                try:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[selected_index]],
                        self.marker_size_m,
                        self.camera_matrix,
                        self.dist_coeffs,
                    )

                    cv2.drawFrameAxes(
                        debug,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvecs[0],
                        tvecs[0],
                        self.marker_size_m * 0.5,
                    )
                except Exception:
                    pass
        else:
            cv2.putText(
                debug,
                'target marker not found',
                (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        return debug


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = DrivingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()