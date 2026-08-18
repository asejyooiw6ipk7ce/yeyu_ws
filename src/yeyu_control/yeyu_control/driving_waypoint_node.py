import math
import os
import yaml
import rclpy
import threading
from enum import Enum
from dataclasses import dataclass
from typing import Optional

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA, Float32, Bool   # [병합] Float32, Bool은 A(장애물/비상LED)에서
from std_srvs.srv import Trigger
from yeyu_msgs.msg import DrivingStatus, AudioCommand
from yeyu_msgs.msg import IRSensor
from yeyu_msgs.srv import StartRetry
from yeyu_control.states.driving_mode import DrivingMode
from yeyu_control.states.parking_state import ParkingState
from yeyu_control.states.stage_result import StageResult
from yeyu_control.states.linecourse_state import LineCourseState
from yeyu_control.states.linecourse_state import SCourseState
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener


# ================= wp 도착 시 자동 모드 전환 테이블 =================
NAV_ARRIVAL_TRANSITIONS = {
    DrivingMode.NAV_TO_START: DrivingMode.TRACING_CRANK,
    DrivingMode.NAV_TO_S: DrivingMode.TRACING_S,
    DrivingMode.NAV_TO_SIGNAL: DrivingMode.SIGNAL_WAIT,
    DrivingMode.NAV_TO_ACCEL: DrivingMode.ACCEL_ZONE,
    DrivingMode.NAV_TO_PARKING: DrivingMode.PARKING,
}

LED_COLOR_MAP = {
    # 'START':       (1.0, 0.0, 0.0),          # RED (255,0,0)
    'TRACING_CRANK':  (128/255, 0.0, 1.0),      # PURPLE
    'TRACING_S':      (1.0, 20/255, 147/255),   # HOT PINK
    'SIGNAL_WAIT': (11/255, 1.0, 11/255),    # GREEN
    'ACCEL_ZONE':  (0.0, 1.0, 1.0),          # BLUE
    'PARKING':     (1.0, 70/255, 0.0),       # YELLOW
    'END':         (1.0, 0.0, 0.0),          # RED
}

STAGE_TTS_LABELS = {   # 발음 가능한 한글 라벨을 별도로 관리
    'TRACING_CRANK': '크랭크 코스',
    'TRACING_S': 'S자 코스',
    'SIGNAL_WAIT': '신호대기',
    'ACCEL_ZONE': '가속구간',
    'PARKING': '직각주차',
}


class RunPhase(Enum):
    MAIN = 'MAIN'
    RETRY = 'RETRY'


RETRY_ENTRY = {
    'TRACING_CRANK': {'wp_index': 0, 'mode': DrivingMode.NAV_TO_START},
    'TRACING_S': {'wp_index': 2, 'mode': DrivingMode.NAV_TO_S},
    'SIGNAL_WAIT': {'wp_index': 5, 'mode': DrivingMode.NAV_TO_SIGNAL},
    'ACCEL_ZONE':  {'wp_index': 6, 'mode': DrivingMode.NAV_TO_ACCEL},
    'PARKING':     {'wp_index': 8, 'mode': DrivingMode.NAV_TO_PARKING},
}

HOME_WP_INDEX = 10


@dataclass
class ArucoObservation:
    marker_id: int
    x_m: float
    y_m: float
    z_m: float
    bearing_rad: float
    marker_normal_yaw_rad: float
    center_x: float
    center_y: float
    image_width: int
    image_height: int
    area: float


class DrivingNode(Node):
    def __init__(self):
        super().__init__('driving_node')

        # --- 락(Lock) 모음 ---
        self.data_lock = threading.Lock()
        self.crank_lock = threading.Lock()
        self.s_course_lock = threading.Lock()
        self.camera_lock = threading.Lock()          # [수정] on_camera에서 쓰는데 초기화가 빠져 있던 것 추가
        self._nav_result_lock = threading.Lock()      # [수정] 마찬가지로 초기화 누락 추가

        self.wp_index = 0
        self.green_count = 0
        self.blue_count = 0
        self.signal_wait_enter_time = None
        self.accel_zone_enter_time = None
        self.ACCEL_GRACE_PERIOD_SEC = 2.0
        self.accel_zone_max_speed = 0.0
        self.ACCEL_TARGET_SPEED = 0.18
        

        self.ACCEL_SUSTAIN_SEC = 0.5
        self.ACCEL_DIP_TOLERANCE_SEC = 0.15
        self.accel_last_below_time = None
        self.accel_sustain_start = None
        self.current_linear_x = 0.0   # [수정] accel_zone_check_loop에서 쓸 최신 오도메트리 속도 저장용

        # --- [병합: A] 장애물(초음파) 감지 상태 ---
        self.OBSTACLE_STOP_DISTANCE_CM = 5.0
        self.is_handling_obstacle = False
        self.obstacle_saved_wp_index = None
        self.obstacle_blink_count = 0
        self.obstacle_blink_timer = None

        # --- [병합: A] costmap inflation_radius 관련 ---
        self.DEFAULT_LOCAL_INFLATION_RADIUS = 0.15
        self.DEFAULT_GLOBAL_INFLATION_RADIUS = 0.15
        self.ACCEL_LOCAL_INFLATION_RADIUS = 0.3
        self.ACCEL_GLOBAL_INFLATION_RADIUS = 0.3

        # --- [수정] 누락돼 있던 pending/버퍼 변수 초기화 ---
        self._nav_result_pending = None
        self.latest_frame = None
        self.latest_frame_header = None
        self.accel_timer = None   # _process_speed_sign / accel_zone_check_loop에서 참조하는데 초기화 누락
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- 구간별 성공/실패 결과 저장소 ---
        self.stage_results = {
            'NAV_WAYPOINT': StageResult.IN_PROGRESS,
            'TRACING_CRANK': StageResult.IN_PROGRESS,
            'TRACING_S': StageResult.IN_PROGRESS,
            'SIGNAL_WAIT': StageResult.IN_PROGRESS,
            'ACCEL_ZONE': StageResult.IN_PROGRESS,
            'PARKING': StageResult.IN_PROGRESS,
        }

        self.run_phase = RunPhase.MAIN
        self.retry_target = None
        self.is_estopped = False

        wp_path = os.path.join(
            get_package_share_directory('yeyu_waypoint_nav'),
            'waypoints',
            'waypoint4.yaml'
        )
        with open(wp_path) as f:
            self.waypoints = yaml.safe_load(f)['waypoints']

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        self.nav_fail_count = 0

        self.retry_srv = self.create_service(
            StartRetry, '/start_retry', self.on_start_retry_request)

        self.estop_srv = self.create_service(
            Trigger, '/emergency_stop', self.on_emergency_stop_request)

        self._declare_parking_parameters()
        self._load_parking_parameters()

        self.bridge = CvBridge()

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

        self.latest_observation: Optional[ArucoObservation] = None
        self.last_marker_time = self.get_clock().now() - Duration(seconds=999.0)
        self.parking_state = ParkingState.SEARCH_MARKER
        self.parking_state_enter_time = self.get_clock().now()
        self.parking_start_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now() - Duration(seconds=999.0)
        self.parking_retry_count = 0
        self.last_tracking_angular_z = 0.0

        self.aruco_dict, self.aruco_params, self.aruco_detector = self._create_aruco_detector(
            self.aruco_dictionary_name
        )

        # --- 크랭크 코스 상태 ---
        self.crank_state = LineCourseState.LINE_FOLLOWING
        self.crank_state_enter_time = self.get_clock().now()
        self.crank_line_lost_since = None
        self.last_meaningful_ir = (0, 1, 0)
        self.crank_turn_start_yaw = 0.0
        self.crank_turn_target_delta = 0.0
        self.current_yaw = 0.0
        with self.data_lock:
            self.current_x = None
            self.current_y = None
        self.ir_l = 0
        self.ir_c = 0
        self.ir_r = 0

        self.crank_turn_pending_delta = 0.0
        self.crank_creep_start_time = None
        self.crank_creep_target_sec = 0.0

        self.s_line_offset = None
        self.s_line_last_seen_time = self.get_clock().now() - Duration(seconds=999.0)
        self.s_last_valid_offset = None

        # --- 구독/발행 ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(CompressedImage, self.image_topic, self.on_camera, sensor_qos)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, sensor_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(IRSensor, 'sensor_bridge/ir_state', self.on_ir_sensor, 10)
        self.create_subscription(   # [병합: A] 초음파 장애물 거리
            Float32, 'sensor_bridge/obstacle_distance_cm', self.on_obstacle_distance, 10)

        self.param_client = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.local_costmap_param_client = self.create_client(   # [병합: A]
            SetParameters, '/local_costmap/local_costmap/set_parameters')
        self.global_costmap_param_client = self.create_client(   # [병합: A]
            SetParameters, '/global_costmap/global_costmap/set_parameters')

        self.pub_led = self.create_publisher(ColorRGBA, 'sensor_bridge/rgb_cmd', 10)
        self.pub_emergency_led = self.create_publisher(Bool, 'sensor_bridge/emergency_led_cmd', 10)   # [병합: A]
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(DrivingStatus, '/driving_status', 10)
        self.image_pub = self.create_publisher(   # [수정] 주석 처리되어 있었으나 on_camera에서 실제로 사용 중이라 복구
            CompressedImage, '/camera/image_flipped/compressed', 10)
        self.parking_debug_pub = self.create_publisher(   # [수정] 마찬가지로 _process_aruco에서 사용 중이라 복구
            CompressedImage, '/parking_debug_image/compressed', 10)
        self.s_course_debug_pub = self.create_publisher(   # 추가
            CompressedImage, '/s_course_debug_image/compressed', 10)
        self.audio_pub = self.create_publisher(AudioCommand, '/audio/command', 10)

        # --- HSV 색상 범위 ---
        self.GREEN_LOWER = np.array([35, 40, 40])
        self.GREEN_HIGHER = np.array([90, 255, 255])
        self.BLUE_LOWER = np.array([95, 80, 50])
        self.BLUE_HIGHER = np.array([130, 255, 255])
        self.SIGNAL_PIXEL_THRESHOLD = 300
        self.SPEED_SIGN_PIXEL_THRESHOLD = 300

        # --- 초기 상태: 첫 웨이포인트로 출발 ---
        self.mode = DrivingMode.NAV_TO_START
        self.set_speed(0.13)
        self.set_inflation_radius_pair(   # [병합: A]
            self.DEFAULT_LOCAL_INFLATION_RADIUS, self.DEFAULT_GLOBAL_INFLATION_RADIUS)
        self.startup_timer = self.create_timer(0.5, self.on_startup)

        # ========== 타이머 ===========
        self.timer_period = 1.0 / max(self.control_rate_hz, 0.5)

        self.crank_timer = None
        self.parking_timer = None

        self.s_course_timer = None
        self.vision_timer = self.create_timer(self.timer_period, self.camera_processing_loop)
        self.nav_result_timer = self.create_timer(self.timer_period, self._nav_result_loop)

    # ================= 파라미터 =================
    def _declare_parking_parameters(self):
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('target_marker_id', 5)
        self.declare_parameter('marker_size_m', 0.10)
        self.declare_parameter('pre_dock_distance_m', 0.50)
        self.declare_parameter('pre_dock_tolerance_m', 0.06)
        self.declare_parameter('parking_stop_distance_m', 0.18)
        self.declare_parameter('final_lateral_limit_m', 0.035)
        self.declare_parameter('final_bearing_limit_rad', 0.060)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('max_approach_speed_mps', 0.070)
        self.declare_parameter('min_approach_speed_mps', 0.018)
        self.declare_parameter('final_approach_speed_mps', 0.025)
        self.declare_parameter('max_angular_speed_rps', 0.80)
        self.declare_parameter('max_reverse_speed_mps', 0.040)
        self.declare_parameter('k_z', 0.45)
        self.declare_parameter('k_bearing', 1.80)
        self.declare_parameter('k_final_bearing', 1.20)
        self.declare_parameter('k_final_lateral', 0.90)
        self.declare_parameter('search_angular_speed_rps', 0.28)
        self.declare_parameter('marker_lost_timeout_sec', 2.0)
        self.declare_parameter('stale_stop_timeout_sec', 0.8)
        self.declare_parameter('recovery_backup_time_sec', 1.20)
        self.declare_parameter('max_retry_count', 3)
        self.declare_parameter('max_parking_time_sec', 60.0)
        self.declare_parameter('enable_debug_image', True)
        self.declare_parameter('log_throttle_sec', 1.0)
        self.declare_parameter('enable_motion', True)

    def _load_parking_parameters(self):
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.aruco_dictionary_name = self.get_parameter('aruco_dictionary').value
        self.target_marker_id = int(self.get_parameter('target_marker_id').value)
        self.marker_size_m = float(self.get_parameter('marker_size_m').value)
        self.pre_dock_distance_m = float(self.get_parameter('pre_dock_distance_m').value)
        self.pre_dock_tolerance_m = float(self.get_parameter('pre_dock_tolerance_m').value)
        self.parking_stop_distance_m = float(self.get_parameter('parking_stop_distance_m').value)
        self.final_lateral_limit_m = float(self.get_parameter('final_lateral_limit_m').value)
        self.final_bearing_limit_rad = float(self.get_parameter('final_bearing_limit_rad').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.max_approach_speed_mps = float(self.get_parameter('max_approach_speed_mps').value)
        self.min_approach_speed_mps = float(self.get_parameter('min_approach_speed_mps').value)
        self.final_approach_speed_mps = float(self.get_parameter('final_approach_speed_mps').value)
        self.max_angular_speed_rps = float(self.get_parameter('max_angular_speed_rps').value)
        self.max_reverse_speed_mps = float(self.get_parameter('max_reverse_speed_mps').value)
        self.k_z = float(self.get_parameter('k_z').value)
        self.k_bearing = float(self.get_parameter('k_bearing').value)
        self.k_final_bearing = float(self.get_parameter('k_final_bearing').value)
        self.k_final_lateral = float(self.get_parameter('k_final_lateral').value)
        self.search_angular_speed_rps = float(self.get_parameter('search_angular_speed_rps').value)
        self.marker_lost_timeout_sec = float(self.get_parameter('marker_lost_timeout_sec').value)
        self.stale_stop_timeout_sec = float(self.get_parameter('stale_stop_timeout_sec').value)
        self.recovery_backup_time_sec = float(self.get_parameter('recovery_backup_time_sec').value)
        self.max_retry_count = int(self.get_parameter('max_retry_count').value)
        self.max_parking_time_sec = float(self.get_parameter('max_parking_time_sec').value)
        self.enable_debug_image = self._get_bool_parameter('enable_debug_image')
        self.log_throttle_sec = float(self.get_parameter('log_throttle_sec').value)
        self.enable_motion = self._get_bool_parameter('enable_motion')

        self.CRANK_LINEAR_SPEED = 0.05    # 0.03 -> 0.05
        self.CRANK_STEER_ANGULAR = 0.12   # 0.12 -> 0.15 -> 0.12
        self.CRANK_RECOVERY_SPEED = 0.02
        self.CRANK_TURN_ANGULAR_SPEED = 0.30
        self.CRANK_TURN_TOLERANCE_RAD = math.radians(3.0)
        self.CRANK_TURN_TIMEOUT_SEC = 8.0
        self.CRANK_LINE_GRACE_SEC = 0.2
        self.CRANK_ARRIVAL_TOLERANCE_M = 0.15
        self.CRANK_LINE_LOST_TIMEOUT_SEC = 30.0
        self.CRANK_CREEP_DISTANCE_M = 0.055 # 0.07 -> 0.06 -> 0.07 -> 0.06 -> 0.055

        self.S_ROI_TOP_RATIO = 0.6        # 0.85 -> 0.6 : 하단 40%만 봄
        self.S_LINE_BLACK_THRESHOLD = 60 
        self.S_LINE_PIXEL_MIN = 50     #100 -> 50 : 50픽셀 이상의 픽셀이 있어야 라인있음 판정
        self.S_LINEAR_SPEED_MAX = 0.10
        self.S_LINEAR_SPEED_MIN = 0.04
        self.S_ANGULAR_GAIN = 0.9
        self.S_ANGULAR_MAX = 0.6
        self.S_OFFSET_DEADBAND = 0.05
        self.S_LINE_LOST_TIMEOUT_SEC = 30.0 #1.2 -> 30.0
        self.S_ARRIVAL_TOLERANCE_M = 0.3   # 0.10 -> 0.15 -> 0.2 -> 0.3
        self.S_OFFSET_JUMP_LIMIT = 0.6   # 0.4 -> 0.8 -> 0.6
        self.S_BOTTOM_BAND_HEIGHT_RATIO = 0.3   # 채택된 컨투어의 bounding box 중 하단 몇 %만으로 cx 계산할지

        self.vision_enable = False

    # ================= 시작 시퀀스 (LED 구독자 대기) =================
    def on_startup(self):
        self.startup_timer.cancel()
        self._start_check_timer = self.create_timer(0.3, self._try_start)

    def _try_start(self):
        if (self.pub_led.get_subscription_count() == 0
            or self.audio_pub.get_subscription_count() == 0
            or self.status_pub.get_subscription_count() == 0):
            self.get_logger().warn('[LED] 구독자, [audio]구독자 [gui]구독자 대기 중...')
            return

        if not self.tf_buffer.can_transform(
            'map', 'base_link' , rclpy.time.Time(),
            timeout=Duration(seconds=0.1)
        ):
            self.get_logger().warn('[Nav2] map -> base_link tf 대기중 ')
            return
        self._start_check_timer.cancel()
        self._pending_start_timer = self.create_timer(0.5, self._do_start)

    def _do_start(self):   
        self._pending_start_timer.cancel()
        self.set_led('TRACING_CRANK')
        self.notify_tts('크랭크 코스를 시작합니다')
        self.wp_index = 1
        self.mode = DrivingMode.TRACING_CRANK

        self._report_stage('NAV_WAYPOINT', StageResult.IN_PROGRESS, '') 
        self._report_stage('TRACING_CRANK', StageResult.IN_PROGRESS, '')  # [수정] 출발 시점에 경로 진행중 발행 누락 보완

        self._reset_crank_state()
        if self.crank_timer is None:
            self.crank_timer = self.create_timer(self.timer_period, self.crank_control_loop)

    # ================= 구간 결과 보고 (공통 헬퍼) =================
    def _publish_status(self, mode: str, result: str, reason: str = ''):   # [병합: A] 판정 없이 상태만 알리는 헬퍼
        msg = DrivingStatus()
        msg.mode = mode
        msg.result = result
        msg.reason = reason
        msg.wp_index = self.wp_index
        with self.data_lock:
            obs = self.latest_observation
        msg.error_lateral = float(obs.x_m) if obs is not None else 0.0
        msg.error_distance = float(obs.z_m - self.parking_stop_distance_m) if obs is not None else 0.0
        msg.retry_count = self.parking_retry_count
        self.status_pub.publish(msg)

    def _report_stage(self, stage: str, result: StageResult, reason: str = ''):
        """구간 결과를 stage_results에 반영하고, 동시에 DrivingStatus로도 즉시 발행"""
        self.stage_results[stage] = result
        self._publish_status(stage, result.name, reason)

    # ================= 재시험 서비스 콜백 =================
    def on_start_retry_request(self, request, response):
        target = request.target

        if target not in RETRY_ENTRY:
            response.accepted = False
            response.message = f'알 수 없는 재시험 대상: {target}'
            self.get_logger().warn(f'[RETRY] 잘못된 요청: {target}')
            return response

        if self.run_phase == RunPhase.RETRY:
            response.accepted = False
            response.message = '이미 재시험이 진행 중입니다.'
            self.get_logger().warn('[RETRY] 이미 재시험 진행 중, 요청 거부')
            return response

        if self.is_estopped:
            response.accepted = False
            response.message = '비상정지 상태입니다. 재시험을 시작할 수 없습니다.'
            return response

        entry = RETRY_ENTRY[target]
        self.run_phase = RunPhase.RETRY
        self.retry_target = target
        self._report_stage(target, StageResult.IN_PROGRESS, '재시험 시작')
        self.green_count = 0
        self.blue_count = 0

        self.wp_index = entry['wp_index']
        self.mode = entry['mode']

        if target == 'PARKING':
            self._reset_parking_state()
        elif target == 'TRACING_CRANK':    
            self._reset_crank_state()
        elif target == 'TRACING_S':
            self._reset_s_course_state()

        self.get_logger().info(f'[RETRY] {target} 재시험 시작 → wp{entry["wp_index"] + 1}로 이동')
        label = STAGE_TTS_LABELS.get(target, target)
        self.notify_tts(f'{label} 구간을 재시험합니다.')
        self.send_waypoint(self.waypoints[self.wp_index])

        response.accepted = True
        response.message = 'OK'
        return response

    def _finish_retry(self):
        self.get_logger().info(f'[RETRY] {self.retry_target} 재시험 판정 완료, 홈으로 복귀')
        label = STAGE_TTS_LABELS.get(self.retry_target, self.retry_target)
        self.notify_tts(f'{label} 재시험을 완료했습니다. 도착점으로 이동합니다.')
        self.wp_index = HOME_WP_INDEX
        self.mode = DrivingMode.NAV_TO_END
        self._publish_status('NAV_TO_END', 'IN_PROGRESS', '재시험 종료, 도착점으로 복귀 중')   # [병합: A]
        self.send_waypoint(self.waypoints[HOME_WP_INDEX])

    def _announce_retry_result(self, target: str):
        result = self.stage_results[target]
        self._publish_status(target, result.name, '')
        self._publish_status('RETRY_COMPLETE', '', target)   # [병합: A] GUI 모드 카드가 안 갱신되는 문제 방지용

        self.get_logger().info(f'[RETRY RESULT] {target}: {result.name}')

        if result == StageResult.PASS:
            label = STAGE_TTS_LABELS.get(target, target)
            self.notify_tts(f'{label} 재시험 결과, 통과했습니다.')
        else:
            label = STAGE_TTS_LABELS.get(target, target)
            self.notify_tts(f'{label} 재시험 결과, 실패했습니다.')

    # ================= 비상정지 서비스 콜백 =================
    def on_emergency_stop_request(self, request, response):
        self.get_logger().error('[E_STOP] 비상정지 요청 수신')
        self.is_estopped = True

        self.pause_nav()

        self.mode = DrivingMode.E_STOP
        self.run_phase = RunPhase.MAIN
        self.retry_target = None

        for _ in range(5):
            self.cmd_pub.publish(Twist())

        self.set_led('END')
        self.notify_tts('비상정지가 실행되었습니다.')

        response.success = True
        response.message = '비상정지 완료'
        return response

    # ================= Nav2 제어 =================
    def send_waypoint(self, wp):
        if self.is_estopped:
            self.get_logger().warn('[send_waypoint] 비상정지 상태, 이동 명령 무시')
            return
        self.get_logger().info(f'[send_waypoint] target={wp}')
        self.nav_client.wait_for_server()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        qz = math.sin(wp['yaw'] / 2.0)
        qw = math.cos(wp['yaw'] / 2.0)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('경로 목표가 거부됨')
            return
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)

    def pause_nav(self):
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()

    def resume_nav(self, wp):
        self.send_waypoint(wp)

    def on_nav_result(self, future):
        if self.is_estopped:
            return
        result = future.result()
        status = result.status
        error = result.result
        # ! status를 숫자 대신 문자열로 보기 편하게 바꿈
        # self.get_logger().info(f'[on_nav_result] status={status}')
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }.get(status, f'UNKNOWN({status})')
        self.get_logger().info(f'[on_nav_result] status={status_name}')
    
        with self._nav_result_lock:
            self._nav_result_pending = status

    def _nav_result_loop(self):
        # ? 비상정지 상태면 대기 중이던 nav 결과를 그냥 버림(뒤늦게 도착한 nav가 mode를 바꾸거나 send_waypoint 호출하는거 방지)
        if self.is_estopped:
            with self._nav_result_lock:
                self._nav_result_pending = None
            return

        with self._nav_result_lock:
            if self._nav_result_pending is None:
                return
            status = self._nav_result_pending
            self._nav_result_pending = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.nav_fail_count = 0

            if self.wp_index == 4:   # wp5(미로 시작점) 도착 → wp6(신호등 진입점)로
                self.wp_index = 5
                self.mode = DrivingMode.NAV_TO_SIGNAL
                self.send_waypoint(self.waypoints[5])
                return

            if self.wp_index == 7:   # wp8 도착
                if self.accel_timer is not None:   # [수정] 가속구간 종료 시 타이머 정리 누락 보완
                    self.accel_timer.cancel()
                    self.accel_timer = None

                if self.stage_results['ACCEL_ZONE'] == StageResult.IN_PROGRESS:
                    reason = f'{self.ACCEL_SUSTAIN_SEC}초 연속 유지 실패 (최고 {self.accel_zone_max_speed:.3f} m/s)'
                    self._report_stage('ACCEL_ZONE', StageResult.FAIL, reason)
                    self.notify_tts('가속구간에서 충분히 가속하지 못했습니다.')

                if self.run_phase == RunPhase.RETRY:
                    self._finish_retry()
                    return

                self.wp_index = 8
                self.mode = DrivingMode.NAV_TO_PARKING
                self.set_speed(0.13)
                self.set_inflation_radius_pair(   # [병합: A]
                    self.DEFAULT_LOCAL_INFLATION_RADIUS, self.DEFAULT_GLOBAL_INFLATION_RADIUS)
                self._report_stage('NAV_WAYPOINT', StageResult.IN_PROGRESS, '')
                self.send_waypoint(self.waypoints[self.wp_index])
                return

            if self.wp_index == 9:   # wp10 도착 → wp11로
                self.wp_index = 10
                self.mode = DrivingMode.NAV_TO_END
                self._publish_status('NAV_TO_END', 'IN_PROGRESS', '도착점으로 이동 중')   # [병합: A]
                self.send_waypoint(self.waypoints[self.wp_index])
                return

            if self.wp_index == 10:   # wp11(최종 도착점) 도착
                if self.run_phase == RunPhase.RETRY:
                    self.get_logger().info(f'=== {self.retry_target} 재시험 종료 ===')
                    self._publish_cmd(Twist())
                    self._announce_retry_result(self.retry_target)
                    self.run_phase = RunPhase.MAIN
                    self.retry_target = None
                    return

                if self.stage_results['NAV_WAYPOINT'] == StageResult.IN_PROGRESS:
                    self._report_stage('NAV_WAYPOINT', StageResult.PASS, '전 구간 정상 도착')
                self.get_logger().info('=== 전체 코스 완료 ===')
                self._publish_cmd(Twist())
                self._announce_final_result()
                return

            next_mode = NAV_ARRIVAL_TRANSITIONS.get(self.mode)
            if next_mode is not None:
                if (self.mode == DrivingMode.NAV_TO_ACCEL
                        and self.run_phase == RunPhase.RETRY
                        and self.retry_target == 'SIGNAL_WAIT'):
                    self._finish_retry()
                    return

                self.get_logger().info(f'{self.mode.name} -> {next_mode.name} 모드 전환')
                self.mode = next_mode


                if self.mode == DrivingMode.TRACING_CRANK:
                    self.wp_index = 1
                    self.set_led('TRACING_CRANK')
                    self._report_stage('TRACING_CRANK', StageResult.IN_PROGRESS, '')
                    self._reset_crank_state()
                    if self.crank_timer is None:
                        self.crank_timer = self.create_timer(self.timer_period, self.crank_control_loop)
                elif self.mode == DrivingMode.TRACING_S:
                    self.set_led('TRACING_S')
                    self.notify_tts('S자 코스를 시작합니다')
                    self._report_stage('TRACING_S', StageResult.IN_PROGRESS, '')
                    self._reset_s_course_state()
                    if self.s_course_timer is None:
                        self.s_course_timer = self.create_timer(self.timer_period, self.s_course_control_loop)
                elif self.mode == DrivingMode.SIGNAL_WAIT:
                    self.set_led('SIGNAL_WAIT')
                    self.signal_wait_enter_time = self.get_clock().now()
                    self._report_stage('SIGNAL_WAIT', StageResult.IN_PROGRESS, '')
                elif self.mode == DrivingMode.ACCEL_ZONE:
                    self.set_led('ACCEL_ZONE')
                    self._report_stage('ACCEL_ZONE', StageResult.IN_PROGRESS, '')
                elif self.mode == DrivingMode.PARKING:
                    self.set_led('PARKING')
                    self._report_stage('PARKING', StageResult.IN_PROGRESS, '')
                    self._reset_parking_state()
                    if self.parking_timer is None:
                        self.parking_timer = self.create_timer(self.timer_period, self.parking_control_loop)
            else:
                self.get_logger().warn(f'예상치 못한 도착 콜백, 현재 mode={self.mode.name}')

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('경로 취소됨')

        elif status == GoalStatus.STATUS_ABORTED:
            self.nav_fail_count += 1
            self.get_logger().warn(f'주행 실패(ABORTED), {self.nav_fail_count}/3회')
            if self.nav_fail_count >= 3:
                reason = f'경로 탐색 3회 연속 실패 (wp{self.wp_index + 1} 지점)'
                self._report_stage('NAV_WAYPOINT', StageResult.FAIL, reason)
                self.mode = DrivingMode.E_STOP
                self.get_logger().error('경로 탐색 3회 실패, E_STOP 진입 — 경로를 찾을 수 없습니다')
                self._publish_cmd(Twist())
                self.notify_tts('경로를 찾을 수 없습니다. 임무를 일시정지합니다.')
            else:
                self.send_waypoint(self.waypoints[self.wp_index])
        else:
            self.get_logger().warn(f'예상치 못한 nav 상태: {status}')

    # ================= IR 트래킹 모듈 =================
    def on_ir_sensor(self, msg: IRSensor):
        self.ir_l = int(msg.ir_sensor_l)
        self.ir_c = int(msg.ir_sensor_c)
        self.ir_r = int(msg.ir_sensor_r)

    # ================= 카메라: 신호/표지판/ArUco 통합 콜백 (가벼움: 저장만) =================
    def on_camera(self, msg: CompressedImage):
        
        if self.is_estopped:
            return
        if not msg.data:
            return
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except (CvBridgeError, cv2.error) as e:
            self.get_logger().warn(f'cv_bridge 변환 실패: {e}')
            return

        flipped = cv2.flip(cv_image, -1)

        try:
            out_msg = self.bridge.cv2_to_compressed_imgmsg(flipped, dst_format='jpg')
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f'republish 실패: {e}')

        with self.camera_lock:
            self.latest_frame = flipped
            self.latest_frame_header = msg.header

    def camera_processing_loop(self):
        if self.vision_enable is False:
            return
        
        with self.camera_lock:
            flipped = self.latest_frame
            header = self.latest_frame_header
        if flipped is None:
            return

        if self.mode == DrivingMode.SIGNAL_WAIT:
            self._process_signal(flipped)
        elif self.mode == DrivingMode.ACCEL_ZONE:
            self._process_speed_sign(flipped)
        elif self.mode == DrivingMode.PARKING:
            self._process_aruco(flipped, header, image_width=flipped.shape[1], image_height=flipped.shape[0])   # [수정] msg.header → header
        elif self.mode == DrivingMode.TRACING_S:
            offset = self._detect_s_line_offset(flipped) # ? 왜 이건 _process가 아닌가 : 판단+행동이 들어가있지 않으므로(s_cource_control_loop에서 함)
            with self.s_course_lock:
                self.s_line_offset = offset
                if offset is not None:
                    self.s_line_last_seen_time = self.get_clock().now()

    def _process_signal(self, flipped):
        if self.stage_results['SIGNAL_WAIT'] == StageResult.FAIL:
            return

        color = self.detect_signal_color(flipped)
        if color == 'green':
            self.green_count += 1
            if self.green_count >= 3:
                self._report_stage('SIGNAL_WAIT', StageResult.PASS, '초록 신호 3프레임 연속 인식')
                self.notify_tts('신호등이 초록불로 감지되었습니다. 재출발합니다.')
                self.green_count = 0
                self._after_signal_wait_result()
                return
        else:
            self.green_count = 0

        elapsed = (self.get_clock().now() - self.signal_wait_enter_time).nanoseconds / 1e9
        if elapsed > 5.0:
            reason = f'{elapsed:.1f}초간 재출발 실패 (제한 5초)'
            self._report_stage('SIGNAL_WAIT', StageResult.FAIL, reason)
            self.get_logger().warn(f'[SIGNAL_WAIT] {reason}')
            self.notify_tts('신호대기 시간이 초과되었습니다.')
            self.green_count = 0
            self._after_signal_wait_result()

    def _after_signal_wait_result(self):
        self.wp_index = 6
        self.mode = DrivingMode.NAV_TO_ACCEL
        self._report_stage('NAV_WAYPOINT', StageResult.IN_PROGRESS, '')
        self.send_waypoint(self.waypoints[6])

    def _process_speed_sign(self, flipped):
        color = self.detect_speed_sign(flipped)
        if color == 'blue':
            self.blue_count += 1
            if self.blue_count >= 3:
                self.blue_count = 0
                self.notify_tts('가속표지판이 감지되었습니다. 제한 속도 내로 이동합니다.')
                self.mode = DrivingMode.NAV_TO_PARKING
                self.set_speed(0.22)
                self.set_inflation_radius_pair(   # [병합: A]
                    self.ACCEL_LOCAL_INFLATION_RADIUS, self.ACCEL_GLOBAL_INFLATION_RADIUS)
                self.accel_zone_enter_time = self.get_clock().now()
                self.accel_zone_max_speed = 0.0
                self.accel_sustain_start = None
                self.wp_index = 7
                self.send_waypoint(self.waypoints[self.wp_index])
                if self.accel_timer is None:
                    self.accel_timer = self.create_timer(self.timer_period, self.accel_zone_check_loop)
        else:
            self.blue_count = 0

    # ================= [병합: A] 장애물(초음파) 대응 =================
    def on_obstacle_distance(self, msg: Float32):
        if self.is_estopped or self.is_handling_obstacle:
            return
        if self.mode != DrivingMode.NAV_TO_END:
            return
        if msg.data <= self.OBSTACLE_STOP_DISTANCE_CM:
            self.get_logger().warn(f'[OBSTACLE] 장애물 감지: {msg.data:.1f} cm')
            self._start_obstacle_response()

    def _start_obstacle_response(self):
        self.is_handling_obstacle = True
        self.obstacle_saved_wp_index = self.wp_index

        self.notify_tts('장애물이 감지되었습니다. 정지합니다.')
        self.pause_nav()
        self._publish_cmd(Twist())

        self.obstacle_blink_count = 0
        self.obstacle_blink_timer = self.create_timer(0.5, self._obstacle_blink_step)

    def _obstacle_blink_step(self):
        led_on = (self.obstacle_blink_count % 2 == 0)
        self.pub_emergency_led.publish(Bool(data=led_on))
        self.obstacle_blink_count += 1

        if self.obstacle_blink_count >= 10:
            self.obstacle_blink_timer.cancel()
            self.pub_emergency_led.publish(Bool(data=False))
            self._finish_obstacle_response()

    def _finish_obstacle_response(self):
        self.is_handling_obstacle = False
        self.notify_tts('다시 출발합니다.')
        self.resume_nav(self.waypoints[self.obstacle_saved_wp_index])

    # ================= aruco 주차 =================
    def _process_aruco(self, frame, header, image_width, image_height):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        observation = None
        selected_index = None

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten().astype(int)
            selected_index = self._select_marker_index(ids_flat, corners)

            if selected_index is not None:
                observation = self._make_observation(
                    ids_flat, corners, selected_index, image_width, image_height)
                if observation is not None:
                    with self.data_lock:
                        self.latest_observation = observation
                        self.last_marker_time = self.get_clock().now()

            if self.enable_debug_image:
                debug_frame = self._draw_parking_debug_image(frame, corners, ids, observation, selected_index)
                try:
                    debugout_msg = self.bridge.cv2_to_compressed_imgmsg(debug_frame, dst_format='jpg')
                    debugout_msg.header = header
                    self.parking_debug_pub.publish(debugout_msg)
                except CvBridgeError as exc:
                    self.get_logger().warn(f'debug image publish failed: {exc}')

    def _select_marker_index(self, ids_flat: np.ndarray, corners) -> Optional[int]:
        target_indices = np.where(ids_flat == self.target_marker_id)[0]
        if len(target_indices) > 0:
            return int(target_indices[0])
        if self.target_marker_id < 0:
            areas = [abs(cv2.contourArea(c.reshape(4, 2).astype(np.float32))) for c in corners]
            return int(np.argmax(areas))
        return None

    def _make_observation(self, ids_flat, corners, selected_index, image_width, image_height) -> Optional[ArucoObservation]:
        if self.camera_matrix is None or self.dist_coeffs is None:
            self._throttled_warn('CameraInfo is not available. Docking control is waiting.')
            return None
        try:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[selected_index]], self.marker_size_m, self.camera_matrix, self.dist_coeffs)
        except Exception as exc:
            self._throttled_warn(f'ArUco pose estimation failed: {exc}')
            return None

        corner = corners[selected_index].reshape(4, 2)
        center = corner.mean(axis=0)
        area = abs(cv2.contourArea(corner.astype(np.float32)))

        tvec = tvecs[0][0]
        rvec = rvecs[0][0]
        x_m, y_m, z_m = float(tvec[0]), float(tvec[1]), float(tvec[2])
        bearing_rad = math.atan2(x_m, max(z_m, 1e-6))

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        marker_normal = rotation_matrix[:, 2].astype(np.float64)
        if marker_normal[2] > 0.0:
            marker_normal = -marker_normal
        marker_normal_yaw_rad = self._normalize_angle(
            math.atan2(float(marker_normal[0]), float(-marker_normal[2])))

        return ArucoObservation(
            marker_id=int(ids_flat[selected_index]), x_m=x_m, y_m=y_m, z_m=z_m,
            bearing_rad=bearing_rad, marker_normal_yaw_rad=marker_normal_yaw_rad,
            center_x=float(center[0]), center_y=float(center[1]),
            image_width=image_width, image_height=image_height, area=float(area))

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('CameraInfo received. ArUco pose estimation enabled.')

    def _detect_s_line_offset(self, cv_image) -> Optional[float]:
        h, w = cv_image.shape[:2]
        roi_top = int(h * self.S_ROI_TOP_RATIO)
        roi = cv_image[roi_top:h, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(gray, 0, self.S_LINE_BLACK_THRESHOLD)   #그레이스케일 + 밝기값이 0~threshold 사이인 곳은 흰색만 남기겠다(이진화)
        # # ! 자동노출로 인해 밝기가 달라지면 객체 인식이 안됨 -> 이미지 밝기 분포를 자동 분석해 threshold를 동적으로 결정
        # blur = cv2.GaussianBlur(gray, (5, 5), 0)
        # _, mask = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)  

        # 노이즈 제거 (작은 얼룩 없애기)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        offset = None
        cx_full = None   # 디버그용: 원본 이미지 좌표계에서의 중심 x
        best_contour = None

        candidates = []
        debug_candidates = [] # TODO 디버그용 , 정보 확인

        for c in contours:
            area = cv2.contourArea(c)
            if area < self.S_LINE_PIXEL_MIN:
                continue

            x, y, cw, ch = cv2.boundingRect(c)

            if y <= 3 or cw > w * 0.5:    # 위에 붙어있고 + 폭이 넓으면 벽/가구
                reason = 'top_edge' if y <= 3 else 'too_wide'
                debug_candidates.append((x, y, cw, ch, area, 0.0, reason))  # 탈락 사유 표시
                continue

            # ! 컨투어 전체가 아니라 하단 일부 밴드만으로 cx 계산
            band_h = max(1, int(ch * self.S_BOTTOM_BAND_HEIGHT_RATIO))
            band_y_start = y + ch - band_h   # 이 컨투어의 bounding box 내 하단 밴드 시작 y (ROI 좌표계)

            # 컨투어를 채운 마스크를 만들고, 그 중 하단 밴드 부분만 잘라서 무게중심 계산
            contour_mask = np.zeros(mask.shape, dtype=np.uint8)    # 아무것도 없는 까만 도화지
            cv2.drawContours(contour_mask, [c], -1, 255, -1)   # 새 도화지에 저 색종이 붙임
            band_mask = contour_mask[band_y_start:y + ch, x:x + cw]

            # ? 라인 덩어리 전체의 무게중심을 구함 (오해:특정 y줄을 딱 잘라서 보는게 아님=디버그에 나온 초록가로선의 y좌표와 연결되지않음)
            # M = cv2.moments(c)                         # 컨투어 전체의 무게중심을 구함
            M = cv2.moments(band_mask, binaryImage=True) # 컨투어의 S_BOTTOM_BAND_HEIGHT_RATIO부분만 잘라 무게중심 구함
            if M['m00'] == 0:
                continue
            cx = x + (M['m10'] / M['m00'])

            this_offset = (cx - w / 2.0) / (w / 2.0)

            # solidity 계산
            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            # TODO 디버그 텍스트 보고 주석해체
            if solidity < 0.5 : # 삐뚤삐뚤하고 구멍 많은 형태는 무시
                debug_candidates.append((x, y, cw, ch, area, solidity, 'low_solidity'))
                continue

            rejected_reason = None  

            # 직전에 알던 라인 위치와 너무 멀면 후보에서 제외
            if self.s_last_valid_offset is not None:
                if abs(this_offset - self.s_last_valid_offset) > self.S_OFFSET_JUMP_LIMIT:
                    rejected_reason = 'jump_limit' # TODO 디버그용 추가
                    #continue #TODO 디버그용 주석처리

            debug_candidates.append((x, y, cw, ch, area, solidity, rejected_reason))  # TODO 디버그용 출력
            if rejected_reason is not None:
                continue    

            # 위치 필터: ROI 안에서 얼마나 아래쪽(바닥에 가까운지)에 있는지 점수화
            bottom_y = y + ch   # 이 덩어리의 ROI 내 하단 y좌표
            candidates.append((bottom_y, c, this_offset))

        if candidates:
            # 바닥에 가장 가까운(=bottom_y가 가장 큰) 덩어리를 라인으로 채택
            candidates.sort(key=lambda t: t[0], reverse=True)
            best_contour = candidates[0][1]
            offset = candidates[0][2]
            cx_full = (offset * (w / 2.0)) + (w / 2.0)
            self.s_last_valid_offset = offset   # 성공했을 때만 "최근 유효 위치" 갱신

            # 디버그용: 채택된 컨투어의 밴드 영역 좌표도 구해둠
            bx, by, bcw, bch = cv2.boundingRect(best_contour)
            band_h = max(1, int(bch * self.S_BOTTOM_BAND_HEIGHT_RATIO))
            band_rect = (bx, by + bch - band_h, bcw, band_h)   # (x, y, w, h) — ROI 좌표계
        else:
            band_rect = None

        if self.enable_debug_image:
            # TODO 디버그용 인수에 debug_candidates 추가
            # self._draw_s_course_debug_image(cv_image, mask, roi_top, cx_full, offset, best_contour, band_rect)
            self._draw_s_course_debug_image(cv_image, mask, roi_top, cx_full, offset, best_contour, band_rect, debug_candidates)

        return offset

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z))
        self.current_linear_x = msg.twist.twist.linear.x   # [수정] accel_zone_check_loop가 쓸 최신 속도 저장

    def accel_zone_check_loop(self):
        if self.is_estopped:
            return

        if self.mode != DrivingMode.NAV_TO_PARKING or self.accel_zone_enter_time is None:
            self.accel_sustain_start = None
            self.accel_last_below_time = None
            return

        linear_x = self.current_linear_x   # [수정] 정의되지 않던 msg.twist... 대신 on_odom이 저장해둔 값 사용

        if linear_x > self.accel_zone_max_speed:
            self.accel_zone_max_speed = linear_x

        now = self.get_clock().now()

        if linear_x >= self.ACCEL_TARGET_SPEED:
            if self.accel_sustain_start is None:
                self.accel_sustain_start = now
            self.accel_last_below_time = None

            sustained = (now - self.accel_sustain_start).nanoseconds / 1e9
            if sustained >= self.ACCEL_SUSTAIN_SEC:
                if self.stage_results['ACCEL_ZONE'] == StageResult.IN_PROGRESS:
                    reason = (f'{sustained:.2f}초간 {self.ACCEL_TARGET_SPEED} m/s 이상 유지(허용오차 포함) '
                        f'(최고 {self.accel_zone_max_speed:.3f} m/s)')
                    self._report_stage('ACCEL_ZONE', StageResult.PASS, reason)
                    self.get_logger().info(f'[ACCEL_ZONE] {reason}')
                    self.notify_tts('가속구간을 규정 속도로 통과했습니다.')
                    if self.accel_timer is not None:
                        self.accel_timer.cancel()
                        self.accel_timer = None
        else:
            if self.accel_last_below_time is None:
                self.accel_last_below_time = now

            below_duration = (now - self.accel_last_below_time).nanoseconds / 1e9
            if below_duration > self.ACCEL_DIP_TOLERANCE_SEC:
                self.accel_sustain_start = None

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        with self.data_lock:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y

    def detect_signal_color(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.GREEN_LOWER, self.GREEN_HIGHER)
        count = cv2.countNonZero(mask)
        return 'green' if count > self.SIGNAL_PIXEL_THRESHOLD else 'unknown'

    def detect_speed_sign(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.BLUE_LOWER, self.BLUE_HIGHER)
        count = cv2.countNonZero(mask)
        return 'blue' if count > self.SPEED_SIGN_PIXEL_THRESHOLD else 'unknown'

    def _get_bool_parameter(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ['true', '1', 'yes', 'on']
        return bool(value)

    def _create_aruco_detector(self, dictionary_name: str):
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('cv2.aruco 모듈이 없습니다.')
        if not hasattr(cv2.aruco, dictionary_name):
            valid_names = [n for n in dir(cv2.aruco) if n.startswith('DICT_')]
            raise RuntimeError(f'지원하지 않는 ArUco dictionary: {dictionary_name}. 예: {valid_names[:10]}')
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
        if hasattr(cv2.aruco, 'DetectorParameters'):
            aruco_params = cv2.aruco.DetectorParameters()
        else:
            aruco_params = cv2.aruco.DetectorParameters_create()
        aruco_detector = None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        return aruco_dict, aruco_params, aruco_detector

    # ================= 주차 제어 루프 =================
    def parking_control_loop(self):
        if self.is_estopped:
            self._publish_cmd(Twist())
            return
        if self.mode != DrivingMode.PARKING:
            return

        if self._elapsed(self.parking_start_time) > self.max_parking_time_sec:
            if self.parking_state not in [ParkingState.DONE, ParkingState.FAILED]:
                self._transition_parking(ParkingState.FAILED, 'max parking time exceeded')

        if self.parking_state == ParkingState.DONE:
            self._publish_cmd(Twist())
            return

        if self.parking_state == ParkingState.FAILED:
            if self.stage_results['PARKING'] != StageResult.FAIL:
                reason = f'재시도 {self.parking_retry_count}/{self.max_retry_count}회 소진, 마커 정렬 실패'
                self._report_stage('PARKING', StageResult.FAIL, reason)
                self.notify_tts('직각주차에 실패했습니다.')

                if self.parking_timer is not None:
                    self.parking_timer.cancel()
                    self.parking_timer = None

                if self.run_phase == RunPhase.RETRY:
                    self._finish_retry()
                else:
                    self.wp_index = 9
                    self.mode = DrivingMode.NAV_TO_END
                    self.set_led('END')
                    self._publish_status('NAV_TO_END', 'IN_PROGRESS', '주차 실패, 도착점으로 이동 중')   # [병합: A]
                    self.send_waypoint(self.waypoints[self.wp_index])
            self._publish_cmd(Twist())
            return

        if self.parking_state == ParkingState.SEARCH_MARKER:
            self._handle_parking_search()
        elif self.parking_state == ParkingState.APPROACH_PRE_DOCK:
            self._handle_approach_pre_dock()
        elif self.parking_state == ParkingState.ALIGN_AXIS:
            self._handle_parking_align()
        elif self.parking_state == ParkingState.FINAL_APPROACH:
            self._handle_parking_final()
        elif self.parking_state == ParkingState.RECOVERY:
            self._handle_parking_recovery()

    def _get_tracking_observation(self, lost_reason: str) -> Optional[ArucoObservation]:
        with self.data_lock:
            obs = self.latest_observation
            marker_time = self.last_marker_time

        if obs is None:
            self._publish_cmd(Twist())
            self._start_parking_recovery(lost_reason)
            return None
        age = self._elapsed(marker_time)
        if age > self.marker_lost_timeout_sec:
            self._publish_cmd(Twist())
            self._start_parking_recovery(lost_reason)
            return None
        if age > self.stale_stop_timeout_sec:
            self._publish_cmd(Twist())
            self._throttled_info(f'waiting for marker reacquisition, age={age:.2f}s')
            return None
        return obs

    def _handle_parking_search(self):
        obs = self.get_valid_observation()
        if obs is not None:
            self._publish_cmd(Twist())
            self._transition_parking(ParkingState.APPROACH_PRE_DOCK, 'marker acquired')
            return

        elapsed = self._elapsed(self.parking_state_enter_time)
        if self.last_tracking_angular_z > 0.0:
            initial_direction = -1.0
        elif self.last_tracking_angular_z < 0.0:
            initial_direction = 1.0
        else:
            initial_direction = 1.0

        if elapsed < 2.0:
            direction = initial_direction
        else:
            search_phase = int((elapsed - 2.0) / 60.0)
            direction = -initial_direction if search_phase % 2 == 0 else initial_direction

        angular_z = direction * min(self.search_angular_speed_rps, 0.12)
        cmd = Twist()
        cmd.angular.z = angular_z
        self._publish_cmd(cmd)
        self._throttled_info(f'SEARCH_MARKER: direction={direction:+.0f}, angular_z={angular_z:+.3f}')

    def _handle_approach_pre_dock(self):
        obs = self._get_tracking_observation('marker lost during pre-dock approach')
        if obs is None:
            return
        distance_error = obs.z_m - self.pre_dock_distance_m
        if abs(distance_error) <= self.pre_dock_tolerance_m:
            self._transition_parking(ParkingState.ALIGN_AXIS, 'pre-dock distance reached')
            self.cmd_pub.publish(Twist())
            return
        linear_x = self.k_z * distance_error
        if linear_x > 0.0:
            linear_x = max(self.min_approach_speed_mps, linear_x)
        linear_x = self._clamp(linear_x, -self.max_reverse_speed_mps, self.max_approach_speed_mps)
        angular_z = self.compute_approach_angular(obs)
        self.publish_cmd(linear_x, angular_z)
        self._throttled_info(
            f'APPROACH_PRE_DOCK: z={obs.z_m:.3f}, x={obs.x_m:.3f}, '
            f'bearing={obs.bearing_rad:.3f}, cmd=({linear_x:.3f}, {angular_z:.3f})')

    def _handle_parking_align(self):
        obs = self._get_tracking_observation('marker lost during align')
        if obs is None:
            return
        aligned = (abs(obs.x_m) <= self.final_lateral_limit_m and abs(obs.bearing_rad) <= self.final_bearing_limit_rad)
        if aligned:
            self._transition_parking(ParkingState.FINAL_APPROACH, 'axis aligned')
            self._publish_cmd(Twist())
            return
        angular_z = self.compute_approach_angular(obs)
        angular_z = self._clamp(angular_z, -0.08, 0.08)
        linear_x = 0.012 if abs(obs.bearing_rad) <= 0.12 else 0.0
        self.publish_cmd(linear_x, angular_z)
        self.last_tracking_angular_z = angular_z

    def _handle_parking_final(self):
        obs = self._get_tracking_observation('marker lost during final approach')
        if obs is None:
            return
        if abs(obs.x_m) > self.final_lateral_limit_m * 2:
            self._transition_parking(ParkingState.ALIGN_AXIS, 'final lateral error too large')
            self.cmd_pub.publish(Twist())
            return
        if abs(obs.bearing_rad) > self.final_bearing_limit_rad * 2:
            self._transition_parking(ParkingState.ALIGN_AXIS, 'final bearing error too large')
            self.cmd_pub.publish(Twist())
            return

        if obs.z_m <= self.parking_stop_distance_m:
            self._publish_cmd(Twist())
            self._transition_parking(ParkingState.DONE, 'parking distance reached')
            self._on_parking_done()
            return

        angular_z = -self.k_final_bearing * obs.bearing_rad - self.k_final_lateral * obs.x_m
        angular_z = self._clamp(angular_z, -self.max_angular_speed_rps * 0.5, self.max_angular_speed_rps * 0.5)
        self.publish_cmd(self.final_approach_speed_mps, angular_z)
        self._throttled_info(
            f'FINAL_APPROACH: z={obs.z_m:.3f}, x={obs.x_m:.3f}, '
            f'bearing={obs.bearing_rad:.3f}, cmd=({self.final_approach_speed_mps:.3f}, {angular_z:.3f})')

    def _handle_parking_recovery(self):
        elapsed = self._elapsed(self.parking_state_enter_time)
        if elapsed < self.recovery_backup_time_sec:
            self.publish_cmd(-self.max_reverse_speed_mps, 0.0)
            self._throttled_info(
                f'RECOVERY_BACKUP: backing up, retry={self.parking_retry_count}/{self.max_retry_count}')
            return
        if self.parking_retry_count >= self.max_retry_count:
            self._transition_parking(ParkingState.FAILED, 'retry count exceeded')
            self._publish_cmd(Twist())
            return
        self._transition_parking(ParkingState.SEARCH_MARKER, 'recovery finished')

    def compute_approach_angular(self, obs: ArucoObservation) -> float:
        angular_z = -self.k_bearing * obs.bearing_rad
        return self._clamp(angular_z, -self.max_angular_speed_rps, self.max_angular_speed_rps)

    def _start_parking_recovery(self, reason: str):
        self.parking_retry_count += 1
        self._transition_parking(ParkingState.RECOVERY, reason)

    def _transition_parking(self, new_state: ParkingState, reason: str = ''):
        if self.parking_state == new_state:
            return
        old = self.parking_state
        self.parking_state = new_state
        self.parking_state_enter_time = self.get_clock().now()
        self.get_logger().info(f'PARKING STATE: {old.value} -> {new_state.value}. reason={reason}')

    def get_valid_observation(self) -> Optional[ArucoObservation]:
        with self.data_lock:
            obs = self.latest_observation
            marker_time = self.last_marker_time
        if obs is None:
            return None
        if self._elapsed(marker_time) > self.marker_lost_timeout_sec:
            return None
        return obs

    def _reset_parking_state(self):
        self.parking_state = ParkingState.SEARCH_MARKER
        self.parking_state_enter_time = self.get_clock().now()
        self.parking_start_time = self.get_clock().now()
        self.parking_retry_count = 0
        with self.data_lock:
            self.latest_observation = None


    def _publish_cmd(self, cmd: Twist) -> None:
        if not self.enable_motion:
            # ! 정지 명령 발행하면 teleop 명령하고 싶을 때 충돌할 수 있음
            # self.cmd_pub.publish(Twist())
            return
        self.cmd_pub.publish(cmd)

    def publish_cmd(self, linear_x: float, angular_z: float) -> None:
        cmd = Twist()
        cmd.linear.x = self._clamp(linear_x, -self.max_reverse_speed_mps, self.max_approach_speed_mps)
        cmd.angular.z = self._clamp(angular_z, -self.max_angular_speed_rps, self.max_angular_speed_rps)
        if (self.parking_state in [ParkingState.APPROACH_PRE_DOCK, ParkingState.ALIGN_AXIS, ParkingState.FINAL_APPROACH]
                and abs(cmd.angular.z) > 0.01):
            self.last_tracking_angular_z = cmd.angular.z
        self._publish_cmd(cmd)

    def _on_parking_done(self):
        self._report_stage('PARKING', StageResult.PASS, 'ArUco 정렬 완료 (좌우/거리 오차 이내)')
        self.notify_tts('직각주차가 완료되었습니다.')

        if self.parking_timer is not None:
            self.parking_timer.cancel()
            self.parking_timer = None

        if self.run_phase == RunPhase.RETRY:
            self._finish_retry()
            return

        self.wp_index = 9
        self.mode = DrivingMode.NAV_TO_END
        self.set_led('END')
        self._publish_status('NAV_TO_END', 'IN_PROGRESS', '주차 완료, 도착점으로 이동 중')   # [병합: A]
        self.send_waypoint(self.waypoints[self.wp_index])

    def _draw_parking_debug_image(self, frame, corners, ids, observation, selected_index):
        debug = frame.copy()
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)
        h, w = debug.shape[:2]
        cv2.line(debug, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
        cv2.putText(debug, f'state={self.parking_state.value}', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 0), 2, cv2.LINE_AA)

        if observation is not None:
            cx, cy = int(observation.center_x), int(observation.center_y)
            cv2.circle(debug, (cx, cy), 5, (0, 0, 255), -1)
            info_1 = f'id={observation.marker_id} z={observation.z_m:.2f}m x={observation.x_m:.2f}m'
            info_2 = f'bearing={observation.bearing_rad:.2f} retry={self.parking_retry_count}/{self.max_retry_count}'
            cv2.putText(debug, info_1, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(debug, info_2, (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 0), 2, cv2.LINE_AA)
            if selected_index is not None and self.camera_matrix is not None and self.dist_coeffs is not None:
                try:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[selected_index]], self.marker_size_m, self.camera_matrix, self.dist_coeffs)
                    cv2.drawFrameAxes(debug, self.camera_matrix, self.dist_coeffs,
                                       rvecs[0], tvecs[0], self.marker_size_m * 0.5)
                except Exception:
                    pass
        else:
            cv2.putText(debug, 'target marker not found', (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 0, 255), 2, cv2.LINE_AA)
        return debug

    # TODO 디버그용 인수에 debug_candidates 추가
    # def _draw_s_course_debug_image(self, cv_image, mask, roi_top, cx_full, offset, best_contour=None, band_rect=None):
    def _draw_s_course_debug_image(self, cv_image, mask, roi_top, cx_full, offset, best_contour=None, band_rect=None, debug_candidates=None):
        h, w = cv_image.shape[:2]
        debug = cv_image.copy()

        # TODO 디버그용 모든 후보를 박스+텍스트로 표시
        if debug_candidates:
            for (x, y, cw, ch, area, solidity, rejected_reason) in debug_candidates:
                # 원본 이미지 좌표로 변환
                top_left = (x, y + roi_top)
                bottom_right = (x + cw, y + ch + roi_top)
                color = (0, 0, 255) if rejected_reason else (0, 255, 0)   # 탈락=빨강, 통과=초록
                cv2.rectangle(debug, top_left, bottom_right, color, 1)
                label = f'a={area:.0f} s={solidity:.2f}'
                if rejected_reason:
                    label += f' [{rejected_reason}]'
                cv2.putText(debug, label, (top_left[0], top_left[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

        # ROI 경계선 표시(민트 가로선)
        cv2.rectangle(debug, (0, roi_top), (w, h), (255, 255, 0), 2)
        # ?                     시작꼭짓점   끝꼭짓점    민트색     선 굵기

        # 마스크를 컬러로 변환해서 ROI 위치에 반투명하게 덧씌우기
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_color[mask > 0] = (0, 0, 255)   # 검출된 라인 픽셀 빨간색
        roi_region = debug[roi_top:h, :]
        debug[roi_top:h, :] = cv2.addWeighted(roi_region, 0.6, mask_color, 0.4, 0)


        # 실제로 채택된 컨투어만 초록 테두리로 강조
        if best_contour is not None:
            shifted = best_contour + [0, roi_top]   # ROI 좌표 -> 원본 이미지 좌표로 이동
            cv2.drawContours(debug, [shifted], -1, (0, 255, 0), 2)

        # 화면 중심선(흰색 세로선)
        cv2.line(debug, (w // 2, roi_top), (w // 2, h), (255, 255, 255), 2)
        # ?            세로중앙&하단40%영역 ~ 세로중앙&끝까지   

        # + cx 계산에 쓰인 하단 밴드 영역 표시
        if band_rect is not None:
            bx, by, bcw, bch = band_rect
            # band_rect는 ROI 좌표계이므로 원본 이미지 좌표로 변환(roi_top만큼 더해줌)
            cv2.rectangle(debug, (bx, by + roi_top), (bx + bcw, by + bch + roi_top), (255, 0, 255), 2)
            cv2.line(debug, (w // 2, roi_top), (w // 2, h), (255, 255, 255), 1)

        # 검출된 라인 중심점(초록 가로선)
        if cx_full is not None:
            # cy = roi_top + (h - roi_top) // 2
            if band_rect is not None:
                _, by, _, bch = band_rect
                cy = roi_top + by + bch // 2
            else:
                cy = roi_top + (h - roi_top) // 2
                #? 관심영역에서시작+(ROI 세로길이 의 중점) => ROI의 중앙 Y좌표
            cv2.circle(debug, (int(cx_full), cy), 6, (0, 255, 0), -1)
            # ?                  라인중심(ofset) 반지름             속찬원
            cv2.line(debug, (w // 2, cy), (int(cx_full), cy), (0, 255, 0), 2)

        # 상태 텍스트
        offset_text = f'offset={offset:.3f}' if offset is not None else 'LINE NOT FOUND'
        cv2.putText(debug, offset_text, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(debug, f's_course_state={self.s_course_state.name}', (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)

        try:
            debug_msg = self.bridge.cv2_to_compressed_imgmsg(debug, dst_format='jpg')
            self.s_course_debug_pub.publish(debug_msg)
        except Exception as exc:
            self._throttled_warn(f'[TRACING_S] debug image publish failed: {exc}')

    def _elapsed(self, start_time) -> float:
        return (self.get_clock().now() - start_time).nanoseconds * 1e-9

    def _throttled_info(self, msg: str) -> None:
        if self._elapsed(self.last_log_time) >= self.log_throttle_sec:
            self.get_logger().info(msg)
            self.last_log_time = self.get_clock().now()

    def _throttled_warn(self, msg: str) -> None:
        if self._elapsed(self.last_log_time) >= self.log_throttle_sec:
            self.get_logger().warn(msg)
            self.last_log_time = self.get_clock().now()

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    # ================= 속도 제어 =================
    def set_speed(self, speed):
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('controller_server 파라미터 서비스 응답 없음')
            return
        param = Parameter()
        param.name = 'FollowPath.max_vel_x'
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=speed)
        req = SetParameters.Request()
        req.parameters = [param]
        future = self.param_client.call_async(req)
        future.add_done_callback(self.on_accel_response)

    def on_accel_response(self, future):
        try:
            result = future.result()
            ok = result.results[0].successful
            self.get_logger().info(f'[set_speed] 속도 변경 {"성공" if ok else "실패"}')
        except Exception as e:
            self.get_logger().warn(f'[set_speed] 응답 처리 실패: {e}')

    # ================= [병합: A] costmap inflation_radius 제어 =================
    def set_inflation_radius(self, radius: float, clients=None):
        if clients is None:
            clients = [self.local_costmap_param_client, self.global_costmap_param_client]

        param = Parameter()
        param.name = 'inflation_layer.inflation_radius'
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=radius)
        req = SetParameters.Request()
        req.parameters = [param]

        for client in clients:
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'{client.srv_name} 서비스 응답 없음')
                continue
            future = client.call_async(req)
            future.add_done_callback(
                lambda f, name=client.srv_name: self._on_inflation_response(f, name))

    def _on_inflation_response(self, future, client_name: str):
        try:
            result = future.result()
            ok = result.results[0].successful
            self.get_logger().info(f'[inflation_radius] {client_name} 변경 {"성공" if ok else "실패"}')
        except Exception as e:
            self.get_logger().warn(f'[inflation_radius] {client_name} 응답 처리 실패: {e}')

    def set_inflation_radius_pair(self, local_radius: float, global_radius: float):
        self.set_inflation_radius(local_radius, clients=[self.local_costmap_param_client])
        self.set_inflation_radius(global_radius, clients=[self.global_costmap_param_client])

    # ================= 크랭크 코스 제어 루프 =================
    def crank_control_loop(self):
        if self.is_estopped:
            self._publish_cmd(Twist())
            return
        with self.crank_lock:
            if self.mode != DrivingMode.TRACING_CRANK:
                return
            if self.crank_state in (LineCourseState.DONE, LineCourseState.FAILED):
                self._publish_cmd(Twist())
                return

            if self.stage_results['TRACING_CRANK'] == StageResult.IN_PROGRESS:
                self._throttled_status_republish('TRACING_CRANK', StageResult.IN_PROGRESS)


            if self.crank_state == LineCourseState.LINE_FOLLOWING:
                self._handle_crank_following()
                self._check_crank_arrival()
            elif self.crank_state == LineCourseState.CREEPING:
                self._handle_crank_creeping()
            elif self.crank_state == LineCourseState.TURNING:
                self._handle_crank_turning()
    def _throttled_status_republish(self, stage: str, result: StageResult):
        now = self.get_clock().now()
        last = getattr(self, '_last_status_republish_time', None)
        if last is None or (now - last).nanoseconds / 1e9 >= 2.0:
            self._publish_status(stage, result.name, '')
            self._last_status_republish_time = now


    def _reset_crank_state(self):
        with self.crank_lock:
            self.crank_state = LineCourseState.LINE_FOLLOWING
            self.crank_state_enter_time = self.get_clock().now()
            self.crank_line_lost_since = None
            self.last_meaningful_ir = (0, 1, 0)
            self.crank_turn_pending_delta = 0.0
            self.crank_creep_start_time = None

    def _handle_crank_following(self):
        ir = (self.ir_l, self.ir_c, self.ir_r)
        # self.get_logger().info(f'[CRANK_COURSE] IR={ir}')

        if ir == (1, 1, 0):
            self.crank_line_lost_since = None
            self._start_crank_creep_forward(90.0)
            return
        if ir == (0, 1, 1):
            self.crank_line_lost_since = None
            self._start_crank_creep_forward(-90.0)
            return
        if ir == (0, 1, 0):
            self.crank_line_lost_since = None
            self.last_meaningful_ir = ir
            self.publish_cmd(self.CRANK_LINEAR_SPEED, 0.0)
            # self.get_logger().info(f'[TRACING_CRANK] IR={ir}, 직진 유지, cmd=({self.CRANK_LINEAR_SPEED:.3f}, 0.0)')
            return
        if ir == (1, 0, 0):
            self.crank_line_lost_since = None
            self.last_meaningful_ir = ir
            self.publish_cmd(self.CRANK_RECOVERY_SPEED, self.CRANK_STEER_ANGULAR)
            # self.get_logger().info(f'[CRANK_COURSE] IR={ir}, 왼쪽으로 회전, cmd=({self.CRANK_LINEAR_SPEED:.3f}, {self.CRANK_STEER_ANGULAR:.3f})')
            return
        if ir == (0, 0, 1):
            self.crank_line_lost_since = None
            self.last_meaningful_ir = ir
            self.publish_cmd(self.CRANK_RECOVERY_SPEED, -self.CRANK_STEER_ANGULAR)
            # self.get_logger().info(f'[CRANK_COURSE] IR={ir}, 오른쪽으로 회전, cmd=({self.CRANK_LINEAR_SPEED:.3f}, {-self.CRANK_STEER_ANGULAR:.3f})')
            return
        if ir == (0, 0, 0):
            self._handle_crank_line_lost()
            return

        self._throttled_warn(f'[TRACING_CRANK] 예상치 못한 IR 조합:{ir}')

    def _handle_crank_line_lost(self):
        now = self.get_clock().now()
        if self.crank_line_lost_since is None:
            self.crank_line_lost_since = now

        elapsed = (now - self.crank_line_lost_since).nanoseconds / 1e9

        if elapsed < self.CRANK_LINE_GRACE_SEC:
            return

        if elapsed < self.CRANK_LINE_LOST_TIMEOUT_SEC:
            if self.last_meaningful_ir == (1, 0, 0):
                # self.publish_cmd(self.CRANK_RECOVERY_SPEED, self.CRANK_STEER_ANGULAR)
                # ! 가속 시도 : 100,001일 때의 속도도 동일하게 + 각속도도 더 올림
                self.publish_cmd(self.CRANK_RECOVERY_SPEED, self.CRANK_STEER_ANGULAR)
            elif self.last_meaningful_ir == (0, 0, 1):
                # self.publish_cmd(self.CRANK_RECOVERY_SPEED, -self.CRANK_STEER_ANGULAR)
                # ! 가속 시도 : 100,001일 때의 속도도 동일하게 + 각속도도 더 올림
                self.publish_cmd(self.CRANK_RECOVERY_SPEED, -self.CRANK_STEER_ANGULAR)
            else:
                self.publish_cmd(self.CRANK_RECOVERY_SPEED, 0.0)
            return

        self._on_crank_failed('IR 라인 이탈 지속, 복구 시간 초과')

    def _start_crank_creep_forward(self, target_delta_deg: float):
        self.crank_turn_pending_delta = target_delta_deg
        self.crank_creep_start_time = self.get_clock().now()
        self.crank_creep_target_sec = self.CRANK_CREEP_DISTANCE_M / self.CRANK_LINEAR_SPEED
        self.crank_state = LineCourseState.CREEPING
        self.publish_cmd(self.CRANK_LINEAR_SPEED, 0.0)
        self.get_logger().info(f'[TRACING_CRANK] CREEPING 시작, target_sec={self.crank_creep_target_sec:.3f}')

    def _handle_crank_creeping(self):
        elapsed = self._elapsed(self.crank_creep_start_time)
        if elapsed >= self.crank_creep_target_sec:
            self._start_crank_turn(self.crank_turn_pending_delta)

    def _start_crank_turn(self, target_delta_deg: float):
        self.crank_turn_start_yaw = self.current_yaw
        self.crank_turn_target_delta = math.radians(target_delta_deg)
        self.crank_state = LineCourseState.TURNING
        self.crank_state_enter_time = self.get_clock().now()
        self._publish_cmd(Twist())
        self.get_logger().info(f'[TRACING_CRANK] TURNING 시작, target={target_delta_deg}도')

    def _handle_crank_turning(self):
        elapsed = self._elapsed(self.crank_state_enter_time)
        if elapsed > self.CRANK_TURN_TIMEOUT_SEC:
            self._on_crank_failed('90도 회전 시간 초과')
            return

        yaw_diff = self._normalize_angle(self.current_yaw - self.crank_turn_start_yaw)
        remaining = self._normalize_angle(self.crank_turn_target_delta - yaw_diff)

        if abs(remaining) <= self.CRANK_TURN_TOLERANCE_RAD:
            self._publish_cmd(Twist())
            self.crank_state = LineCourseState.LINE_FOLLOWING
            self.crank_line_lost_since = None
            self.get_logger().info('[TRACING_CRANK] TURNING 완료, LINE_FOLLOWING 복귀')
            return

        direction = 1.0 if self.crank_turn_target_delta > 0 else -1.0
        self.publish_cmd(0.0, direction * self.CRANK_TURN_ANGULAR_SPEED)

    def _check_crank_arrival(self):
        with self.data_lock:
            x, y = self.current_x, self.current_y

        if x is None:
            self.get_logger().warn('[TRACING_CRANK] 현재 위치를 알 수 없습니다.')
            return
        target = self.waypoints[1]
        dist = math.hypot(x - float(target['x']), y - float(target['y']))
        # self.get_logger().info(f'[CRANK] 현재=({x:.3f}, {y:.3f}), 목표=({target["x"]}, {target["y"]}), dist={dist:.3f}m')  # ← 추가
        if dist <= self.CRANK_ARRIVAL_TOLERANCE_M:
            self._on_crank_done()

    def _on_crank_done(self):
        self.crank_state = LineCourseState.DONE
        self._report_stage('TRACING_CRANK', StageResult.PASS, '크랭크 코스 라인트레이싱 완료')
        self.notify_tts('크랭크 코스를 완료했습니다.')
        self._publish_cmd(Twist())

        if self.crank_timer is not None:
            self.crank_timer.cancel()
            self.crank_timer = None

        if self.run_phase == RunPhase.RETRY:
            self._finish_retry()
            return

        self.wp_index = 2
        self.mode = DrivingMode.NAV_TO_S
        self.send_waypoint(self.waypoints[2])
        self.vision_enable = True

    def _on_crank_failed(self, reason: str):
        self.crank_state = LineCourseState.FAILED
        self._report_stage('TRACING_CRANK', StageResult.FAIL, reason)
        self.get_logger().warn(f'[TRACING_CRANK] FAILED:{reason}')
        self.notify_tts('크랭크 코스에 실패했습니다. 다음 구간으로 이동합니다.')
        self._publish_cmd(Twist())

        if self.crank_timer is not None:
            self.crank_timer.cancel()
            self.crank_timer = None

        if self.run_phase == RunPhase.RETRY:
            self._finish_retry()
            return

        self.wp_index = 2
        self.mode = DrivingMode.NAV_TO_S
        self.send_waypoint(self.waypoints[2])
        self.vision_enable = True

    def s_course_control_loop(self):
        if self.is_estopped:
            self._publish_cmd(Twist())
            return
        with self.s_course_lock:
            if self.mode != DrivingMode.TRACING_S:
                return
            if self.s_course_state in (SCourseState.DONE, SCourseState.FAILED):
                self._publish_cmd(Twist())
                return

            offset = self.s_line_offset
            last_seen = self.s_line_last_seen_time

        self._check_s_course_arrival()
        if self.s_course_state == SCourseState.DONE:
            return

        if offset is None:
            if self._elapsed(last_seen) > self.S_LINE_LOST_TIMEOUT_SEC:
                self._on_s_course_failed('카메라에서 라인 미검출 지속')
                return
            self.publish_cmd(self.S_LINEAR_SPEED_MIN, 0.0)
            return

        if abs(offset) < self.S_OFFSET_DEADBAND:
            self.publish_cmd(self.S_LINEAR_SPEED_MAX, 0.0)
            return

        angular_z = self._clamp(-self.S_ANGULAR_GAIN * offset,
                                -self.S_ANGULAR_MAX, self.S_ANGULAR_MAX)
        linear_x = max(self.S_LINEAR_SPEED_MIN,
                        self.S_LINEAR_SPEED_MAX * (1.0 - min(abs(offset), 1.0)))  # ? 라인대로 가는 경우 최대속도, 라인과 벗어나면 최저속도(0은 안되게)
        self.publish_cmd(linear_x, angular_z)

    def _reset_s_course_state(self):
        with self.s_course_lock:
            self.s_course_state = SCourseState.TRACKING
            self.s_line_offset = None
            self.s_line_last_seen_time = self.get_clock().now()
            self.s_last_valid_offset = None   # 추가: 코스 새로 시작할 때 초기화

    def _check_s_course_arrival(self):
        with self.data_lock:
            x, y = self.current_x, self.current_y
        if x is None:
            self.get_logger().warn('[TRACING_CRANK] 현재 위치를 알 수 없습니다.')
            return
        target = self.waypoints[3]
        dist = math.hypot(x - float(target['x']), y - float(target['y'])) 
        self.get_logger().info(f'[S_COURSE] 현재=({x:.3f}, {y:.3f}), 목표=({target["x"]}, {target["y"]}), dist={dist:.3f}m')
        if dist <= self.S_ARRIVAL_TOLERANCE_M:
            self._on_s_course_done()

    def _on_s_course_done(self):
        self.s_course_state = SCourseState.DONE
        self._report_stage('TRACING_S', StageResult.PASS, 'S자 코스 라인트레이싱 완료')
        self.notify_tts('S자 코스를 완료했습니다.')
        self._publish_cmd(Twist())

        if self.s_course_timer is not None:   # 추가
            self.s_course_timer.cancel()
            self.s_course_timer = None

        if self.run_phase == RunPhase.RETRY:
            self._finish_retry()
            return

        self.wp_index = 4
        self.mode = DrivingMode.NAV_TO_SIGNAL
        self.send_waypoint(self.waypoints[4])

    def _on_s_course_failed(self, reason: str):
        self.s_course_state = SCourseState.FAILED
        self._report_stage('TRACING_S', StageResult.FAIL, reason)
        self.get_logger().warn(f'[S_COURSE] FAILED: {reason}')
        self.notify_tts('S자 코스에 실패했습니다. 다음 구간으로 이동합니다.')
        self._publish_cmd(Twist())

        if self.s_course_timer is not None:   # 추가
            self.s_course_timer.cancel()
            self.s_course_timer = None

        if self.run_phase == RunPhase.RETRY:
            self._finish_retry()
            return

        self.wp_index = 4
        self.mode = DrivingMode.NAV_TO_SIGNAL
        self.send_waypoint(self.waypoints[4])

    # ================= LED 제어 =================
    def set_led(self, state_key):
        color = LED_COLOR_MAP.get(state_key)
        if color is None:
            self.get_logger().warn(f'[LED] 알 수 없는 상태키: {state_key}')
            return
        msg = ColorRGBA()
        msg.r, msg.g, msg.b = color
        msg.a = 1.0
        self.pub_led.publish(msg)
        self.get_logger().info(f'[LED] {color}점등 ({state_key})')

    # ================= TTS 제어 =================
    def notify_tts(self, text: str):
        msg = AudioCommand()
        msg.type = AudioCommand.TYPE_TTS
        msg.text = text
        msg.volume = 1.0
        msg.repeat = 1
        self.audio_pub.publish(msg)
        self.get_logger().info(f'[TTS] {text}')

    # ================= 결과 요약 =================
    def publish_final_result(self):
        for stage, result in self.stage_results.items():
            self.get_logger().info(f'{stage}: {result.name}')
            msg = DrivingStatus()
            msg.mode = stage
            msg.result = result.name
            msg.reason = ''
            msg.wp_index = self.wp_index
            self.status_pub.publish(msg)

    def _announce_final_result(self):
        self.publish_final_result()
        overall_pass = all(r == StageResult.PASS for r in self.stage_results.values())

        self.mode = DrivingMode.RESULT_SUMMARY   # [병합: A] 장애물 콜백이 완료 후 상태를 구분할 수 있도록
        self._publish_status('COMPLETE', 'PASS' if overall_pass else 'FAIL', '전체 코스 완료')   # [병합: A]

        if overall_pass:
            self.notify_tts('전체 코스를 완료했습니다. 모든 구간을 성공적으로 통과했습니다.')
        else:
            fail_stages = [name for name, r in self.stage_results.items() if r == StageResult.FAIL]
            self.notify_tts(f'전체 코스를 완료했습니다. {", ".join(fail_stages)} 구간에서 실패했습니다.')

    def destroy_node(self):
        for _ in range(5):
            self.cmd_pub.publish(Twist())
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DrivingNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()