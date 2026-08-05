import math
import os
import yaml
import rclpy
from enum import Enum
from dataclasses import dataclass
from typing import Optional

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import String
# 여기추가 밑 한줄
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from yeyu_msgs.msg import DrivingStatus, AudioCommand
from yeyu_msgs.srv import StartRetry
from yeyu_control.states.driving_mode import DrivingMode
from yeyu_control.states.parking_state import ParkingState
from yeyu_control.states.stage_result import StageResult
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


# ================= wp 도착 시 자동 모드 전환 테이블 =================
NAV_ARRIVAL_TRANSITIONS = {
    DrivingMode.NAV_TO_SIGNAL: DrivingMode.SIGNAL_WAIT,
    DrivingMode.NAV_TO_ACCEL: DrivingMode.ACCEL_ZONE,
    DrivingMode.NAV_TO_PARKING: DrivingMode.PARKING,
}

# 여기 수정
LED_COLOR_MAP = {
    'START':       (1.0, 0.0, 0.0),          # RED (255,0,0)
    'SIGNAL_WAIT': (11/255, 1.0, 11/255),    # GREEN (11,255,11)
    'ACCEL_ZONE':  (0.0, 1.0, 1.0),          # BLUE (0,255,255)
    'PARKING':     (1.0, 70/255, 0.0),       # YELLOW (255,70,0)
    'END':         (1.0, 0.0, 0.0),          # RED
}


class RunPhase(Enum):
    MAIN = 'MAIN'     # Trajectory 1(정규 코스) 진행 중
    RETRY = 'RETRY'   # 특정 구간 재시험 진행 중


# 재시험 대상별 진입점(wp 인덱스)과, 그 wp로 이동할 때의 mode
RETRY_ENTRY = {
    'SIGNAL_WAIT': {'wp_index': 1, 'mode': DrivingMode.NAV_TO_SIGNAL},   # wp2
    'ACCEL_ZONE':  {'wp_index': 2, 'mode': DrivingMode.NAV_TO_ACCEL},    # wp3
    'PARKING':     {'wp_index': 4, 'mode': DrivingMode.NAV_TO_PARKING}, # wp5
}
HOME_WP_INDEX = 6   # wp7, 재시험 종료 후 항상 여기로 복귀


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

        self.wp_index = 0
        self.green_count = 0
        self.blue_count = 0
        self.signal_wait_enter_time = None
        self.accel_zone_enter_time = None
        self.ACCEL_GRACE_PERIOD_SEC = 2.0   # 표지판 인식 후 가속 유예 시간 (실측 후 조정)
        self.accel_zone_max_speed = 0.0   # 가속구간 진입 후 관측된 최고 속도
        self.ACCEL_TARGET_SPEED = 0.18    # 이 속도를 한 번이라도 넘기면 PASS

        self.ACCEL_SUSTAIN_SEC = 0.5           # 목표 속도 이상을 이만큼 연속 유지하면 PASS
        self.ACCEL_DIP_TOLERANCE_SEC = 0.15   # 이 시간 이내의 짧은 하락은 봐줌
        self.accel_last_below_time = None     # 마지막으로 기준 밑으로 떨어진 시각
        self.accel_sustain_start = None        # 목표 속도 이상이 시작된 시각 (끊기면 None으로 리셋)

        # --- 콜백 그룹 ---
        self.camera_cb_group = ReentrantCallbackGroup()
        self.nav_cb_group = ReentrantCallbackGroup()
        self.parking_cb_group = ReentrantCallbackGroup()

        # --- 구간별 성공/실패 결과 저장소 ---
        self.stage_results = {
            'NAV_WAYPOINT': StageResult.IN_PROGRESS,
            'SIGNAL_WAIT': StageResult.IN_PROGRESS,
            'ACCEL_ZONE': StageResult.IN_PROGRESS,
            'PARKING': StageResult.IN_PROGRESS,
        }

        # --- 재시험 상태 ---
        self.run_phase = RunPhase.MAIN
        self.retry_target = None

        # --- 비상정지 상태 ---
        self.is_estopped = False

        # --- waypoints 로드 (wp1~wp7) ---
        wp_path = os.path.join(
            get_package_share_directory('yeyu_waypoint_nav'),
            'waypoints',
            'waypoint2.yaml'
        )
        with open(wp_path) as f:
            self.waypoints = yaml.safe_load(f)['waypoints']

        # --- Nav2 액션 클라이언트 ---
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.nav_cb_group)
        self.current_goal_handle = None
        self.nav_fail_count = 0

        # --- 재시험 서비스 ---
        self.retry_srv = self.create_service(
            StartRetry, '/start_retry', self.on_start_retry_request,
            callback_group=self.nav_cb_group)

        # --- 비상정지 서비스 ---
        self.estop_srv = self.create_service(
            Trigger, '/emergency_stop', self.on_emergency_stop_request,
            callback_group=self.nav_cb_group)

        # --- ArUco/주차 파라미터 선언 및 로드 ---
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

        # --- 구독/발행 ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            CompressedImage, self.image_topic, self.on_camera, sensor_qos,
            callback_group=self.camera_cb_group)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, sensor_qos,
            callback_group=self.camera_cb_group)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10,
            callback_group=self.camera_cb_group)

        self.param_client = self.create_client(SetParameters, '/controller_server/set_parameters')
        # 여기 추가
        #self.pub_led = self.create_publisher(String, '/led_command', 10)
        self.pub_led = self.create_publisher(ColorRGBA, 'sensor_bridge/rgb_cmd', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(DrivingStatus, '/driving_status', 10)
        self.image_pub = self.create_publisher(CompressedImage, '/camera/image_flipped/compressed', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '/parking_debug_image/compressed', 10)
        self.audio_pub = self.create_publisher(AudioCommand, '/audio/command', 10)

        timer_period = 1.0 / max(self.control_rate_hz, 0.5)
        self.control_timer = self.create_timer(
            timer_period, self.parking_control_loop,
            callback_group=self.parking_cb_group)

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
        self.startup_timer = self.create_timer(0.5, self.on_startup)

    # ================= 파라미터 =================
    def _declare_parking_parameters(self):
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('target_marker_id', 0)
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

    # ================= 시작 시퀀스 (LED 구독자 대기) =================
    def on_startup(self):
        self.startup_timer.cancel()
        self._start_check_timer = self.create_timer(0.3, self._try_start)

    def _try_start(self):
        if self.pub_led.get_subscription_count() == 0 & self.audio_pub.get_subscription_count()==0:
            # self.get_logger().warn('[LED] 구독자, [audio]구독자 대기 중...')
            return
        self._start_check_timer.cancel()
        self.set_led('START')
        self.notify_tts('주행을 시작합니다.')
        self._report_stage('NAV_WAYPOINT', StageResult.IN_PROGRESS, '')
        self.send_waypoint(self.waypoints[0])

    # ================= 구간 결과 보고 (공통 헬퍼) =================
    def _publish_status(self, mode: str, result: str, reason: str = ''):
        """/driving_status를 그대로 발행 (stage_results는 건드리지 않음).
        NAV_TO_END처럼 채점 대상이 아닌 '이동 중' 표시에도 재사용."""
        msg = DrivingStatus()
        msg.mode = mode
        msg.result = result
        msg.reason = reason
        msg.wp_index = self.wp_index
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
        self._report_stage(target, StageResult.IN_PROGRESS, '재시험 시작')   # 결과 리셋 + GUI에 즉시 반영
        self.green_count = 0
        self.blue_count = 0
        self.speed_violation_start = None

        self.wp_index = entry['wp_index']
        self.mode = entry['mode']

        if target == 'PARKING':
            self._reset_parking_state()

        self.get_logger().info(f'[RETRY] {target} 재시험 시작 → wp{entry["wp_index"] + 1}로 이동')
        self.notify_tts(f'{target} 구간을 재시험합니다.')
        self.send_waypoint(self.waypoints[self.wp_index])

        response.accepted = True
        response.message = 'OK'
        return response

    def _finish_retry(self):
        """재시험 판정이 끝난 뒤 공통으로 호출: 결과와 무관하게 항상 홈(wp7)으로 복귀"""
        self.get_logger().info(f'[RETRY] {self.retry_target} 재시험 판정 완료, 홈으로 복귀')
        self.notify_tts(f'{self.retry_target} 재시험을 완료했습니다. 도착점으로 이동합니다.')
        # run_phase, retry_target은 여기서 바꾸지 않음 — wp7 도착 후 결과 발표까지 유지
        self.wp_index = HOME_WP_INDEX
        self.mode = DrivingMode.NAV_TO_END
        self._publish_status('NAV_TO_END', 'IN_PROGRESS', '재시험 종료, 도착점으로 복귀 중')
        self.send_waypoint(self.waypoints[HOME_WP_INDEX])

    def _announce_retry_result(self, target: str):
        """재시험 대상 구간 하나의 결과만 로그+토픽+TTS로 알림"""
        result = self.stage_results[target]
        msg = DrivingStatus()
        msg.mode = target
        msg.result = result.name
        msg.reason = ''
        msg.wp_index = self.wp_index
        self.status_pub.publish(msg)

        self.get_logger().info(f'[RETRY RESULT] {target}: {result.name}')

        if result == StageResult.PASS:
            self.notify_tts(f'{target} 재시험 결과, 통과했습니다.')
        else:
            self.notify_tts(f'{target} 재시험 결과, 실패했습니다.')

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
        self.get_logger().info(f'[on_nav_result] status={status}')

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.nav_fail_count = 0

            if self.run_phase == RunPhase.MAIN and self.wp_index == 0:   # wp1 도착 → wp2로
                self.wp_index = 1
                self.mode = DrivingMode.NAV_TO_SIGNAL
                self.send_waypoint(self.waypoints[1])
                return

            if self.wp_index == 3:   # wp4 도착
                if self.run_phase == RunPhase.RETRY:
                    if self.stage_results['ACCEL_ZONE'] == StageResult.IN_PROGRESS:
                        reason = f'{self.ACCEL_SUSTAIN_SEC}초 연속 유지 실패 (최고 {self.accel_zone_max_speed:.3f} m/s)'
                        self._report_stage('ACCEL_ZONE', StageResult.FAIL, reason)
                        self.notify_tts('가속구간에서 충분히 가속하지 못했습니다.')
                    self._finish_retry()
                    return

                self.wp_index = 4
                self.mode = DrivingMode.NAV_TO_PARKING
                self.set_speed(0.13)
                if self.stage_results['ACCEL_ZONE'] == StageResult.IN_PROGRESS:
                    reason = f'{self.ACCEL_SUSTAIN_SEC}초 연속 유지 실패 (최고 {self.accel_zone_max_speed:.3f} m/s)'
                    self._report_stage('ACCEL_ZONE', StageResult.FAIL, reason)
                    self.notify_tts('가속구간에서 충분히 가속하지 못했습니다.')
                self._report_stage('NAV_WAYPOINT', StageResult.IN_PROGRESS, '')
                self.send_waypoint(self.waypoints[4])
                return

            if self.wp_index == 5:   # wp6 도착 → wp7로
                self.wp_index = 6
                self.mode = DrivingMode.NAV_TO_END
                self._publish_status('NAV_TO_END', 'IN_PROGRESS', '도착점으로 이동 중')
                self.send_waypoint(self.waypoints[6])
                return

            if self.wp_index == 6:   # wp7(최종 도착점) 도착
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

            # wp3 도착 시점: SIGNAL_WAIT 재시험 종료 처리
            next_mode = NAV_ARRIVAL_TRANSITIONS.get(self.mode)
            if next_mode is not None:
                if (self.mode == DrivingMode.NAV_TO_ACCEL
                        and self.run_phase == RunPhase.RETRY
                        and self.retry_target == 'SIGNAL_WAIT'):
                    self._finish_retry()
                    return

                self.get_logger().info(f'{self.mode.name} -> {next_mode.name} 모드 전환')
                self.mode = next_mode

                if self.mode == DrivingMode.SIGNAL_WAIT:
                    self.set_led('SIGNAL_WAIT')
                    self.signal_wait_enter_time = self.get_clock().now()
                    self._report_stage('SIGNAL_WAIT', StageResult.IN_PROGRESS, '')
                elif self.mode == DrivingMode.ACCEL_ZONE:
                    self.set_led('ACCEL_ZONE')
                    self._report_stage('ACCEL_ZONE', StageResult.IN_PROGRESS, '')
                elif self.mode == DrivingMode.PARKING:
                    self.set_led('PARKING')
                    self._reset_parking_state()
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

    # ================= 카메라: 신호/표지판/ArUco 통합 콜백 =================
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

        if self.mode == DrivingMode.SIGNAL_WAIT:
            self._process_signal(flipped)
        elif self.mode == DrivingMode.ACCEL_ZONE:
            self._process_speed_sign(flipped)
        elif self.mode == DrivingMode.PARKING:
            self._process_aruco(flipped, msg.header, image_width=flipped.shape[1], image_height=flipped.shape[0])

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
        """SIGNAL_WAIT 판정 완료 후: 항상 wp3로 이동 (본 코스든 재시험이든 동일)"""
        self.wp_index = 2
        self.mode = DrivingMode.NAV_TO_ACCEL
        self._report_stage('NAV_WAYPOINT', StageResult.IN_PROGRESS, '')
        self.send_waypoint(self.waypoints[2])

    def _process_speed_sign(self, flipped):
        color = self.detect_speed_sign(flipped)
        if color == 'blue':
            self.blue_count += 1
            if self.blue_count >= 3:
                self.blue_count = 0
                self.notify_tts('가속표지판이 감지되었습니다. 제한 속도 내로 이동합니다.')
                self.mode = DrivingMode.NAV_TO_PARKING
                self.set_speed(0.22)
                self.accel_zone_enter_time = self.get_clock().now()
                self.accel_zone_max_speed = 0.0
                self.accel_sustain_start = None      # 추가
                self.wp_index = 3
                self._report_stage('NAV_WAYPOINT', StageResult.IN_PROGRESS, '')
                self.send_waypoint(self.waypoints[3])
        else:
            self.blue_count = 0

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
                    self.latest_observation = observation
                    self.last_marker_time = self.get_clock().now()

            if self.enable_debug_image:
                debug_frame = self._draw_debug_image(frame, corners, ids, observation, selected_index)
                try:
                    debugout_msg = self.bridge.cv2_to_compressed_imgmsg(debug_frame, dst_format='jpg')
                    debugout_msg.header = header
                    self.debug_pub.publish(debugout_msg)
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

    def on_odom(self, msg):
        if self.is_estopped:
            return
        if self.mode != DrivingMode.NAV_TO_PARKING or self.accel_zone_enter_time is None:
            self.accel_sustain_start = None
            self.accel_last_below_time = None
            return

        linear_x = msg.twist.twist.linear.x
        self.get_logger().info(f'[ACCEL_ZONE] 현재 속도: {linear_x:.3f} m/s')   # ← 여기 추가


        if linear_x > self.accel_zone_max_speed:
            self.accel_zone_max_speed = linear_x

        now = self.get_clock().now()

        if linear_x >= self.ACCEL_TARGET_SPEED:
            if self.accel_sustain_start is None:
                self.accel_sustain_start = now   # 목표 속도 이상 시작된 순간 기록
            self.accel_last_below_time = None

            sustained = (now - self.accel_sustain_start).nanoseconds / 1e9
            if sustained >= self.ACCEL_SUSTAIN_SEC:
                if self.stage_results['ACCEL_ZONE'] == StageResult.IN_PROGRESS:
                    reason = (f'{sustained:.2f}초간 {self.ACCEL_TARGET_SPEED} m/s 이상 유지(허용오차 포함) '
                          f'(최고 {self.accel_zone_max_speed:.3f} m/s)')
                    self._report_stage('ACCEL_ZONE', StageResult.PASS, reason)
                    self.get_logger().info(f'[ACCEL_ZONE] {reason}')
                    self.notify_tts('가속구간을 규정 속도로 통과했습니다.')
        else:
            if self.accel_last_below_time is None:
                self.accel_last_below_time = now

            below_duration = (now -self.accel_last_below_time).nanoseconds / 1e9
            if below_duration > self.ACCEL_DIP_TOLERANCE_SEC:

                self.accel_sustain_start = None   # 목표 속도 밑으로 떨어지면 리셋, 처음부터 다시 셈

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
        self.publish_parking_state()

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

                if self.run_phase == RunPhase.RETRY:
                    self._finish_retry()
                else:
                    self.wp_index = 5
                    self.mode = DrivingMode.NAV_TO_END
                    self.set_led('END')
                    self._publish_status('NAV_TO_END', 'IN_PROGRESS', '주차 실패, 도착점으로 이동 중')
                    self.send_waypoint(self.waypoints[5])
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
        if self.latest_observation is None:
            self._publish_cmd(Twist())
            self._start_parking_recovery(lost_reason)
            return None
        age = self._elapsed(self.last_marker_time)
        if age > self.marker_lost_timeout_sec:
            self._publish_cmd(Twist())
            self._start_parking_recovery(lost_reason)
            return None
        if age > self.stale_stop_timeout_sec:
            self._publish_cmd(Twist())
            self._throttled_info(f'waiting for marker reacquisition, age={age:.2f}s')
            return None
        return self.latest_observation

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
        aligned = (abs(obs.x_m) <= self.final_lateral_limit_m
                   and abs(obs.bearing_rad) <= self.final_bearing_limit_rad)
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
        if self.latest_observation is None:
            return None
        if self._elapsed(self.last_marker_time) > self.marker_lost_timeout_sec:
            return None
        return self.latest_observation

    def _reset_parking_state(self):
        self.parking_state = ParkingState.SEARCH_MARKER
        self.parking_state_enter_time = self.get_clock().now()
        self.parking_start_time = self.get_clock().now()
        self.parking_retry_count = 0
        self.latest_observation = None

    def publish_parking_state(self) -> None:
        msg = DrivingStatus()
        msg.mode = self.mode.name
        if self.parking_state == ParkingState.DONE:
            msg.result = 'PASS'
        elif self.parking_state == ParkingState.FAILED:
            msg.result = 'FAIL'
        else:
            msg.result = 'IN_PROGRESS'
        msg.reason = self.parking_state.value
        msg.wp_index = self.wp_index
        obs = self.latest_observation
        msg.error_lateral = float(obs.x_m) if obs is not None else 0.0
        msg.error_distance = float(obs.z_m - self.parking_stop_distance_m) if obs is not None else 0.0
        msg.retry_count = self.parking_retry_count
        self.status_pub.publish(msg)

    def _publish_cmd(self, cmd: Twist) -> None:
        if not self.enable_motion:
            self.cmd_pub.publish(Twist())
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

        if self.run_phase == RunPhase.RETRY:
            self._finish_retry()
            return

        self.wp_index = 5
        self.mode = DrivingMode.NAV_TO_END
        self.set_led('END')
        self._publish_status('NAV_TO_END', 'IN_PROGRESS', '주차 완료, 도착점으로 이동 중')
        self.send_waypoint(self.waypoints[5])

    def _draw_debug_image(self, frame, corners, ids, observation, selected_index):
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

    # ================= LED 제어 =================
    def set_led(self, state_key):
        color = LED_COLOR_MAP.get(state_key)
        if color is None:
            self.get_logger().warn(f'[LED] 알 수 없는 상태키: {state_key}')
            return
        # 여기수정(주석처리가 원래버전 그 뒤5줄이 새로 생긴 줄)
        # self.pub_led.publish(String(data=color))
        # self.get_logger().info(f'[LED] {color}점등 ({state_key})')
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