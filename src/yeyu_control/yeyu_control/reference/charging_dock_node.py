#!/usr/bin/env python3

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Union

import cv2
import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, CameraInfo, Image, CompressedImage
from std_msgs.msg import Bool, String


class DockState(Enum):
    SEARCH_MARKER = 'SEARCH_MARKER'          # 마커 찾는 중
    APPROACH_PRE_DOCK = 'APPROACH_PRE_DOCK'  # 사전 정렬 위치까지 먼저 이동하는 단계
    ALIGN_DOCK_AXIS = 'ALIGN_DOCK_AXIS'      # 좌우/각도 정렬 중
    FINAL_APPROACH = 'FINAL_APPROACH'        # 목표 거리까지 직진
    # ===========================================
    CONTACT_PUSH = 'CONTACT_PUSH'            # 커넥처에 맞물리도록 살짝 밀어붙이는 단계
    CHARGE_VERIFY = 'CHARGE_VERIFY'          # 전류/전압이 흐르는지 확인
    # ================================================
    DOCKED = 'DOCKED'                        # 완료
    RECOVERY_BACKUP = 'RECOVERY_BACKUP'      # 마커 놓쳐서 후진 + 재탐색
    FAILED = 'FAILED'                        # 포기


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

class ChargingDockNode(Node):
    def __init__(self) -> None:
        super().__init__('charging_dock_node')

        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('debug_image_topic', '/charging_dock/debug_image/compressed')
        self.declare_parameter('state_topic', '/charging_dock/state')
        self.declare_parameter('docked_topic', '/charging_dock/docked')

        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('target_marker_id', 0)
        self.declare_parameter('marker_size_m', 0.10)

        self.declare_parameter('pre_dock_distance_m', 0.50)
        self.declare_parameter('pre_dock_tolerance_m', 0.06)
        self.declare_parameter('final_dock_distance_m', 0.18)

        # self.declare_parameter('lateral_tolerance_m', 0.035)
        # self.declare_parameter('bearing_tolerance_rad', 0.060)
        self.declare_parameter('final_lateral_limit_m', 0.060)    # 정렬단계에서 정렬완
        self.declare_parameter('final_bearing_limit_rad', 0.100)

        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('max_approach_speed_mps', 0.070)
        self.declare_parameter('min_approach_speed_mps', 0.018)
        self.declare_parameter('final_approach_speed_mps', 0.025)
        # =========================================================
        self.declare_parameter('contact_push_speed_mps', 0.010)
        # ==========================================================
        self.declare_parameter('max_angular_speed_rps', 0.80)
        self.declare_parameter('max_reverse_speed_mps', 0.040)

        self.declare_parameter('k_z', 0.45)
        self.declare_parameter('k_bearing', 1.80)
        # self.declare_parameter('k_lateral', 1.40)
        self.declare_parameter('k_final_bearing', 1.20)
        self.declare_parameter('k_final_lateral', 0.90)
        # self.declare_parameter('k_normal_yaw', 0.30)

        self.declare_parameter('search_angular_speed_rps', 0.28)
        self.declare_parameter('marker_lost_timeout_sec', 0.70)
        self.declare_parameter('stale_stop_timeout_sec', 0.20)
        # =====================================================
        self.declare_parameter('contact_push_time_sec', 1.20)
        # =====================================================
        self.declare_parameter('charge_verify_timeout_sec', 6.0)
        self.declare_parameter('recovery_backup_time_sec', 1.20)
        self.declare_parameter('max_retry_count', 3)
        self.declare_parameter('max_docking_time_sec', 90.0)

        self.declare_parameter('use_battery_verify', False)
        self.declare_parameter('charge_current_threshold_a', 0.05)
        self.declare_parameter('battery_timeout_sec', 2.0)

        self.declare_parameter('enable_debug_image', True)
        self.declare_parameter('log_throttle_sec', 1.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.battery_topic = self.get_parameter('battery_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.docked_topic = self.get_parameter('docked_topic').value

        self.aruco_dictionary_name = self.get_parameter('aruco_dictionary').value
        self.target_marker_id = int(self.get_parameter('target_marker_id').value)
        self.marker_size_m = float(self.get_parameter('marker_size_m').value)
        self.pre_dock_distance_m = float(self.get_parameter('pre_dock_distance_m').value)
        self.pre_dock_tolerance_m = float(self.get_parameter('pre_dock_tolerance_m').value)
        self.final_dock_distance_m = float(self.get_parameter('final_dock_distance_m').value)
        self.lateral_tolerance_m = float(self.get_parameter('lateral_tolerance_m').value)
        self.bearing_tolerance_rad = float(self.get_parameter('bearing_tolerance_rad').value)
        self.final_lateral_limit_m = float(self.get_parameter('final_lateral_limit_m').value)
        self.final_bearing_limit_rad = float(self.get_parameter('final_bearing_limit_rad').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.max_approach_speed_mps = float(self.get_parameter('max_approach_speed_mps').value)
        self.min_approach_speed_mps = float(self.get_parameter('min_approach_speed_mps').value)
        self.final_approach_speed_mps = float(self.get_parameter('final_approach_speed_mps').value)
        # ==========================================================================================
        self.contact_push_speed_mps = float(self.get_parameter('contact_push_speed_mps').value)
        # ==========================================================================================
        self.max_angular_speed_rps = float(self.get_parameter('max_angular_speed_rps').value)
        self.max_reverse_speed_mps = float(self.get_parameter('max_reverse_speed_mps').value)

        self.k_z = float(self.get_parameter('k_z').value)
        self.k_bearing = float(self.get_parameter('k_bearing').value)
        self.k_lateral = float(self.get_parameter('k_lateral').value)
        self.k_final_bearing = float(self.get_parameter('k_final_bearing').value)
        self.k_final_lateral = float(self.get_parameter('k_final_lateral').value)
        self.k_normal_yaw  = float(self.get_parameter('k_normal_yaw').value)

        self.search_angular_speed_rps = float(self.get_parameter('search_angular_speed_rps').value)
        self.marker_lost_timeout_sec = float(self.get_parameter('marker_lost_timeout_sec').value)
        self.stale_stop_timeout_sec = float(self.get_parameter('stale_stop_timeout_sec').value)
        # ==========================================================================================
        self.contact_push_time_sec = float(self.get_parameter('contact_push_time_sec').value)
        # ==========================================================================================
        self.charge_verify_timeout_sec = float(self.get_parameter('charge_verify_timeout_sec').value)
        self.recovery_backup_time_sec = float(self.get_parameter('recovery_backup_time_sec').value)
        self.max_retry_count = int(self.get_parameter('max_retry_count').value)
        self.max_docking_time_sec = float(self.get_parameter('max_docking_time_sec').value)
        self.use_battery_verify = self._get_bool_parameter('use_battery_verify')
        self.charge_current_threshold_a = float(self.get_parameter('charge_current_threshold_a').value)
        self.battery_timeout_sec = float(self.get_parameter('battery_timeout_sec').value)
        self.enable_debug_image = self._get_bool_parameter('enable_debug_image')
        self.log_throttle_sec = float(self.get_parameter('log_throttle_sec').value)

        self.bridge = CvBridge()

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

        self.latest_observation: Optional[ArucoObservation] = None
        self.last_marker_time = self.get_clock().now() - Duration(seconds=999.0)
        # ===========================================================================
        self.latest_battery: Optional[BatteryState] = None
        self.last_battery_time = self.get_clock().now() - Duration(seconds=999.0)
        # ===========================================================================

        self.state = DockState.SEARCH_MARKER
        self.state_enter_time = self.get_clock().now()
        self.docking_start_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now() - Duration(seconds=999.0)
        self.retry_count = 0

        self.last_tracking_angular_z = 0.0

        self.aruco_dict, self.aruco_params, self.aruco_detector = self._create_aruco_detector(
            self.aruco_dictionary_name
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(CompressedImage,self.image_topic,self.image_callback,sensor_qos,)
        self.camera_info_sub = self.create_subscription(CameraInfo,self.camera_info_topic,self.camera_info_callback,sensor_qos,)
        # =========================================================
        self.battery_sub = self.create_subscription(BatteryState,self.battery_topic,self.battery_callback,10,)
        # ======================================================


        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.debug_pub = self.create_publisher(CompressedImage, self.debug_image_topic, sensor_qos)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        # ===============================================================
        self.docked_pub = self.create_publisher(Bool, self.docked_topic, 10)
        # ================================================================

        timer_period = 1.0 / max(self.control_rate_hz, 0.5)
        self.control_timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f'Charging dock node started. image={self.image_topic}, '
            f'camera_info={self.camera_info_topic}, cmd_vel={self.cmd_vel_topic}, '
            f'battery={self.battery_topic}, marker_id={self.target_marker_id}'
        )

    def get_tracking_observation(
        self,
        lost_reason: str,
    ) -> Optional[ArucoObservation]:
        if self.latest_observation is None:
            self.publish_stop()
            self.start_recovery(lost_reason)
            return None

        observation_age = self._elapsed(self.last_marker_time)

        if observation_age > self.marker_lost_timeout_sec:
            self.publish_stop()
            self.start_recovery(lost_reason)
            return None

        if observation_age > self.stale_stop_timeout_sec:
            self.publish_stop()
            self._throttled_info(
                f'waiting for marker reacquisition, '
                f'age={observation_age:.2f}s'
            )
            return None

        return self.latest_observation

    def _get_bool_parameter(self, name: str) -> bool:
        value = self.get_parameter(name).value

        if isinstance(value, bool):
            return value

        if isinstance(value, str):
            return value.lower() in ['true', '1', 'yes', 'on']

        return bool(value)

    def _create_aruco_detector(self, dictionary_name: str):
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('cv2.aruco 모듈이 없습니다. OpenCV 설치 상태를 확인하세요.')

        if not hasattr(cv2.aruco, dictionary_name):
            valid_names = [name for name in dir(cv2.aruco) if name.startswith('DICT_')]
            raise RuntimeError(
                f'지원하지 않는 ArUco dictionary: {dictionary_name}. '
                f'사용 가능 예: {valid_names[:10]}'
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

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)

            self.get_logger().info(
                'CameraInfo received. ArUco pose estimation is enabled.'
            )

    # ================================================
    def battery_callback(self, msg: BatteryState) -> None:
        self.latest_battery = msg
        self.last_battery_time = self.get_clock().now()
    # =============================================

    def image_callback(self, msg: CompressedImage) -> None:
        
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warn(f'cv_bridge conversion failed: {exc}')
            return

        frame = cv2.flip(frame, -1)
        image_height, image_width = frame.shape[:2]

        # [ 흑백으로 마커 검출 ]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,self.aruco_dict,parameters=self.aruco_params,
            )

        selected_index = None
        observation = None

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten().astype(int)
            selected_index = self._select_marker_index(ids_flat, corners)

            if selected_index is not None:
                observation = self._make_observation(
                    ids_flat,
                    corners,
                    selected_index,
                    image_width,
                    image_height,
                )

                if observation is not None:
                    self.latest_observation = observation
                    self.last_marker_time = self.get_clock().now()

        if self.enable_debug_image:
            debug_frame = self._draw_debug_image(frame,corners,ids,observation,selected_index,)
            try:
                debugout_msg = self.bridge.cv2_to_compressed_imgmsg(debug_frame,dst_format='jpg')
                debugout_msg.header = msg.header
                self.debug_pub.publish(debugout_msg)
            except CvBridgeError as exc:
                self.get_logger().warn(f'debug image publish failed: {exc}')

    def _select_marker_index(self, ids_flat: np.ndarray, corners) -> Optional[int]:
        target_indices = np.where(ids_flat == self.target_marker_id)[0]

        if len(target_indices) > 0:
            return int(target_indices[0])

        if self.target_marker_id < 0:
            areas = [
                abs(cv2.contourArea(corner.reshape(4, 2).astype(np.float32)))
                for corner in corners
            ]
            return int(np.argmax(areas))

        return None

    def _make_observation(
        self,
        ids_flat: np.ndarray,
        corners,
        selected_index: int,
        image_width: int,
        image_height: int,
    ) -> Optional[ArucoObservation]:
        if self.camera_matrix is None or self.dist_coeffs is None:
            self._throttled_warn('CameraInfo is not available. Docking control is waiting.')
            return None

        try:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[selected_index]],
                self.marker_size_m,
                self.camera_matrix,
                self.dist_coeffs,
            )
        except Exception as exc:
            self._throttled_warn(f'ArUco pose estimation failed: {exc}')
            return None

        corner = corners[selected_index].reshape(4, 2)
        center = corner.mean(axis=0)
        area = abs(cv2.contourArea(corner.astype(np.float32)))

        tvec = tvecs[0][0]
        rvec = rvecs[0][0]

        x_m = float(tvec[0])
        y_m = float(tvec[1])
        z_m = float(tvec[2])

        bearing_rad = math.atan2(x_m, max(z_m, 1e-6))

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        marker_normal = rotation_matrix[:, 2].astype(np.float64)

        if marker_normal[2] > 0.0:
            marker_normal = -marker_normal

        marker_normal_yaw_rad = math.atan2(
            float(marker_normal[0]),
            float(-marker_normal[2]),
        )

        marker_normal_yaw_rad = self._normalize_angle(marker_normal_yaw_rad)

        return ArucoObservation(
            marker_id=int(ids_flat[selected_index]),
            x_m=x_m,
            y_m=y_m,
            z_m=z_m,
            bearing_rad=bearing_rad,
            marker_normal_yaw_rad=marker_normal_yaw_rad,
            center_x=float(center[0]),
            center_y=float(center[1]),
            image_width=image_width,
            image_height=image_height,
            area=float(area),
        )

    def control_loop(self) -> None:
        self.publish_state()

        if self._elapsed(self.docking_start_time) > self.max_docking_time_sec:
            if self.state not in [DockState.DOCKED, DockState.FAILED]:
                self.transition_to(DockState.FAILED, 'max docking time exceeded')

        if self.state == DockState.DOCKED:
            self.publish_stop()
            self.publish_docked(True)
            return

        if self.state == DockState.FAILED:
            self.publish_stop()
            self.publish_docked(False)
            return

        if self.is_charging():
            self.transition_to(DockState.DOCKED, 'charging detected')
            self.publish_stop()
            self.publish_docked(True)
            return

        if self.state == DockState.SEARCH_MARKER:
            self.handle_search_marker()
        elif self.state == DockState.APPROACH_PRE_DOCK:
            self.handle_approach_pre_dock()
        elif self.state == DockState.ALIGN_DOCK_AXIS:
            self.handle_align_dock_axis()
        elif self.state == DockState.FINAL_APPROACH:
            self.handle_final_approach()
        # ==========================================================================================
        elif self.state == DockState.CONTACT_PUSH:
            self.handle_contact_push()
        elif self.state == DockState.CHARGE_VERIFY:
            self.handle_charge_verify()
        # ==========================================================================================
        elif self.state == DockState.RECOVERY_BACKUP:
            self.handle_recovery_backup()

    def handle_search_marker(self) -> None:
        obs = self.get_valid_observation()

        if obs is not None:
            self.transition_to(DockState.APPROACH_PRE_DOCK, 'marker acquired')
            return


        elapsed = self._elapsed(self.state_enter_time)

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

        angular_z = (
            direction * min(self.search_angular_speed_rps, 0.12)
        )

        self.publish_cmd(0.0, angular_z)

        self._throttled_info(
            f'SEARCH_MARKER: direction={direction:+.0f}, '
            f'angular_z={angular_z:+.3f}'
        )

    def handle_approach_pre_dock(self) -> None:
        obs = self.get_tracking_observation(
            'marker lost during pre-dock approach'
        )

        if obs is None:
            return

        distance_error = obs.z_m - self.pre_dock_distance_m

        if abs(distance_error) <= self.pre_dock_tolerance_m:
            self.transition_to(DockState.ALIGN_DOCK_AXIS, 'pre-dock distance reached')
            self.publish_stop()
            return

        linear_x = self.k_z * distance_error

        if linear_x > 0.0:
            linear_x = max(self.min_approach_speed_mps, linear_x)

        linear_x = self._clamp(
            linear_x,
            -self.max_reverse_speed_mps,
            self.max_approach_speed_mps,
        )

        angular_z = self.compute_approach_angular(obs)

        self.publish_cmd(linear_x, angular_z)

        self._throttled_info(
            f'APPROACH_PRE_DOCK: z={obs.z_m:.3f}, x={obs.x_m:.3f}, '
            f'bearing={obs.bearing_rad:.3f}, cmd=({linear_x:.3f}, {angular_z:.3f})'
        )

    def handle_align_dock_axis(self) -> None:

        obs = self.get_tracking_observation(
            'marker lost during dock-axis alignment'
        )

        if obs is None:
            return

        aligned = (
            abs(obs.x_m) <= self.final_lateral_limit_m
            and abs(obs.bearing_rad) <= self.final_bearing_limit_rad
        )

        if aligned:
            self.publish_stop()
            self.transition_to(
                DockState.FINAL_APPROACH,
                'dock axis aligned'
            )
            return

        angular_z = self.compute_approach_angular(obs)

        angular_z = self._clamp(
            angular_z,
            -0.08,
            0.08,
        )

        if abs(obs.bearing_rad) > 0.12:
            linear_x = 0.0
        else:
            linear_x = 0.012

        self.publish_cmd(linear_x, angular_z)

    def handle_final_approach(self) -> None:
        obs = self.get_tracking_observation('marker lost during final approach')
        if obs is None:
            return

        obs = self.get_valid_observation()

        if obs is None:
            self.start_recovery('marker lost during final approach')
            return

        if abs(obs.x_m) > self.final_lateral_limit_m:
            self.transition_to(DockState.ALIGN_DOCK_AXIS, 'final lateral error too large')
            self.publish_stop()
            return

        if abs(obs.bearing_rad) > self.final_bearing_limit_rad:
            self.transition_to(DockState.ALIGN_DOCK_AXIS, 'final bearing error too large')
            self.publish_stop()
            return

        if obs.z_m <= self.final_dock_distance_m:
            self.transition_to(DockState.CONTACT_PUSH, 'final dock distance reached')
            self.publish_stop()
            return
        
        angular_z = (
            -self.k_final_bearing * obs.bearing_rad
            -self.k_final_lateral * obs.x_m
        )

        angular_z = self._clamp(
            angular_z,
            -self.max_angular_speed_rps * 0.5,
            self.max_angular_speed_rps * 0.5,
        )

        self.publish_cmd(self.final_approach_speed_mps, angular_z)

        self._throttled_info(
            f'FINAL_APPROACH: z={obs.z_m:.3f}, x={obs.x_m:.3f}, '
            f'bearing={obs.bearing_rad:.3f}, cmd=({self.final_approach_speed_mps:.3f}, {angular_z:.3f})'
        )

    # ==========================================================================================
    def handle_contact_push(self) -> None:
        if self.is_charging():
            self.transition_to(DockState.DOCKED, 'charging detected during contact push')
            self.publish_stop()
            self.publish_docked(True)
            return

        elapsed = self._elapsed(self.state_enter_time)

        if elapsed < self.contact_push_time_sec:
            self.publish_cmd(self.contact_push_speed_mps, 0.0)
            self._throttled_info(
                f'CONTACT_PUSH: gently pushing contacts, elapsed={elapsed:.2f}s'
            )
            return

        self.publish_stop()
        self.transition_to(DockState.CHARGE_VERIFY, 'contact push finished')

    def handle_charge_verify(self) -> None:
        self.publish_stop()

        if not self.use_battery_verify:
            self.transition_to(DockState.DOCKED, 'battery verification disabled')
            self.publish_docked(True)
            return

        if True:
         if self.is_charging():
            self.transition_to(DockState.DOCKED, 'charging verified')
            self.publish_docked(True)
            return

        elapsed = self._elapsed(self.state_enter_time)

        if elapsed > self.charge_verify_timeout_sec:
            self.start_recovery('charging verification failed')
            return

        self._throttled_info(
            f'CHARGE_VERIFY: waiting for charging signal, elapsed={elapsed:.2f}s'
        )
    # ==========================================================================================

    def handle_recovery_backup(self) -> None:

        elapsed = self._elapsed(self.state_enter_time)

        if elapsed < self.recovery_backup_time_sec:
            self.publish_cmd(-self.max_reverse_speed_mps, 0.0)
            self._throttled_info(
                f'RECOVERY_BACKUP: backing up, retry={self.retry_count}/{self.max_retry_count}'
            )
            return

        if self.retry_count >= self.max_retry_count:
            self.transition_to(DockState.FAILED, 'retry count exceeded')
            self.publish_stop()
            return

        self.transition_to(DockState.SEARCH_MARKER, 'recovery finished')

    def compute_approach_angular(self, obs: ArucoObservation) -> float:
        angular_z = -self.k_bearing * obs.bearing_rad
                    
        self.get_logger().info(
            f'x={obs.x_m:+.3f}, '
            f'z={obs.z_m:+.3f}, '
            f'bearing={obs.bearing_rad:+.3f}, '
            f'angular_z={angular_z:+.3f}'
        )

        return self._clamp(
            angular_z,
            -self.max_angular_speed_rps,
            self.max_angular_speed_rps,
        )

    def start_recovery(self, reason: str) -> None:
        self.retry_count += 1
        self.transition_to(DockState.RECOVERY_BACKUP, reason)

    def transition_to(self, new_state: DockState, reason: str = '') -> None:
        if self.state == new_state:
            return

        old_state = self.state
        self.state = new_state
        self.state_enter_time = self.get_clock().now()

        self.get_logger().info(
            f'STATE: {old_state.value} -> {new_state.value}. reason={reason}'
        )

    def get_valid_observation(self) -> Optional[ArucoObservation]:
        if self.latest_observation is None:
            return None

        if self._elapsed(self.last_marker_time) > self.marker_lost_timeout_sec:
            return None

        return self.latest_observation

    # ===============================================================================
    def is_charging(self) -> bool:
        if not self.use_battery_verify:
            return False

        if self.latest_battery is None:
            return False

        if self._elapsed(self.last_battery_time) > self.battery_timeout_sec:
            return False

        if self.latest_battery.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            return True

        current = self.latest_battery.current

        if not math.isnan(current) and current > self.charge_current_threshold_a:
            return True

        return False

    def publish_cmd(self, linear_x: float, angular_z: float) -> None:
        cmd = Twist()

        cmd.linear.x = self._clamp(
            linear_x,
            -self.max_reverse_speed_mps,
            self.max_approach_speed_mps,
        )

        cmd.angular.z = self._clamp(
            angular_z,
            -self.max_angular_speed_rps,
            self.max_angular_speed_rps,
        )

        if (
            self.state in [
                DockState.APPROACH_PRE_DOCK,
                DockState.ALIGN_DOCK_AXIS,
                DockState.FINAL_APPROACH,
            ]
            and abs(cmd.angular.z) > 0.01
        ):
            self.last_tracking_angular_z = cmd.angular.z

        self.cmd_pub.publish(cmd)
    # ============================================================================

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def publish_state(self) -> None:
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    # =================================================================================
    def publish_docked(self, docked: bool) -> None:
        msg = Bool()
        msg.data = docked
        self.docked_pub.publish(msg)
    # ==============================================================================

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

        state_text = f'state={self.state.value}'
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
                f'retry={self.retry_count}/{self.max_retry_count}'
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

    def destroy_node(self):
        for _ in range(5):
            self.publish_stop()

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = ChargingDockNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()