import math

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, LaserScan, CompressedImage
from std_msgs.msg import Float32 
from std_srvs.srv import Trigger
from yeyu_msgs.msg import DrivingStatus
from yeyu_msgs.srv import StartRetry

from cv_bridge import CvBridge, CvBridgeError

from PyQt5.QtCore import QObject, QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped


DRIVING_NODE_NAME = 'driving_node'
NO_OBSTACLE_READING = -1.0


class RosSignals(QObject):
    """DashboardRosNode의 콜백(스핀 스레드)에서 GUI 스레드로 데이터를 전달하는 신호 모음."""

    driving_status = pyqtSignal(dict)
    odom = pyqtSignal(float, float)
    battery = pyqtSignal(float, float, int)
    obstacle = pyqtSignal(float)
    image = pyqtSignal(QImage)
    s_course_debug_image = pyqtSignal(QImage)
    parking_debug_image = pyqtSignal(QImage)
    ros_connected = pyqtSignal(bool)
    estop_result = pyqtSignal(bool, str)
    retry_result = pyqtSignal(str, bool, str)
    map_data = pyqtSignal(QImage, float, float, float)   # 이미지, resolution, origin_x, origin_y
    robot_pose = pyqtSignal(float, float, float)          # x, y, yaw


class DashboardRosNode(Node):
    """구독/서비스 호출만 담당하는 순수 표시·전달용 노드."""

    def __init__(self):
        super().__init__('yeyu_dashboard_gui')

        self.signals = RosSignals()
        self.bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(DrivingStatus, '/driving_status', self.on_driving_status, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(BatteryState, '/battery_state', self.on_battery, sensor_qos)
        # self.create_subscription(LaserScan, '/scan', self.on_scan, sensor_qos)
        self.create_subscription(
            Float32, 'sensor_bridge/obstacle_distance_cm', self.on_obstacle_distance, sensor_qos)  
        self.create_subscription(
            CompressedImage, '/camera/image_flipped/compressed', self.on_image, sensor_qos)
        self.create_subscription(   
            CompressedImage, '/s_course_debug_image/compressed', self.on_s_course_debug_image, sensor_qos)
        self.create_subscription(                                                        
            CompressedImage, '/parking_debug_image/compressed', self.on_parking_debug_image, sensor_qos)
        self.create_subscription(
            OccupancyGrid, '/map', self.on_map, 10)
        # /amcl_pose는 이미 driving_node가 구독하지만, GUI도 따로 구독 가능
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, 10)

        self.estop_client = self.create_client(Trigger, '/emergency_stop')
        self.retry_client = self.create_client(StartRetry, '/start_retry')

        self.create_timer(1.0, self._check_connection)

    # ================= 구독 콜백 =================
    def _check_connection(self):
        connected = DRIVING_NODE_NAME in self.get_node_names()
        self.signals.ros_connected.emit(connected)

    def on_driving_status(self, msg: DrivingStatus):
        self.signals.driving_status.emit({
            'mode': msg.mode,
            'result': msg.result,
            'reason': msg.reason,
            'wp_index': msg.wp_index,
            'error_lateral': msg.error_lateral,
            'error_distance': msg.error_distance,
            'retry_count': msg.retry_count,
        })

    def on_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.signals.odom.emit(pos.x, pos.y)

    def on_battery(self, msg: BatteryState):
        percentage = msg.percentage if math.isfinite(msg.percentage) else -1.0
        self.signals.battery.emit(msg.voltage, percentage, msg.power_supply_status)

    # def on_scan(self, msg: LaserScan):
    #     valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
    #     min_range = min(valid) if valid else NO_OBSTACLE_READING
        # self.signals.obstacle.emit(min_range)

    def on_obstacle_distance(self, msg: Float32):   # [추가] on_scan을 대체
        distance_m = msg.data / 100.0   # cm → m 변환 (기존 signals.obstacle가 m 단위를 쓰고 있어서 통일)
        if msg.data >= 255.0:   # 아두이노 펌웨어에서 255는 "응답없음/범위밖" 약속값
            self.signals.obstacle.emit(NO_OBSTACLE_READING)
            return
        self.signals.obstacle.emit(distance_m)

    def on_image(self, msg: CompressedImage):
        if not msg.data:
            return
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except (CvBridgeError, cv2.error) as e:
            self.get_logger().warn(f'camera image decode 실패: {e}')
            return
        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        rgb = np.ascontiguousarray(rgb)
        h, w, ch = rgb.shape
        qimage = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
        self.signals.image.emit(qimage)

    def on_s_course_debug_image(self, msg: CompressedImage):   # [추가] on_debug_image와 동일 패턴
        if not msg.data:
            return
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except (CvBridgeError, cv2.error) as e:
            self.get_logger().warn(f'scourse debug image decode 실패: {e}')
            return
        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        rgb = np.ascontiguousarray(rgb)
        h, w, ch = rgb.shape
        qimage = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
        self.signals.s_course_debug_image.emit(qimage)

    def on_parking_debug_image(self, msg: CompressedImage):
        if not msg.data:
            return
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except (CvBridgeError, cv2.error) as e:
            self.get_logger().warn(f'debug image decode 실패: {e}')
            return
        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        rgb = np.ascontiguousarray(rgb)
        h, w, ch = rgb.shape
        qimage = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
        self.signals.parking_debug_image.emit(qimage)

    def on_map(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        # -1(미탐사)=회색, 0(빈공간)=흰색, 100(장애물)=검정
        img = np.zeros((h, w), dtype=np.uint8)
        img[data == -1] = 128
        img[data == 0] = 255
        img[data == 100] = 0
        img = np.flipud(img)  # OccupancyGrid는 아래→위 순서라 뒤집어줘야 화면과 맞음
        img = np.ascontiguousarray(img)
        qimage = QImage(img.data, w, h, w, QImage.Format_Grayscale8).copy()
        self.signals.map_data.emit(qimage, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.signals.robot_pose.emit(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    # ================= 서비스 호출 =================
    def call_emergency_stop(self):
        if not self.estop_client.service_is_ready():
            self.signals.estop_result.emit(False, '/emergency_stop 서비스에 연결할 수 없습니다.')
            return
        future = self.estop_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_estop_response)

    def _on_estop_response(self, future):
        try:
            res = future.result()
            self.signals.estop_result.emit(res.success, res.message)
        except Exception as e:
            self.signals.estop_result.emit(False, str(e))

    def call_start_retry(self, target: str):
        self.get_logger().info(f'[DEBUG] call_start_retry target={target!r}')
        if not self.retry_client.service_is_ready():
            self.signals.retry_result.emit(target, False, '/start_retry 서비스에 연결할 수 없습니다.')
            return
        req = StartRetry.Request()
        req.target = target
        self.get_logger().info(f'[DEBUG] req.target={req.target!r}')
        future = self.retry_client.call_async(req)
        future.add_done_callback(lambda f: self._on_retry_response(target, f))

    def _on_retry_response(self, target, future):
        try:
            res = future.result()
            self.signals.retry_result.emit(target, res.accepted, res.message)
        except Exception as e:
            self.signals.retry_result.emit(target, False, str(e))


class RosSpinThread(QThread):
    """DashboardRosNode를 MultiThreadedExecutor로 스핀하는 백그라운드 스레드."""

    def __init__(self, node: DashboardRosNode, parent=None):
        super().__init__(parent)
        self.node = node
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(node)

    def run(self):
        try:
            self.executor.spin()
        except rclpy.executors.ExternalShutdownException:
            pass

    def stop(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.wait(2000)