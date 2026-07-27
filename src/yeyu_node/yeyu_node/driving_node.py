import math
import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from yeyu_msgs.msg import DrivingStatus
from yeyu_node.driving_mode import DrivingMode
from ament_index_python.packages import get_package_share_directory

NAV_ARRIVAL_TRANSITIONS = {
    DrivingMode.NAV_TO_PARKING: DrivingMode.PARKING,
    DrivingMode.NAV_TO_SIGNAL: DrivingMode.SIGNAL_WAIT,
    DrivingMode.NAV_TO_ACCEL: DrivingMode.ACCEL_ZONE,
}


class DrivingNode(Node):
    def __init__(self):
        super().__init__('driving_node')

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

        # --- 3. 구독/발행 ---
        # self.create_subscription(LaserScan, '/scan', self.on_lidar, 10)
        # self.create_subscription(Image, '/camera/image_raw', self.on_camera, 10)
        self.pub_led = self.create_publisher(String, '/led_command', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(DrivingStatus, '/driving_status', 10)

        # --- 4. 초기 상태: 첫 웨이포인트(직각주차)로 출발 ---
        self.mode = DrivingMode.NAV_TO_PARKING
        self.send_waypoint(self.waypoints[0])   # ①

    # ================= Nav2 제어 =================

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
        if not goal_handle.accepted:
            self.get_logger().warn('경로 목표가 거부됨')
            return
        self.current_goal_handle = goal_handle          # ★ 취소하려면 이 핸들이 꼭 있어야 함
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)

    def pause_nav(self):
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()

    def resume_nav(self, wp):
        self.send_waypoint(wp)

    def on_nav_result(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
                next_mode = NAV_ARRIVAL_TRANSITIONS.get(self.mode)
                if next_mode is not None:
                    self.get_logger().info(f'{self.mode.name} -> {next_mode.name} 모드 전환')
                    self.mode = next_mode
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
    #     self.mode = DrivingMode.NAV_TO_PARKING

    # # ================= 카메라: 직각주차/신호/가속 =================

    # def on_camera(self, msg):
    #     frame = msg  # TODO: cv_bridge로 실제 cv2 이미지 변환 필요

    #     if self.mode == DrivingMode.PARKING:
    #         pose = detect_aruco_pose(frame)
    #         if pose is not None and aligned(pose):
    #             self.wp_index = 1
    #             self.send_waypoint(self.waypoints[1])                  # ⑤
    #             self.mode = DrivingMode.NAV_TO_SIGNAL

    #     elif self.mode == DrivingMode.NAV_TO_SIGNAL:
    #         if reached_stop_line():                                     # ⑥
    #             self.pause_nav()
    #             self.mode = DrivingMode.SIGNAL_WAIT

    #     elif self.mode == DrivingMode.SIGNAL_WAIT:
    #         color = detect_signal_color(frame)
    #         if color == 'green':
    #             self.green_count += 1
    #             if self.green_count >= 3:
    #                 self.wp_index = 2
    #                 self.resume_nav(self.waypoints[2])                  # ⑧
    #                 self.mode = DrivingMode.NAV_TO_ACCEL
    #                 self.green_count = 0
    #         else:
    #             self.green_count = 0

    #     elif self.mode == DrivingMode.NAV_TO_ACCEL:
    #         if detect_speed_sign(frame):                                 # ⑨
    #             self.mode = DrivingMode.ACCEL_ZONE
    #             self.accelerate_to(0.2)


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