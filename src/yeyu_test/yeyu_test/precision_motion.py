#!/usr/bin/env python3
"""
precision_motion.py
------------------------------------------------------------
TurtleBot3 Burger 직진 / 회전 정밀도 시험용 모션 프리미티브

- /odom 을 구독하여 오도메트리 기준 이동량(거리·각도)을 실시간 추적
- 목표 거리/각도에 근접하면 감속(비례 제어) 후 정지 -> 오버슈트 최소화
- 이 노드가 계산하는 값은 '오도메트리 기준' 참고값일 뿐이며,
  실제 정밀도 평가는 반드시 줄자·각도기로 측정한 실측값으로 해야 함
  (엔코더/오도메트리 자체가 오차를 포함하므로 자기 검증이 될 수 없음)

ROS2 Humble / rclpy 기준
------------------------------------------------------------
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def quaternion_to_yaw(q) -> float:
    """geometry_msgs/Quaternion -> yaw(rad), Z축 회전만 사용 (평면 주행 가정)"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """각도를 -pi ~ +pi 범위로 정규화 (wrap-around 처리)"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PrecisionMover(Node):
    """
    직진(move_straight)과 제자리 회전(rotate)을 오도메트리 폐루프로 실행하는 노드.
    사용 예:
        mover = PrecisionMover()
        mover.move_straight(1.0)   # 전방 1m 전진
        mover.rotate(90.0)         # 반시계 90도 회전
        mover.rotate(-180.0)       # 시계 180도 회전
    """

    def __init__(self):
        super().__init__('precision_mover')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False

    # ---------------- 내부 콜백 ----------------
    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_ready = True

    def _spin_until_odom(self):
        while not self.odom_ready and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def stop(self):
        self.cmd_pub.publish(Twist())

    # ---------------- 직진 ----------------
    def move_straight(self, distance_m: float, lin_speed: float = 0.12,
                       tolerance_m: float = 0.002) -> float:
        """
        distance_m > 0 : 전진, < 0 : 후진
        tolerance_m    : 정지 판정 오차 허용치 (오도메트리 기준)
        반환값: 오도메트리로 측정한 실제 이동 거리(m) - 참고용
        """
        self._spin_until_odom()
        x0, y0 = self.x, self.y
        direction = 1.0 if distance_m >= 0 else -1.0
        target = abs(distance_m)

        twist = Twist()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            traveled = math.hypot(self.x - x0, self.y - y0)
            remaining = target - traveled
            if remaining <= tolerance_m:
                break
            # 목표에 가까워질수록 감속 (남은 15cm 구간에서 비례 감속)
            speed = direction * min(lin_speed, max(0.03, lin_speed * (remaining / 0.15)))
            twist.linear.x = speed
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

        self.stop()
        return math.hypot(self.x - x0, self.y - y0)

    # ---------------- 회전 ----------------
    def rotate(self, angle_deg: float, ang_speed: float = 0.4,
               tolerance_deg: float = 0.5) -> float:
        """
        angle_deg > 0 : 반시계(CCW), < 0 : 시계(CW)
        누적 방식으로 회전량을 계산하여 180도 부근 wrap-around 문제를 회피
        반환값: 오도메트리로 측정한 실제 회전각(deg) - 참고용
        """
        self._spin_until_odom()
        prev_yaw = self.yaw
        turned_total = 0.0
        target_rad = math.radians(angle_deg)
        direction = 1.0 if angle_deg >= 0 else -1.0
        tol_rad = math.radians(tolerance_deg)

        twist = Twist()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            dyaw = normalize_angle(self.yaw - prev_yaw)
            turned_total += dyaw
            prev_yaw = self.yaw

            remaining = abs(target_rad) - abs(turned_total)
            if remaining <= tol_rad:
                break
            speed = direction * min(ang_speed, max(0.15, ang_speed * (remaining / math.radians(30))))
            twist.linear.x = 0.0
            twist.angular.z = speed
            self.cmd_pub.publish(twist)

        self.stop()
        return math.degrees(turned_total)