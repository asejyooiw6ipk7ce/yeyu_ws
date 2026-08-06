#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GUI <-> ROS2 파이프라인 검증용 가짜 퍼블리셔.

실제 driving_node가 아직 완성되지 않았으므로, /driving_status 토픽에
JSON 문자열(std_msgs/String)로 가짜 주행 상태를 1초마다 흘려보낸다.
GUI의 QThread(ros_worker.py)가 이 토픽을 구독해서 값이 잘 들어오는지 확인하는 용도.

실행 (터미널에서 GUI와 별개로):
    source /opt/ros/humble/setup.bash
    python3 fake_publisher_test.py
"""
import json
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 실제 STAGES 순서와 동일하게 맞춤 (theme.py 참고)
STAGE_KEYS = ["idle", "s_course", "crank", "parking", "signal", "accel", "result"]


class FakeDrivingStatusPublisher(Node):
    def __init__(self):
        super().__init__("fake_driving_status_publisher")
        self.publisher_ = self.create_publisher(String, "/driving_status", 10)
        self.timer = self.create_timer(1.0, self._tick)
        self._stage_index = 0
        self._battery = 78.0
        self.get_logger().info("가짜 /driving_status 퍼블리시 시작 (1초 간격)")

    def _tick(self):
        # 5번에 한 번꼴로 다음 구간으로 진행하는 척
        if random.random() < 0.2:
            self._stage_index = (self._stage_index + 1) % len(STAGE_KEYS)

        self._battery = max(0.0, self._battery - 0.05)

        payload = {
            "stage": STAGE_KEYS[self._stage_index],
            "battery_percent": round(self._battery, 1),
            "retry_count": random.randint(0, 3),
            "ros_connected": True,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.publisher_.publish(msg)
        self.get_logger().info(f"publish: {msg.data}")


def main():
    rclpy.init()
    node = FakeDrivingStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
