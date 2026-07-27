#!/usr/bin/env python3

import argparse
import math
import sys
import threading
from pathlib import Path

import yaml

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import PoseWithCovarianceStamped


def quaternion_to_yaw(q):
    """
    Quaternion 값을 yaw 값으로 변환한다.

    YEYU 로봇은 2D 평면에서 움직이므로
    roll, pitch는 거의 사용하지 않고 yaw만 사용한다.
    """
    siny_cosp = 2.0 * ((q.w * q.z) + (q.x * q.y))
    cosy_cosp = 1.0 - 2.0 * ((q.y * q.y) + (q.z * q.z))

    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


class AmclWaypointRecorder(Node):
    def __init__(self, output_file: str, frame_id: str):
        super().__init__("amcl_waypoint_recorder")

        self.output_file = Path(output_file).expanduser()
        self.frame_id = frame_id

        self.current_pose = None
        self.waypoints = []
        self.running = True

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_pose_callback,
            10
        )

        self.get_logger().info("AMCL waypoint recorder started.")
        self.get_logger().info("Subscribing: /amcl_pose")
        self.get_logger().info(f"Output YAML: {self.output_file}")
        self.get_logger().info("Move robot to target position, then press Enter to save waypoint.")
        self.get_logger().info("Type q and press Enter to save file and quit.")

        self.input_thread = threading.Thread(target=self.keyboard_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        /amcl_pose 토픽에서 현재 로봇 위치를 계속 갱신한다.
        """
        pose = msg.pose.pose

        x = pose.position.x
        y = pose.position.y
        yaw = quaternion_to_yaw(pose.orientation)

        self.current_pose = {
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw)
        }

    def keyboard_loop(self):
        """
        Enter 입력을 받을 때마다 현재 AMCL 위치를 waypoint로 저장한다.
        """
        while self.running:
            user_input = input()

            if user_input.lower() == "q":
                self.running = False
                self.save_yaml()
                rclpy.shutdown()
                break

            self.save_current_pose_as_waypoint()

    def save_current_pose_as_waypoint(self):
        """
        현재 AMCL pose를 waypoints 리스트에 추가한다.
        """
        if self.current_pose is None:
            self.get_logger().warn("No AMCL pose received yet. Check /amcl_pose.")
            return

        index = len(self.waypoints) + 1
        waypoint_name = f"wp{index}"

        waypoint = {
            "name": waypoint_name,
            "x": round(self.current_pose["x"], 3),
            "y": round(self.current_pose["y"], 3),
            "yaw": round(self.current_pose["yaw"], 3)
        }

        self.waypoints.append(waypoint)

        self.get_logger().info(
            f"Saved {waypoint_name}: "
            f"x={waypoint['x']}, y={waypoint['y']}, yaw={waypoint['yaw']}"
        )

    def save_yaml(self):
        """
        저장된 waypoint 목록을 YAML 파일로 저장한다.
        """
        if len(self.waypoints) == 0:
            self.get_logger().warn("No waypoints saved. YAML file will not be created.")
            return

        yaml_data = {
            "frame_id": self.frame_id,
            "initial_pose": {
                "x": self.waypoints[0]["x"],
                "y": self.waypoints[0]["y"],
                "yaw": self.waypoints[0]["yaw"]
            },
            "waypoints": self.waypoints
        }

        self.output_file.parent.mkdir(parents=True, exist_ok=True)

        with open(self.output_file, "w", encoding="utf-8") as file:
            yaml.dump(
                yaml_data,
                file,
                allow_unicode=True,
                sort_keys=False,
                default_flow_style=False
            )

        self.get_logger().info(f"Saved waypoint YAML: {self.output_file}")


def parse_arguments():
    argv = remove_ros_args(args=sys.argv)[1:]

    parser = argparse.ArgumentParser(
        description="Record YEYU AMCL pose as waypoint YAML"
    )

    parser.add_argument(
        "--output",
        default="~/yeyu_ws/src/yeyu_waypoint_nav/waypoints/recorded_waypoints.yaml",
        help="Output waypoint YAML file path"
    )

    parser.add_argument(
        "--frame-id",
        default="map",
        help="Frame ID for waypoint YAML"
    )

    return parser.parse_args(argv)


def main():
    args = parse_arguments()

    rclpy.init()

    node = AmclWaypointRecorder(
        output_file=args.output,
        frame_id=args.frame_id
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().warn("Keyboard interrupt received.")
        node.save_yaml()

    finally:
        node.running = False
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
