# main_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from yeyu_msgs.msg import SystemState as SystemStateMsg, DrivingStatus, FunctionStatus
from src.yeyu_control.yeyu_control.system_state import SystemState

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.state = SystemState.IDLE
        self.state_entry_time = self.get_clock().now()

        self.pub_state = self.create_publisher(SystemStateMsg, '/system_state', 10)
        self.pub_tts = self.create_publisher(String, '/tts_request', 10)

        self.create_subscription(DrivingStatus, '/driving_status', self.on_driving_status, 10)
        self.create_subscription(FunctionStatus, '/line_status', self.on_line_status, 10)

        self.create_timer(0.1, self.tick)  # 10Hz

    def set_state(self, new_state, tts_msg=None):
        self.get_logger().info(f'{self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_entry_time = self.get_clock().now()
        msg = SystemStateMsg()
        msg.state = new_state.name
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_state.publish(msg)
        if tts_msg:
            self.pub_tts.publish(String(data=tts_msg))

    def on_driving_status(self, msg):
        if msg.mode == 'PARKING' and self.state == SystemState.PARKING:
            if msg.result == 'PASS':
                self.set_state(SystemState.SIGNAL_WAIT)
            elif msg.result == 'FAIL':
                self.set_state(SystemState.RETRY)

    def on_line_status(self, msg):
        if msg.status == 'LOST' and self.state == SystemState.LINE_TRACING:
            self.set_state(SystemState.RETRY)

    def tick(self):
        # TODO: 비상정지 조건 검사 (Wi-Fi, 센서 타임아웃 등)
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()