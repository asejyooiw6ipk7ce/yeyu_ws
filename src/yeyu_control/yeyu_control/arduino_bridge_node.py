# arduino_bridge_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')

        self.declare_parameter('port', '/dev/ttyACM1')   # 실제 포트로 확정 필요
        self.declare_parameter('baudrate', 9600)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.serial_conn = None
        try:
            self.serial_conn = serial.Serial(port, baudrate, timeout=1.0)
            time.sleep(2.0)  # 아두이노 리셋 후 부팅 대기 (필수, 안 하면 첫 명령 씹힘)
            self.get_logger().info(f'아두이노 시리얼 연결 성공: {port} @ {baudrate}')
        except serial.SerialException as e:
            self.get_logger().error(f'아두이노 시리얼 연결 실패: {e}')

        self.create_subscription(String, '/led_command', self.on_led_command, 10)

    def on_led_command(self, msg):
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().warn('시리얼 연결이 없어 LED 명령을 보낼 수 없습니다')
            return

        try:
            command = msg.data.strip() + '\n'   # 펌웨어가 readStringUntil('\n')으로 받으므로 개행 필수
            self.serial_conn.write(command.encode('utf-8'))
            self.get_logger().info(f'[arduino_bridge] 전송: {msg.data}')
        except serial.SerialException as e:
            self.get_logger().warn(f'시리얼 전송 실패: {e}')

    def destroy_node(self):
        if self.serial_conn is not None and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()