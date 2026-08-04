#!/usr/bin/env python3
import threading
from typing import List, Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, ColorRGBA, UInt8
import serial

# 통신규격 암호 변수
START_BYTE = 0xAA          # 편지봉투 앞 스티커
END_BYTE_1 = 0xAA          # 편지봉투 뒷 스티커 반쪽
END_BYTE_2 = 0xEE          # 편지봉투 뒷 스티커 반쪽
PKT_SENSOR_STATE = 0x31    # 아두이노가 보낸 상태보고서 편지번호
PKT_SET_RGB = 0x42         # LED 명령 할 편지 번호
PKT_PING = 0x7F            # 생사확인할 편지 번호
MAX_PAYLOAD_LEN = 32       # 편지 속에 담길 수 있는 최대 알맹이 크기


class ParserState:
    WAIT_START_1 = 0
    WAIT_START_2 = 1
    WAIT_START_3 = 2
    READ_PACKET_ID = 3
    READ_LENGTH = 4
    READ_SEQUENCE = 5
    READ_PAYLOAD = 6
    READ_CHECKSUM = 7
    READ_END_1 = 8
    READ_END_2 = 9


class ArduinoSensorBridge(Node):

    def __init__(self):
        super().__init__('arduino_sensor_bridge')

        self.declare_parameter('port', '/dev/tb3_sensor')
        # [설명] 기본 Baudrate는 Arduino 펌웨어 설정과 일치하도록 115200으로 설정합니다.
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('read_period_sec', 0.005)
        self.declare_parameter('ping_period_sec', 1.0)

        self.port = self.get_parameter('port').value                                  # 아두이노-컴퓨터 연결포트
        self.baudrate = int(self.get_parameter('baudrate').value)                     # 시리얼 통신 속도값
        self.read_period_sec = float(self.get_parameter('read_period_sec').value)     # 우체통을 들여다보는 주기
        self.ping_period_sec = float(self.get_parameter('ping_period_sec').value)     # 생사확인 편지 보내는 주기

        self.serial_port: Optional[serial.Serial] = None                              # 진짜로 열린 시리얼 우체통 객체 저장되는 방
        self.serial_lock = threading.Lock()                                           # 편지를 쓰고 읽을 때 데이터가 꼬이지 않게 문을 잠그는 장치

        self.tx_sequence = 0

        self.parser_state = ParserState.WAIT_START_1
        self.rx_packet_id = 0
        self.rx_length = 0
        self.rx_sequence = 0
        self.rx_payload: List[int] = []
        self.rx_checksum = 0

        # 퍼블리셔 선언
        self.ir_l_pub = self.create_publisher(Bool, 'sensor_bridge/ir_l_state', 10)
        self.ir_c_pub = self.create_publisher(Bool, 'sensor_bridge/ir_c_state', 10)
        self.ir_r_pub = self.create_publisher(Bool, 'sensor_bridge/ir_r_state', 10)
        self.rgb_state_pub = self.create_publisher(ColorRGBA, 'sensor_bridge/rgb_state', 10)
        self.rx_sequence_pub = self.create_publisher(UInt8, 'sensor_bridge/rx_sequence', 10)
        
        # 서브스크라이버 선언
        self.rgb_cmd_sub = self.create_subscription(ColorRGBA, 'sensor_bridge/rgb_cmd', self.rgb_cmd_callback, 10)

        # 시리얼 포트 오픈
        self.open_serial()

        # 타이머 선언
        self.read_timer = self.create_timer(self.read_period_sec, self.read_serial_timer_callback)
        self.ping_timer = self.create_timer(self.ping_period_sec, self.ping_timer_callback)

    def open_serial(self):
        try:
            # [설명] 시리얼 포트는 pyserial 라이브러리를 통해 오픈합니다.
            # timeout=0.0 설정은 non-blocking read 모드입니다.
            # 데이터가 없어도 무한정 대기하지 않고 즉시 리턴하므로, ROS 2 타이머 콜백이 시리얼 입력 대기 때문에 멈추는 현상을 방지합니다.
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,
                write_timeout=0.1
            )
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.get_logger().info(f'Opened serial port {self.port} at {self.baudrate} baud.')
        except serial.SerialException as e:
            self.serial_port = None
            self.get_logger().error(f'Failed to open serial port {self.port}: {e}')

    def calc_checksum(self, packet_id: int, length: int, sequence: int, payload: List[int]) -> int:
        checksum_sum = 0
        checksum_sum += packet_id
        checksum_sum += length
        checksum_sum += sequence

        for byte_value in payload:
            checksum_sum += byte_value

        return checksum_sum & 0xFF

    def send_packet(self, packet_id: int, payload: List[int]):
        if self.serial_port is None or not self.serial_port.is_open:
            return

        if len(payload) > 255:
            self.get_logger().error('Payload too large.')
            return

        length = len(payload)
        sequence = self.tx_sequence & 0xFF
        checksum = self.calc_checksum(packet_id, length, sequence, payload)

        packet = bytearray()
        packet.append(START_BYTE)
        packet.append(START_BYTE)
        packet.append(START_BYTE)

        packet.append(packet_id & 0xFF)
        packet.append(length & 0xFF)
        packet.append(sequence)

        for byte_value in payload:
            packet.append(byte_value & 0xFF)

        packet.append(checksum)
        packet.append(END_BYTE_1)
        packet.append(END_BYTE_2)

        try:
            with self.serial_lock:
                self.serial_port.write(packet)
            self.tx_sequence = (self.tx_sequence + 1) & 0xFF
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write failed: {e}')

    def clamp_color_to_u8(self, value: float) -> int:
        if value <= 0.0:
            return 0
        if value >= 1.0:
            return 255
        return int(value * 255.0)

    # [설명] RGB LED 제어 명령 콜백 함수입니다.
    # 예시: ROS 2에서 r=1.0, g=0.0, b=0.0 (빨간색) 명령을 받으면,
    # clamp_color_to_u8를 거쳐 [255, 0, 0] (16진수로 FF 00 00) 데이터가 Payload로 구성되어 아두이노로 전송됩니다.
    def rgb_cmd_callback(self, msg: ColorRGBA):
        red = self.clamp_color_to_u8(msg.r)
        green = self.clamp_color_to_u8(msg.g)
        blue = self.clamp_color_to_u8(msg.b)
        self.send_packet(PKT_SET_RGB, [red, green, blue])


    def ping_timer_callback(self):
        self.send_packet(PKT_PING, [])

    def read_serial_timer_callback(self):
        if self.serial_port is None or not self.serial_port.is_open:
            return

        try:
            waiting = self.serial_port.in_waiting
            if waiting <= 0:
                return

            with self.serial_lock:
                data = self.serial_port.read(waiting)

            for byte_value in data:
                self.parse_byte(byte_value)
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read failed: {e}')

    def reset_parser(self):
        self.parser_state = ParserState.WAIT_START_1
        self.rx_packet_id = 0
        self.rx_length = 0
        self.rx_sequence = 0
        self.rx_payload = []
        self.rx_checksum = 0

    def parse_byte(self, byte_in: int):
        byte_in = byte_in & 0xFF

        if self.parser_state == ParserState.WAIT_START_1:
            if byte_in == START_BYTE:
                self.parser_state = ParserState.WAIT_START_2

        elif self.parser_state == ParserState.WAIT_START_2:
            if byte_in == START_BYTE:
                self.parser_state = ParserState.WAIT_START_3
            else:
                self.parser_state = ParserState.WAIT_START_1

        elif self.parser_state == ParserState.WAIT_START_3:
            if byte_in == START_BYTE:
                self.parser_state = ParserState.READ_PACKET_ID
            else:
                self.parser_state = ParserState.WAIT_START_1

        elif self.parser_state == ParserState.READ_PACKET_ID:
            self.rx_packet_id = byte_in
            self.parser_state = ParserState.READ_LENGTH

        elif self.parser_state == ParserState.READ_LENGTH:
            self.rx_length = byte_in
            if self.rx_length > MAX_PAYLOAD_LEN:
                self.get_logger().warn(f'Invalid payload length: {self.rx_length}')
                self.reset_parser()
                return
            self.rx_payload = []
            self.parser_state = ParserState.READ_SEQUENCE

        elif self.parser_state == ParserState.READ_SEQUENCE:
            self.rx_sequence = byte_in
            if self.rx_length == 0:
                self.parser_state = ParserState.READ_CHECKSUM
            else:
                self.parser_state = ParserState.READ_PAYLOAD

        elif self.parser_state == ParserState.READ_PAYLOAD:
            self.rx_payload.append(byte_in)
            if len(self.rx_payload) >= self.rx_length:
                self.parser_state = ParserState.READ_CHECKSUM

        elif self.parser_state == ParserState.READ_CHECKSUM:
            self.rx_checksum = byte_in
            self.parser_state = ParserState.READ_END_1

        elif self.parser_state == ParserState.READ_END_1:
            if byte_in == END_BYTE_1:
                self.parser_state = ParserState.READ_END_2
            else:
                self.reset_parser()

        elif self.parser_state == ParserState.READ_END_2:
            if byte_in == END_BYTE_2:
                # [설명] 종료 바이트까지 정상 수신되면 Checksum을 계산하여 검증 단계를 거칩니다.
                calculated = self.calc_checksum(
                    self.rx_packet_id,
                    self.rx_length,
                    self.rx_sequence,
                    self.rx_payload
                )

                # [설명] 검증 값과 수신된 체크섬이 일치할 때만 데이터를 패킷 처리 함수로 넘깁니다.
                if calculated == self.rx_checksum:
                    self.handle_packet(
                        self.rx_packet_id,
                        self.rx_length,
                        self.rx_sequence,
                        self.rx_payload
                    )
                else:
                    self.get_logger().warn(
                        f'Checksum mismatch. packet_id=0x{self.rx_packet_id:02X}, '
                        f'rx=0x{self.rx_checksum:02X}, calc=0x{calculated:02X}'
                    )
            self.reset_parser()
        else:
            self.reset_parser()

    def handle_packet(self, packet_id: int, length: int, sequence: int, payload: List[int]):
        # [설명] 아두이노로부터 상태 패킷인 0x31 (PKT_SENSOR_STATE)이 들어오면 parse_sensor_state 함수를 호출합니다.
        if packet_id == PKT_SENSOR_STATE:
            self.parse_sensor_state(length, sequence, payload)
        else:
            self.get_logger().debug(f'Unknown packet id: 0x{packet_id:02X}')

    # [설명] 아두이노에서 전송한 상태 데이터를 해석하는 함수입니다.
    def parse_sensor_state(self, length: int, sequence: int, payload: List[int]):
        # [설명] 센서 상태 수신 데이터의 페이로드는 반드시 5바이트여야 합니다. (트래킹 L/C/R, R, G, B)
        if length != 6:
            self.get_logger().warn(f'Invalid sensor state payload length: {length}')
            return

        # [설명] 페이로드 바이트 배열을 각각의 하드웨어 상태 정보로 매핑 및 해석합니다.
        ir_l_state = payload[0] != 0
        ir_c_state = payload[1] != 0
        ir_r_state = payload[2] != 0
        red = payload[3]
        green = payload[4]
        blue = payload[5]
        

        # [설명] 해석된 데이터를 바탕으로 각각 ROS 2 토픽에 맞춰 퍼블리시를 수행합니다.
        ir_l_msg = Bool()
        ir_l_msg.data = ir_l_state
        self.ir_l_state_pub.publish(ir_l_msg)
        
        ir_c_msg = Bool()
        ir_c_msg.data = ir_c_state
        self.ir_c_state_pub.publish(ir_c_msg)
        
        ir_r_msg = Bool()
        ir_r_msg.data = ir_r_state
        self.ir_r_state_pub.publish(ir_r_msg)
        
        # [설명] 아두이노에서 수신한 0 ~ 255 정수형 RGB 값을 ROS 2 표준에 맞추어 
        # 255.0으로 나눈 뒤 0.0 ~ 1.0의 float 값 범위로 변환하여 최종 퍼블리시합니다.
        rgb_msg = ColorRGBA()
        rgb_msg.r = float(red) / 255.0
        rgb_msg.g = float(green) / 255.0
        rgb_msg.b = float(blue) / 255.0
        rgb_msg.a = 1.0
        self.rgb_state_pub.publish(rgb_msg)
        
        seq_msg = UInt8()
        seq_msg.data = sequence & 0xFF
        self.rx_sequence_pub.publish(seq_msg)
        
        



def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSensorBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port is not None and node.serial_port.is_open:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()