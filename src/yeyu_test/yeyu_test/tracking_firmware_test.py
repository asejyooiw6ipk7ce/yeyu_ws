#!/usr/bin/env python3
# tracking_firmware_test.py
# 트래킹(IR 센서) 펌웨어 단독 테스트용 스크립트
# 브릿지 노드 없이, PC(윈도우)에서 아두이노가 100ms마다 자동으로 보내는
# PKT_SENSOR_STATE 패킷을 직접 받아서 IR L/C/R 상태를 실시간으로 출력함
#
# 사용법:
#   python tracking_firmware_test.py <포트>
#   예) python tracking_firmware_test.py COM4
#
# 필요 라이브러리: pyserial (pip install pyserial)

import sys
import time
import serial

# ---- 펌웨어와 반드시 동일하게 맞춰야 하는 프로토콜 상수 ----
START_BYTE = 0xAA
END_BYTE_1 = 0xAA
END_BYTE_2 = 0xEE

PKT_SENSOR_STATE = 0x31
PKT_PING = 0x7F

BAUDRATE = 115200


def calc_checksum(packet_id, length, sequence, payload):
    total = packet_id + length + sequence + sum(payload)
    return total & 0xFF


def build_ping_packet(tx_sequence):
    payload = []
    length = 0
    sequence = tx_sequence & 0xFF
    checksum = calc_checksum(PKT_PING, length, sequence, payload)

    packet = bytearray()
    packet.append(START_BYTE)
    packet.append(START_BYTE)
    packet.append(START_BYTE)
    packet.append(PKT_PING)
    packet.append(length)
    packet.append(sequence)
    packet.append(checksum)
    packet.append(END_BYTE_1)
    packet.append(END_BYTE_2)
    return bytes(packet)


class PacketParser:
    """펌웨어의 parseByte()와 동일한 상태기계를 파이썬으로 재현"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = "WAIT_START_1"
        self.packet_id = 0
        self.length = 0
        self.sequence = 0
        self.payload = []
        self.checksum = 0

    def feed(self, byte_in):
        """바이트 하나 넣고, 완성된 패킷이면 (packet_id, length, sequence, payload)를 반환, 아니면 None"""
        if self.state == "WAIT_START_1":
            if byte_in == START_BYTE:
                self.state = "WAIT_START_2"

        elif self.state == "WAIT_START_2":
            self.state = "WAIT_START_3" if byte_in == START_BYTE else "WAIT_START_1"

        elif self.state == "WAIT_START_3":
            self.state = "READ_PACKET_ID" if byte_in == START_BYTE else "WAIT_START_1"

        elif self.state == "READ_PACKET_ID":
            self.packet_id = byte_in
            self.state = "READ_LENGTH"

        elif self.state == "READ_LENGTH":
            self.length = byte_in
            if self.length > 32:
                self.reset()
                return None
            self.payload = []
            self.state = "READ_SEQUENCE"

        elif self.state == "READ_SEQUENCE":
            self.sequence = byte_in
            self.state = "READ_CHECKSUM" if self.length == 0 else "READ_PAYLOAD"

        elif self.state == "READ_PAYLOAD":
            self.payload.append(byte_in)
            if len(self.payload) >= self.length:
                self.state = "READ_CHECKSUM"

        elif self.state == "READ_CHECKSUM":
            self.checksum = byte_in
            self.state = "READ_END_1"

        elif self.state == "READ_END_1":
            if byte_in == END_BYTE_1:
                self.state = "READ_END_2"
            else:
                self.reset()

        elif self.state == "READ_END_2":
            result = None
            if byte_in == END_BYTE_2:
                calculated = calc_checksum(self.packet_id, self.length, self.sequence, self.payload)
                if calculated == self.checksum:
                    result = (self.packet_id, self.length, self.sequence, list(self.payload))
                else:
                    print(f"[경고] checksum 불일치: rx=0x{self.checksum:02X}, calc=0x{calculated:02X}")
            self.reset()
            return result

        else:
            self.reset()

        return None


def main():
    if len(sys.argv) < 2:
        print("사용법: python tracking_firmware_test.py <포트>  예) python tracking_firmware_test.py COM4")
        sys.exit(1)

    port = sys.argv[1]

    try:
        ser = serial.Serial(port, BAUDRATE, timeout=0.05)
    except serial.SerialException as e:
        print(f"시리얼 포트 열기 실패: {e}")
        sys.exit(1)

    print(f"{port} @ {BAUDRATE} 연결됨. 아두이노 부팅 대기 중...")
    time.sleep(2.0)
    ser.reset_input_buffer()

    parser = PacketParser()
    last_print = ""

    print("\n=== IR 센서 상태 실시간 모니터링 시작 ===")
    print("센서 위에 손이나 물체를 대었다 떼면서 값이 바뀌는지 확인하세요.")
    print("Ctrl+C로 종료\n")

    try:
        while True:
            n = ser.in_waiting
            if n > 0:
                data = ser.read(n)
                for b in data:
                    result = parser.feed(b)
                    if result is None:
                        continue
                    packet_id, length, sequence, payload = result

                    if packet_id == PKT_SENSOR_STATE and length >= 3:
                        ir_l, ir_c, ir_r = payload[0], payload[1], payload[2]
                        line = f"[seq={sequence:3d}] IR_L={ir_l}  IR_C={ir_c}  IR_R={ir_r}"
                        if len(payload) >= 6:
                            r, g, b = payload[3], payload[4], payload[5]
                            line += f"   RGB=({r},{g},{b})"
                        if line != last_print:
                            print(line)
                            last_print = line
            else:
                time.sleep(0.005)

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("\n연결 종료.")


if __name__ == "__main__":
    main()
