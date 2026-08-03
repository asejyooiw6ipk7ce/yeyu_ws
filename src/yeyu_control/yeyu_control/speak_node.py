import rclpy
from rclpy.node import Node
from robot_audio_interfaces.msg import AudioCommand

class SimpleAudioCommandPublisher(Node):
    def __init__(self):
        super().__init__('simple_audio_command_publisher')

        self.audio_pub = self.create_publisher(AudioCommand, '/audio/command', 10)

        # 노드가 뜨자마자 바로 발행하면 구독자(오디오 출력 노드)가
        # 아직 준비 안 됐을 수 있어서, 짧은 지연 후 한 번만 실행
        self.timer = self.create_timer(1.0, self.publish_once)
        self.sent = False

    def publish_once(self):
        if self.sent:
            return

        msg = AudioCommand()
        msg.type = AudioCommand.TYPE_TTS_AND_EFFECT
        msg.text = '경유점 주행을 시작합니다!'
        msg.sound_id = 'start'
        msg.volume = 1.0
        msg.repeat = 1

        self.audio_pub.publish(msg)
        self.get_logger().info('Audio command published')

        self.sent = True

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAudioCommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()