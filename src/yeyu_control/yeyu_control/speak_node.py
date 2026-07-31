import rclpy
from rclpy.node import Node

from robot_audio_interfaces.msg import AudioCommand

class SimpleAudioCommandPublisher(Node):
    def __init__(self):
        super().__init__('simple_audio_command_publisher')\

        self.audio_sub = self.create_subscriber(AudioCommand,self.audio_callback, '/audio/command',10)
		# /audio/command로 msg 발행할 스피커 만듦 (오디오 출력 노드가 /audio/command를 구독중)
        self.audio_pub= self.create_publisher(AudioCommand,'/audio/command',10)

        self.sent= False

    def audio_callback(self):
        if self.sent:
            return
        
        msg=AudioCommand()
        msg.type =AudioCommand.TYPE_TTS_AND_EFFECT
        msg.text = '경유점 주행을 시작합니다!'
        msg.sound_id ='start'
        msg.volume = 1.0
        msg.repeat = 1

        self.audio_pub.publish(msg) #'start 효과음 + 경유점~~' msg를 /audio/command 토픽 주소로 보

        self.get_logger().info('Audio command published')

        self.sent =True             # 한번 한 뒤에는 그냥 넘어감(--once과 같은 역할) 

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

if __name__ =='__main__':
    main()
