#!/usr/bin/env python3

import os
import queue
import shutil
import subprocess
import threading
import tempfile

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from gtts import gTTS

from yeyu_msgs.msg import AudioCommand

class AudioOutputNode(Node):
    def __init__(self):
        super().__init__('audio_output_node')

        self.declare_parameter('topic_name','/audio/command') #어떤 토픽을 들을지
        self.declare_parameter('sound_dir','')
        self.declare_parameter('tts_language','ko') #기본언어: 한국어
        self.declare_parameter('tts_slow',False)
        self.declare_parameter('enable_tts',True)
        self.declare_parameter('enable_effect',True)

        self.topic_name = self.get_parameter('topic_name').value
        self.tts_language = self.get_parameter('tts_language').value
        self.tts_slow = bool(self.get_parameter('tts_slow').value)
        self.enable_tts = bool(self.get_parameter('enable_tts').value)
        self.enable_effect = bool(self.get_parameter('enable_effect').value)

        sound_dir_param = self.get_parameter('sound_dir').value

        if sound_dir_param:
            self.sound_dir = sound_dir_param
        else:
            self.sound_dir = os.path.join(get_package_share_directory('robot_audio_output'),
                'sounds'
            )
        
        # 효과음 매핑 ; 짧은 단어 명령 들어오면 어떤 mp3 파일 실행할지
        self.sound_map =  {
            'start': 'start.mp3',
            'waypoint':'waypoint.mp3',
            'goal': 'goal.mp3',
            'warning': 'warning.mp3',
            'error': 'error.mp3',
        }

        self.current_process = None
        self.process_lock = threading.Lock()
        # 소리중복 방지 함수 한번의 하나의 소리 프로세스만 실행되도록 제어


        self.audio_queue = queue.Queue() # 대기열
        self.worker_thread = threading.Thread( 
            target = self.audio_worker,      #audio_worker : 일꾼 쓰레드
            daemon=True
        )
        self.worker_thread.start()

        self.subscription = self.create_subscription(
            AudioCommand,
            self.topic_name,
            self.audio_command_callback,
            10
        )

        self.get_logger().info('Audio output node started')
        self.get_logger().info(f'Subscribe topic:{self.topic_name}')
        self.get_logger().info(f'Sound directory: {self.sound_dir}')
        self.get_logger().info(f'Sound directory: {self.sound_dir}')
        self.get_logger().info('TTS engine: gTTS')
        self.get_logger().info('Audio player: mpg123')
        
		# 주문 받기 ; 다른 노드에서 명령 메세지(AudioCommand)를 보내면 실행되도록
    def audio_command_callback(self, msg: AudioCommand):
        if msg.type == AudioCommand.TYPE_STOP: # 정지 명령(TYPE_STOP)
            self.clear_queue()                 # 대기열에 쌓인 주문 모두 삭제
            self.stop_audio()                  # 내고 있던 소리 종료
            return

        self.audio_queue.put(msg) # 일반명령이면, 대기열(queue)에 넣음(put)

    def clear_queue(self):
        try:
            while True:
                self.audio_queue.get_nowait()
        except queue.Empty:
            pass        

    def audio_worker(self):
        while rclpy.ok():
            try:
                msg = self.audio_queue.get(timeout=0.1)
            except queue.Empty:
                continue
                
            try:
                self.process_audio_command(msg)
            except Exception as e:
                self.get_logger().error('Audio processing error: {e}')
    
    # 주문 처리하기
    def process_audio_command(self, msg:AudioCommand): 
        repeat = msg.repeat
        if repeat <= 0:
            repeat = 1
        
        for _ in range(repeat):                     #repeat 수 만큼 반복
            if msg.type == AudioCommand.TYPE_TTS:
                self.play_tts(msg.text)           # tts 재생
            
            elif msg.type == AudioCommand.TYPE_EFFECT:
                self.play_effect(msg.sound_id)          # 지정 mp3 재생

            elif msg.type == AudioCommand.TYPE_TTS_AND_EFFECT:
                self.play_effect(msg.sound_id)                 # 효과음 내고
                self.play_tts(msg.text)                       # 목소리 재생

            elif msg.type == AudioCommand.TYPE_STOP:
                self.stop_audio()

            else:                     # 그 외의 명령일 경우 모르겠다고 하는거
                self.get_logger.warn('Unkown audio command type: {msg.type}')

		# 글자를 목소리로 바꾸기(by 구글의 gTTS)
    def play_tts(self, text:str): 
        if not self.enable_tts:
            return

        if not text:
            return

        if shutil.which('mpg123') is None:
            self.get_logger.error('mpg123 is not installed')
            return

        self.get_logger().info(f'TTS: {text}')

        mp3_path = None

        try:
            with tempfile.NamedTemporaryFile(
                delete=False,
                suffix='.mp3'
            ) as temp_mp3:
                mp3_path = temp_mp3.name 
                # 전달받은 텍스트를 구글 엔진을 통해 mp3 파일 데이터로 바꾼 뒤, tempfile로 저장
            
            tts = gTTS(
                text=text,
                lang=self.tts_language,
                slow=self.tts_slow
            )

            tts.save(mp3_path)
            self.play_mp3_file(mp3_path)

        except Exception as e:
            self.get_logger().error(f'gTTS error: {e}')

        finally:
		        # 만약 생성된 임시 파일 경로(mp3_path)가 존재하고, 실제로 그 파일이 컴퓨터에 있다면
            if mp3_path and os.path.exists(mp3_path):
                try:
                    os.remove(mp3_path)                # 🧹 임시 MP3 파일을 삭제
                except Exception as e:
                    self.get_logger().warn(f'Failed to remove temp mp3 file: {e}')


    def play_effect(self, sound_id: str):
        if not self.enable_effect:
            return
        
        if not sound_id:
            return
        
        filename = self.sound_map.get(sound_id)

        if filename is None:
            self.get_logger().warn(f'Unkown sound_id: {sound_id}')
            return 
        
        sound_path = os.path.join(self.sound_dir, filename)

        if not os.path.exists(sound_path):
            self.get_logger().warn(f'Sound file not found: {sound_path}')
            return

        self.get_logger().info(f'Play effect: {sound_id}')

        self.play_mp3_file(sound_path)
        
    def play_mp3_file(self,mp3_path: str):
        if shutil.which('mpg123') is None:
            self.get_logger().error('mpg123 is not installed')
            return
        
        with self.process_lock:                # 소리가 겹치거나 먹통이 되는거 방지
            self.current_process = subprocess.Popen(
                ['mpg123','-q',mp3_path],           # mpg123 을 실행해 임시파일 재생
                stdout = subprocess.DEVNULL,        # 컴퓨터 오디오 프로그램
                stderr = subprocess.DEVNULL
        )

        try:
            self.current_process.wait()
        finally:
            with self.process_lock:
                self.current_process = None
    
    def stop_audio(self):
        with self.process_lock:
            if self.current_process is not None:
                try:
                    self.current_process.terminate()
                    self.current_process.wait(timeout=1.0)
                except Exception:
                    try:
                        self.current_process.kill()
                    except Exception:
                        pass

                self.current_process = None
        
        subprocess.run(             # 강제 종료 기능
            ['pkill','-f','mpg123'],     # pkill -f mpg123 : 현재 재생 중인 모든 오디오 종료
            stdout=subprocess.DEVNULL, 
            stderr=subprocess.DEVNULL
        )


def main(args=None):
    rclpy.init(args=args)

    node = AudioOutputNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_audio()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()