#!/usr/bin/env python3

import os
import queue
import shutil
import subprocess
import threading
import tempfile

import rclpy
from rclpy.node import Node

from gtts import gTTS

from yeyu_msgs.msg import AudioCommand

class AudioOutputNode(Node):
    def __init__(self):
        super().__init__('audio_output_node')

        self.declare_parameter('topic_name','/audio/command') #어떤 토픽을 들을지
        self.declare_parameter('tts_language','ko') #기본언어: 한국어
        self.declare_parameter('tts_slow',False)
        self.declare_parameter('enable_tts',True)

        self.topic_name = self.get_parameter('topic_name').value
        self.tts_language = self.get_parameter('tts_language').value
        self.tts_slow = bool(self.get_parameter('tts_slow').value)
        self.enable_tts = bool(self.get_parameter('enable_tts').value)

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
        self.get_logger().info('TTS engine: gTTS(온라인) + espeak-ng(오프라인 대체)')
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
                self.get_logger().error(f'Audio processing error: {e}')

    # 주문 처리하기
    def process_audio_command(self, msg:AudioCommand):
        repeat = msg.repeat
        if repeat <= 0:
            repeat = 1

        for _ in range(repeat):                     #repeat 수 만큼 반복
            if msg.type == AudioCommand.TYPE_TTS:
                self.play_tts(msg.text)           # tts 재생

            elif msg.type == AudioCommand.TYPE_STOP:
                self.stop_audio()

            else:                     # 그 외의 명령일 경우 모르겠다고 하는거
                self.get_logger().warn(f'Unkown audio command type: {msg.type}')

		# 글자를 목소리로 바꾸기(by 구글의 gTTS)
    def play_tts(self, text:str):
        if not self.enable_tts:
            return

        if not text:
            return

        if shutil.which('mpg123') is None:
            self.get_logger().error('mpg123 is not installed')
            return

        self.get_logger().info(f'TTS: {text}')

        mp3_path = None

        gtts_ok = False
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
            gtts_ok = True

        except Exception as e:
            # ! gTTS는 구글 서버 접속이 필요해서 인터넷이 끊긴 상황(정확히 이 TTS가
            # 알려야 하는 "Wifi 단절" 그 상황)에서는 항상 실패한다. 아래에서 오프라인
            # espeak-ng로 대체 재생하므로, 여기서는 실패를 조용히 기록만 한다.
            self.get_logger().warn(f'gTTS 실패(인터넷 필요) - 오프라인 TTS로 대체: {e}')

        finally:
		        # 만약 생성된 임시 파일 경로(mp3_path)가 존재하고, 실제로 그 파일이 컴퓨터에 있다면
            if mp3_path and os.path.exists(mp3_path):
                try:
                    os.remove(mp3_path)                # 🧹 임시 MP3 파일을 삭제
                except Exception as e:
                    self.get_logger().warn(f'Failed to remove temp mp3 file: {e}')

        if not gtts_ok:
            self.play_tts_offline(text)

    # ================= 오프라인 TTS (espeak-ng, 인터넷 불필요) =================
    def play_tts_offline(self, text: str):
        if shutil.which('espeak-ng') is None:
            self.get_logger().error('espeak-ng가 설치되어 있지 않아 오프라인 TTS 재생 불가 (sudo apt install espeak-ng 필요)')
            return

        if shutil.which('paplay') is None:
            self.get_logger().error('paplay가 설치되어 있지 않아 오프라인 TTS 재생 불가 (sudo apt install pulseaudio-utils 필요)')
            return

        # espeak-ng 자체 오디오 출력은 이 로봇에서는 무음이었음(PulseAudio 기본
        # 싱크가 아닌 엉뚱한 ALSA 카드로 나감). --stdout으로 wav를 뽑아
        # paplay로 재생해야 실제 스피커(PulseAudio 기본 싱크)로 소리가 나온다.
        with self.process_lock:                # 소리가 겹치거나 먹통이 되는거 방지
            espeak_proc = subprocess.Popen(
                ['espeak-ng', '-v', self.tts_language, '--stdout', text],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL
            )
            self.current_process = subprocess.Popen(
                ['paplay'],
                stdin=espeak_proc.stdout,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            espeak_proc.stdout.close()

        try:
            self.current_process.wait()
        finally:
            espeak_proc.wait()
            with self.process_lock:
                self.current_process = None

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
        subprocess.run(             # 오프라인 TTS(espeak-ng | paplay) 잔여 프로세스 종료
            ['pkill','-f','espeak-ng'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        subprocess.run(
            ['pkill','-f','paplay'],
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
