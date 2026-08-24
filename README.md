# yeyu_ws — 자율주행 사전검증 로봇

## 1. 프로젝트 소개

TurtleBot3 Burger 기반으로 실내 시험 코스에서 웨이포인트 자율주행, 크랭크·S자 코스 라인트레이싱, 신호등·가속표지판 인식, ArUco 마커 기반 주차를 순서대로 수행하는 자율주행 사전검증 로봇입니다. 자율주행 플랫폼의 이동·정지·정밀 제어 성능과 카메라 비전 인식 알고리즘의 판단 정확도를 사전에 검증하는 것이 목적이며, 원격 대시보드로 실시간 모니터링·구간별 재시험·비상정지를 지원합니다.

## 2. 서비스 시나리오

1. 전원 인가 후 초기 위치(wp1)에서 대기, 시작 조건 충족 시 자동 출발
2. 웨이포인트를 Nav2로 순차 이동하며 각 임무 구간 진입
3. 크랭크 코스 → S자 코스 → 신호대기 → 가속구간 → ArUco 주차 순서로 수행
4. 각 구간은 자체 기준으로 통과/실패를 판정하고 다음 구간으로 자동 진행
5. 주행 중 전방 장애물 감지 시 정지 후 자동 재개
6. 운영자는 대시보드에서 실시간 상태 확인, 특정 구간 재시험 요청, 비상정지 가능
7. Wifi 연결이 끊기면 초기 위치로 자동 복귀
8. 전체 코스 종료 시 구간별 결과를 집계해 음성(TTS)으로 안내

## 3. 하드웨어 구성

| 구성요소 | 비고 |
|---|---|
| TurtleBot3 Burger (차동구동) | 이동 플랫폼 |
| Dynamixel 모터 + OpenCR | 구동 제어, 엔코더·IMU·배터리 |
| 2D LiDAR | SLAM/AMCL/Nav2용 |
| RGB 카메라 | 라인·신호·표지판·ArUco 인식 |
| IR 3센서 (좌/중/우) | 크랭크 코스 라인트레이싱 |
| 초음파 거리센서 | 장애물 감지 |
| RGB LED | 구간별 상태 표시, 비상정지 점멸 |
| USB 스피커 | TTS 음성 안내 |
| 온보드 컴퓨터 (Ubuntu/ROS 2) | 전체 노드 실행 |

## 4. 소프트웨어 구성

- **ROS 2 Humble**
- **Cartographer** — SLAM 기반 지도 작성
- **AMCL / Nav2** — 위치추정, 웨이포인트 경로계획·추종
- **driving_waypoint_node** — 임무 전체를 관리하는 단일 통합 노드 (웨이포인트 전환, 구간별 상태머신, 판정, 재시험, 비상정지, LED/TTS)
- **arduino_bridge_node_serial** — IR·초음파 센서 수신, LED 명령 송신 (직렬 패킷 통신)
- **대시보드 (PyQt, yeyu_gui)** — 원격 모니터링, 재시험/비상정지 명령

> ⚠️ 별도의 상태관리 노드/안전관리 노드로 분리되어 있지 않고, `driving_waypoint_node` 하나가 MultiThreadedExecutor(4 threads)로 동시성을 관리합니다.

## 5. 패키지 구조

```
yeyu_ws/
└── src/
    ├── yeyu_bringup/        # TurtleBot3 기본 구동 launch, config
    ├── yeyu_cartographer/   # SLAM 설정
    ├── yeyu_control/        # driving_waypoint_node, arduino_bridge_node_serial
    ├── yeyu_description/    # URDF (TurtleBot3 Burger)
    ├── yeyu_gui/            # 대시보드(PyQt) 애플리케이션
    ├── yeyu_hw_node/        # OpenCR 인터페이스 (C++)
    ├── yeyu_launch/         # PM2 ecosystem, 부팅 스크립트
    ├── yeyu_msgs/           # 커스텀 msg/srv 정의
    ├── yeyu_navigation2/    # Nav2 설정, 지도(map)
    ├── yeyu_teleop/         # 수동 조작 (지도 작성용)
    ├── yeyu_test/           # 정밀도 시험 결과/스크립트
    └── yeyu_waypoint_nav/   # 웨이포인트 설정 파일, 기록 도구
```

## 6. 설치 방법

### 6-1. 시스템 업데이트 및 ROS 2 Humble

```bash
sudo apt update && sudo apt upgrade -y

# ROS 2 Humble이 이미 설치되어 있지 않다면
sudo apt install -y ros-humble-desktop ros-humble-ros-base
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init   # 이미 초기화되어 있으면 생략
rosdep update
```

### 6-2. TurtleBot3 / Nav2 / Cartographer(SLAM)

```bash
sudo apt install -y \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-msgs \
  ros-humble-dynamixel-sdk \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-amcl \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-cv-bridge \
  ros-humble-vision-opencv \
  ros-humble-image-transport \
  ros-humble-compressed-image-transport \
  ros-humble-tf2-ros \
  ros-humble-rclpy \
  ros-humble-rclcpp

echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

### 6-3. LiDAR 드라이버

TurtleBot3 Burger 기본 LDS 센서를 사용하는 경우 `ros-humble-turtlebot3` 설치 시 함께 포함되지만, 별도 2D LiDAR(RPLiDAR 등)를 쓰는 경우 아래처럼 추가 설치가 필요합니다.

```bash
sudo apt install -y ros-humble-rplidar-ros   # RPLiDAR 계열을 사용하는 경우
```

### 6-4. 컴퓨터 비전(OpenCV + ArUco)

```bash
pip3 install --upgrade pip
pip3 install \
  opencv-contrib-python \
  numpy \
  pyyaml
```

> `cv2.aruco` 모듈은 `opencv-contrib-python`에 포함되어 있습니다. `opencv-python`만 설치하면 ArUco 인식(직각주차)이 동작하지 않으니 주의하세요.

### 6-5. 대시보드(PyQt5)

```bash
sudo apt install -y python3-pyqt5 libxcb-xinerama0
pip3 install pyqt5
```

### 6-6. USB 스피커 / TTS 음성 안내

```bash
# 시스템 오디오 스택 (USB 스피커 인식/재생)
sudo apt install -y alsa-utils pulseaudio

# TTS 엔진 (audio_output_node에서 사용하는 엔진에 맞춰 택1)
sudo apt install -y espeak-ng      # 오프라인 TTS
# 또는
pip3 install gTTS pyttsx3          # 온라인/파이썬 TTS 라이브러리 사용 시
```

### 6-7. OpenCR / Dynamixel 펌웨어 개발 환경 (모터 보드 재설정이 필요한 경우)

```bash
sudo apt install -y arduino
# Arduino IDE 보드 매니저 URL에 ROBOTIS OpenCR 보드 정의 추가 후,
# 라이브러리 매니저에서 Dynamixel SDK 설치
```

### 6-8. 자동 실행(PM2)

```bash
sudo apt install -y nodejs npm
sudo npm install -g pm2
```

### 6-9. 시리얼 포트 권한 (arduino_bridge_node_serial 용)

```bash
sudo usermod -aG dialout $USER
# 반드시 재부팅 필요 (재로그인만으로는 그룹 권한이 반영되지 않음)
sudo reboot
```

### 6-10. 프로젝트 클론 및 의존성 설치

```bash
mkdir -p ~/yeyu_ws/src
cd ~/yeyu_ws/src
git clone https://github.com/unitydt-ros2-class2-2026/yeyu_main_project_01.git .
cd ~/yeyu_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 7. 빌드 방법

```bash
cd ~/yeyu_ws
colcon build --symlink-install
source install/setup.bash
```

## 8. 실행 방법

`yeyu_launch/nodes/`의 스크립트를 순서대로 실행합니다(각 스크립트 내부에 지연·의존성 대기 포함):

```bash
01_bringup.sh      # TurtleBot3 기본 구동
02_camera.sh       # 카메라 (odom 준비 후)
03_sensor.sh       # arduino_bridge_node_serial (센서 보드 연결 후)
04_audio.sh        # USB 스피커 설정 + audio_output_node
05_navigation.sh   # Nav2 (odom, scan 준비 후)
06_driving.sh      # driving_waypoint_node (navigate_to_pose 액션 서버 준비 후)
```

대시보드는 운영자 PC에서 별도로 실행합니다:

```bash
ros2 run yeyu_gui dashboard_node   # 패키지/실행 파일명은 실제 setup.py 확인 필요
```

## 9. 자동 실행 방법

`yeyu_launch/ecosystem.config.js`로 PM2가 위 6개 스크립트를 순서·지연시간에 맞춰 자동 실행합니다.

```bash
cd yeyu_ws/src/yeyu_launch
pm2 start ecosystem.config.js
pm2 status        # 실행 상태 확인
pm2 logs          # 로그 확인
```

## 10. 토픽, 서비스, 액션 목록

### 주요 토픽

| 토픽 | 메시지 | 설명 |
|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | 최종 속도 명령 |
| `/odom` | nav_msgs/Odometry | 오도메트리 |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | AMCL 추정 위치 |
| `/camera/image_raw/compressed` | sensor_msgs/CompressedImage | 원본 압축 영상 |
| `sensor_bridge/ir_state` | yeyu_msgs/IRSensor | IR 좌/중/우 상태 |
| `sensor_bridge/obstacle_distance_cm` | std_msgs/Float32 | 초음파 거리 |
| `sensor_bridge/rgb_cmd` | std_msgs/ColorRGBA | 상태 LED 명령 |
| `sensor_bridge/emergency_led_cmd` | std_msgs/Bool | 비상 LED 점멸 |
| `/driving_status` | yeyu_msgs/DrivingStatus | 구간별 판정 결과 |
| `/audio/command` | yeyu_msgs/AudioCommand | TTS 음성 명령 |
| `/parking_debug_image/compressed`, `/s_course_debug_image/compressed` | sensor_msgs/CompressedImage | 디버그 영상 |
| `/battery_state` | sensor_msgs/BatteryState | 배터리 상태 |

### 서비스

| 서비스 | 형식 | 기능 |
|---|---|---|
| `/start_retry` | yeyu_msgs/StartRetry | 특정 구간 재시험 요청 |
| `/emergency_stop` | std_srvs/Trigger | 비상정지 |
| `/controller_server/set_parameters` 등 | rcl_interfaces/SetParameters | 가속구간 진입 시 Nav2 속도·costmap 조정 |

### 액션

| 액션 | 형식 | 기능 |
|---|---|---|
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | 웨이포인트 이동 (ABORTED 시 최대 3회 재시도) |

## 11. 지도와 경유점 설명

- 지도: Cartographer로 사전 작성한 `yeyu_navigation2/map/yeyu_map2.yaml`
- 경유점: `yeyu_waypoint_nav/waypoints/waypoint4.yaml`에 wp1~wp11 좌표(x, y, yaw) 등록
  - wp1: 초기 대기 위치 / wp2: 크랭크 도착점 / wp3: S자 시작점 / … / wp11: 최종 도착점
  - `amcl_waypoint_recorder.py`로 AMCL 기반 좌표를 직접 기록해 웨이포인트 생성 가능

## 12. 시험 결과

작성일: 2026-08-18 / 시험 장소: yeyu 자체 제작 미니 테스트 코스(실내) / 시험 담당: 강유나, 김예은

전체 110회 중 83 PASS / 26 FAIL — **성공률 75.5%**

| 시험 ID | 내용 | 목표(인수 기준) | 반복 | PASS | FAIL | 성공률 | 판정 |
|---|---|---|---|---|---|---|---|
| T-001 | 크랭크 코스 주행 | 오차 1cm 이하 | 10 | 4 | 6 | 40% | 합격 |
| T-002 | S 코스 주행 | 각도 오차 최소화 | 10 | 7 | 3 | 70% | 합격 |
| T-003 | 목표 경유점 도착 | 위치 오차 3cm 이내 | 10 | 8 | 2 | 80% | 합격 |
| T-004 | 좁은 통로 통과 | 충돌 없이 통과 | 5 | 4 | 1 | 80% | 합격 |
| T-005 | 신호등 인식 확인 | 초록 신호 3프레임 연속 인식 후 재출발 | 5 | 3 | 2 | 60% | 합격 |
| T-006 | 가속 확인 | 0.18m/s 이상 유지 | 5 | 4 | 1 | 80% | 합격 |
| T-007 | 장애물 감지 | 정지 성공률 100% | 10 | 10 | 0 | 100% | 합격 |
| T-008 | ArUco 마커 인식 | 인식 성공률 기록 | 10 | 8 | 2 | 80% | 합격 |
| T-009 | 충전 위치 파킹 | 성공률과 오차 기록 | 10 | 7 | 2 | 70% | 합격 |
| T-010 | 재시험 주행 | 정상 재이동 및 재판정 성공률 | 10 | 9 | 1 | 90% | 합격 |
| T-011 | Wi-Fi 연결 해제 | 정의된 안전 동작 수행 | 5 | 4 | 1 | 80% | 합격 |
| T-012 | 노드 비정상 종료 | 자동 재시작 확인 | 5 | 4 | 1 | 80% | 합격 |
| T-013 | 전원 재인가 | 자동 실행 확인 | 5 | 4 | 1 | 80% | 합격 |
| T-014 | 전체 서비스 시나리오 | 성공률 기록 | 10 | 7 | 3 | 70% | 합격 |

**실패 원인 카테고리별 집계**: 라인(IR) 이탈/오검출 7건, 정렬/오차 기준 초과 6건, 타임아웃(제한시간 초과) 4건, 카메라 인식 실패(HSV/ArUco) 3건, AMCL/TF 불안정 3건, 통신(Wi-Fi/Serial) 문제 2건, 소프트웨어 예외/버그 2건

### 주요 실패 사례

- **T-001 크랭크**: 회전(TURNING) 구간에서 IR 라인 재진입 각도가 어긋나 직진 복귀 시 오차 누적 → PID 게인 조정 후 7회차부터 연속 PASS
- **T-002 S자**: 초반 offset 값이 오른쪽으로 치우쳐 도착 오차 40~60cm(기준 30cm) → 파라미터 조정 후 4회차부터 오차 10~22cm로 안정
- **T-003 경유점**: AMCL 초기 위치 수렴 전 도착판정이 발생해 오차 초과 → 이후 안정적으로 1~2cm대 오차로 PASS
- **T-005 신호등**: HSV 임계값이 좁아 조명 변화 시 인식 지연(5초 초과) → 임계값 조정 후 1.4~3.4초 내 재출발
- **T-009 파킹**: 마커 가림 시 재탐색 로직 미비로 정렬 실패 → 저속 재탐색 로직 적용 후 대부분 좌우 3cm/거리 3cm대 오차로 PASS
- **T-010 재시험**: RETRY_ENTRY 진입 조건 판정 로직에서 이전 구간 FAIL 플래그 미초기화로 1회차 거부 → 이후 정상 동작
- **T-012 노드 재시작**: PM2 ecosystem에 restart_delay/max_restarts 미설정으로 1회차 재시작 실패 → 설정 후 4~5초 내 재시작 확인
- **T-014 전체 시나리오**: 초기 3회는 가속구간·주차·크랭크 각각의 개별 이슈로 FAIL, 이후 파라미터 안정화되며 4~10회차 연속 전 구간 PASS(4.5~6.5분 소요)

전체 시험 회차별 상세 기록은 `YEYU_통합_시험결과_기록표.xlsx`의 '기록' 시트를 참고하세요.

## 13. 알려진 문제

- **S자 코스 초반 정렬 불안정**: 라인 오프셋이 한쪽으로 치우치는 현상이 초기 회차에서 반복 발생 (파라미터 조정 후 개선됨)
- **증거 자료(사진/영상) 저장 기능 없음**: 결과는 로그·음성 안내로만 제공
- **보고서 파일(JSON/CSV/PDF) 생성 기능 없음**
- **사용자 인증/접근권한 분리 없음**: 대시보드 접속에 인증 없음, 시험용 로컬 네트워크 전제
- **비상정지 해제(clear_emergency) 서비스 없음**: 노드 재기동 등 수동 조치 필요
- **AMCL 위치추정 실패 시 자동 재초기화 로직 없음**: TF 실패 시 amcl_pose 폴백만 존재
- **카메라 데이터 단절 시 자동 재시작 로직 없음**: 프레임 실패는 스킵만 함
- **PM2 재시작 설정 초기 미비**: restart_delay/max_restarts 미설정으로 1차 노드 재시작 시험 실패 이력 있음 (설정 후 해결)

## 14. 팀원 역할

| 팀원 | 담당 |
|---|---|
| **강유나** | 라인트레이싱(크랭크·S자 코스), ArUco 마커 기반 주차, Wi-Fi 단절 대응, 센서 브릿지 노드, 펌웨어 설계, TTS 음성 안내 |
| **김예은** | driving_waypoint_node 전체 구조 설계, 구간별 재시험, 신호등 인식, 가속구간, 장애물 대응, PM2 배포 통합, GUI(대시보드), 구간별 LED 점등 |
