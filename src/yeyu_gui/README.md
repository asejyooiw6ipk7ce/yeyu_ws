# YEYU 자율주행 사전검증 로봇 · GUI

PyQt5로 만든 데스크톱 GUI. 대시보드 / 실시간 모니터링 / 시험 결과 / 설정 4개 화면으로 구성됨.
지금은 목업 데이터(mock)로 채워져 있고, ROS2 노드 연동은 아직 안 붙어있음.

## 우분투에서 실행하기

이미 시스템에 PyQt5가 설치되어 있다면 (ROS2 워크스페이스에서 자주 그렇듯) venv 없이 바로 실행하면 됨:

```bash
unzip yeyu_gui.zip
cd yeyu_gui
python3 -c "import PyQt5"   # 에러 없으면 이미 설치된 것 → 바로 실행
python3 main.py
```

PyQt5가 없다는 에러가 나면:
```bash
pip install PyQt5 --break-system-packages
python3 main.py
```

여러 프로젝트를 오가며 패키지 버전 충돌이 걱정되는 경우에만 venv 사용 (선택):
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 main.py
```

## 폴더 구조

```
main.py                        # 실행 진입점
yeyu_gui/
├── theme.py                   # 색상·폰트·코스 구간(Stage) 정의
├── main_window.py             # 메인 윈도우 (사이드바 + 상단바 + 화면 전환)
├── settings_manager.py        # 설정값 YAML 파일 저장/불러오기 (예정)
└── widgets/
    ├── card.py                # 공통 카드 컨테이너
    ├── stage_stepper.py       # 구간 스테퍼 (LED 시퀀스 연동 시그니처 위젯)
    ├── sidebar.py             # 좌측 네비게이션
    ├── topbar.py              # 상단 상태바 (ROS 연결, 시계)
    ├── dashboard_screen.py    # 대시보드 화면
    ├── monitor_screen.py      # 실시간 모니터링 화면
    ├── results_screen.py      # 시험 결과 화면 (UR-011)
    └── settings_screen.py     # 설정 화면 (통신/코스 파라미터/저장)
```

## 설정값 저장 방식

`SettingsScreen`의 값들(ROS_DOMAIN_ID, IP, 토픽 이름, 코스 파라미터 등)은 별도 데이터베이스 없이
**YAML 파일 하나**로 저장/불러오기 함 (`~/.config/yeyu_gui/settings.yaml` 예정).

- 이런 값들은 "현재 상태 하나"만 유지하면 되는 key-value 데이터라 DB가 필요 없음.
- 반면 `ResultsScreen`처럼 시험 결과가 계속 쌓이는 데이터(행 단위로 누적)는 나중에 SQLite 같은
  경량 DB로 관리하는 게 더 적합함 (설정값과는 성격이 다름).
- 노드 실행 시 `ros2 launch` 파일에서 같은 YAML을 `--params-file`로 그대로 읽게 하면, 별도
  통신 코드 없이도 GUI ↔ 노드가 초기 설정값을 공유할 수 있음.
- 이후 `TBD-01`(라인 이탈 판정 프레임), `TBD-05`(규정 속도)처럼 **주행 중 실시간으로 바꾸고
  싶은 값**이 생기면, 그 항목에 한해서만 ROS2 파라미터 서비스(`set_parameters`)를 추가로 붙여
  실시간 반영이 되게 확장 가능. IP/토픽 이름처럼 노드 실행 전에 정해지는 값은 계속 YAML로 남음.

## 다음 단계 (ROS2 연동)

지금은 모든 값이 하드코딩된 mock 데이터. 실제 로봇과 연결하려면:

1. **rclpy 노드를 별도 QThread로 실행** — PyQt 메인 루프와 ROS2 spin을 같은 스레드에서 돌리면 GUI가 멈추므로, `rclpy.spin()`을 QThread 안에서 돌리고 `pyqtSignal`로 GUI에 값을 전달하는 방식 추천.
2. **어댑터 함수 그대로 재사용** — 이전에 설계한 `DrivingStatus` 파싱 어댑터를 이 GUI의 각 화면에 연결. `getattr(msg, 'field', 기본값)` 패턴으로 아직 없는 필드는 안전하게 처리.
3. 연결 지점:
   - `DashboardScreen`의 요약 카드 → `/driving_status`, `/battery_state`, `/odom` 구독
   - `MonitorScreen`의 디버그 영상 → `/parking_debug_image` (`cv2_to_qimage` 변환 필요)
   - `MonitorScreen`의 버튼 → `/start_mission`, `/cancel_mission` 서비스 호출
   - `ResultsScreen` → 각 구간 완료 시 결과를 누적하는 별도 상태 관리 필요 (지금은 정적 데이터)
4. **`settings_manager.py` 구현** — YAML 로드/저장 함수 작성 후 `SettingsScreen`의 저장 버튼과
   화면 초기화 로직에 연결 (위 "설정값 저장 방식" 참고).

## 참고
- `app.setStyle("Fusion")`을 사용해 우분투 배포판(GTK 테마 등)에 관계없이 항상 같은 룩 앤 필로 보이도록 함.
- 한글 폰트가 시스템에 없으면 기본 폰트로 대체됨. 필요하면 `theme.py`의 `FONT_UI`, `FONT_MONO`를 시스템에 설치된 폰트명으로 바꾸면 됨 (예: Noto Sans KR, D2Coding).