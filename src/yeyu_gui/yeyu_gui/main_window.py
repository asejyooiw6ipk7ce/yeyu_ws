# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QStackedWidget
from . import theme
from .widgets.sidebar import Sidebar
from .widgets.topbar import TopBar
from .widgets.dashboard_screen import DashboardScreen
from .widgets.monitor_screen import MonitorScreen
from .widgets.results_screen import ResultsScreen
from .widgets.settings_screen import SettingsScreen

# rclpy가 없는 환경(예: ROS2 미설치 PC)에서도 GUI 자체는 뜨도록 임포트 실패를 허용.
# 실패하면 ROS 연동 없이 mock 데이터만 보여주는 상태로 동작한다.
try:
    from .ros_worker import RosWorker
    _ROS_AVAILABLE = True
except ImportError:
    RosWorker = None
    _ROS_AVAILABLE = False

TITLES = {
    "dashboard": "대시보드",
    "monitor": "실시간 모니터링",
    "results": "시험 결과",
    "settings": "설정",
}


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("YEYU 자율주행 사전검증 로봇 · GUI")
        self.resize(1280, 820)
        self.setStyleSheet(f"background: {theme.BG};")

        central = QWidget()
        root = QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        self.sidebar = Sidebar()
        root.addWidget(self.sidebar)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(0)

        self.topbar = TopBar()
        right_layout.addWidget(self.topbar)

        self.stack = QStackedWidget()
        self.screens = {
            "dashboard": DashboardScreen(),
            "monitor": MonitorScreen(),
            "results": ResultsScreen(),
            "settings": SettingsScreen(),
        }
        for key in ["dashboard", "monitor", "results", "settings"]:
            self.stack.addWidget(self.screens[key])

        right_layout.addWidget(self.stack)
        root.addWidget(right, 1)

        self.setCentralWidget(central)

        self.sidebar.screen_changed.connect(self._on_screen_changed)
        self._on_screen_changed("dashboard")

        self.ros_worker = None
        self._start_ros_worker()

    def _on_screen_changed(self, key):
        self.stack.setCurrentWidget(self.screens[key])
        self.topbar.set_title(TITLES[key])

    # GUI 켜질 때 ROS2 워커 가동 & 워커가 보내는 신호 받아서 처리
    
    def _start_ros_worker(self):
        if not _ROS_AVAILABLE:
            # rclpy 자체가 안 깔려있는 경우. topbar에 연결 끊김 표시만 하고 넘어감.
            self.topbar.set_ros_connected(False)
            return

        self.ros_worker = RosWorker()
        self.ros_worker.status_received.connect(self._on_status_received)
        self.ros_worker.connection_changed.connect(self.topbar.set_ros_connected)
        self.ros_worker.error_occurred.connect(self._on_ros_error)
        self.ros_worker.start()

    def _on_status_received(self, data: dict):
        # 지금은 대시보드만 반영. 나중에 monitor_screen 등에도 필요하면 여기서 같이 호출.
        self.screens["dashboard"].apply_live_status(data)

    def _on_ros_error(self, message: str):
        # 우선은 콘솔 로그만. 필요하면 topbar나 상태바에 표시하는 방식으로 확장 가능.
        print(f"[ROS] {message}")

    def closeEvent(self, event):
        if self.ros_worker is not None:
            self.ros_worker.stop()
        super().closeEvent(event)
