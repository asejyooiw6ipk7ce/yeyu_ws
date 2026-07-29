# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QStackedWidget
from . import theme
from .widgets.sidebar import Sidebar
from .widgets.topbar import TopBar
from .widgets.dashboard_screen import DashboardScreen
from .widgets.monitor_screen import MonitorScreen
from .widgets.results_screen import ResultsScreen
from .widgets.settings_screen import SettingsScreen

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

    def _on_screen_changed(self, key):
        self.stack.setCurrentWidget(self.screens[key])
        self.topbar.set_title(TITLES[key])
