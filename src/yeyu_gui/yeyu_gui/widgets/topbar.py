# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLabel, QPushButton
from PyQt5.QtCore import QTimer, QDateTime, Qt
from .. import theme


class TopBar(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(56)
        self.setStyleSheet(f"background: {theme.PANEL}; border-bottom: 1px solid {theme.BORDER};")

        layout = QHBoxLayout(self)
        layout.setContentsMargins(24, 0, 24, 0)

        self.title_label = QLabel("대시보드")
        self.title_label.setStyleSheet(
            f"font-family: {theme.FONT_UI}; font-weight: 600; font-size: 15px; color: {theme.INK};"
        )
        layout.addWidget(self.title_label)
        layout.addStretch()

        self.ros_dot = QLabel("●")
        self.ros_dot.setStyleSheet("color: #16A34A; font-size: 10px;")
        self.ros_label = QLabel("ROS 연결")
        self.ros_label.setStyleSheet(
            f"font-family: {theme.FONT_MONO}; font-size: 12.5px; color: {theme.MUTED};"
        )
        layout.addWidget(self.ros_dot)
        layout.addWidget(self.ros_label)
        layout.addSpacing(16)

        self.clock_label = QLabel()
        self.clock_label.setStyleSheet(
            f"font-family: {theme.FONT_MONO}; font-size: 12.5px; color: {theme.MUTED};"
        )
        layout.addWidget(self.clock_label)
        layout.addSpacing(16)

        settings_btn = QPushButton("⚙")
        settings_btn.setFixedSize(32, 32)
        settings_btn.setCursor(Qt.PointingHandCursor)
        settings_btn.setStyleSheet(
            f"""
            QPushButton {{
                border: 1px solid {theme.BORDER};
                border-radius: 8px;
                background: {theme.PANEL};
                color: {theme.MUTED};
            }}
            QPushButton:hover {{ background: {theme.BG}; }}
            """
        )
        layout.addWidget(settings_btn)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(1000)
        self._tick()

    def _tick(self):
        now = QDateTime.currentDateTime()
        self.clock_label.setText(now.toString("HH:mm:ss"))

    def set_title(self, text):
        self.title_label.setText(text)

    def set_ros_connected(self, connected: bool):
        color = "#16A34A" if connected else "#DC2626"
        self.ros_dot.setStyleSheet(f"color: {color}; font-size: 10px;")
        self.ros_label.setText("ROS 연결" if connected else "ROS 연결 끊김")
