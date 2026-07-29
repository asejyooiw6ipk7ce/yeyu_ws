# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel, QFrame
from PyQt5.QtCore import pyqtSignal, Qt
from .. import theme

NAV_ITEMS = [
    ("dashboard", "대시보드"),
    ("monitor", "실시간 모니터링"),
    ("results", "시험 결과"),
    ("settings", "설정"),
]


class Sidebar(QWidget):
    screen_changed = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(220)
        self.setStyleSheet(f"background: {theme.SIDEBAR_BG};")
        self._buttons = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # 로고 / 프로젝트명
        header = QWidget()
        header_layout = QVBoxLayout(header)
        header_layout.setContentsMargins(20, 24, 20, 20)
        header_layout.setSpacing(4)

        eyebrow = QLabel("YEYU · YURS-ADV-001")
        eyebrow.setStyleSheet(
            f"color: #7C8698; font-family: {theme.FONT_MONO}; font-size: 11px; letter-spacing: 1px;"
        )
        title = QLabel("자율주행\n사전검증 로봇")
        title.setStyleSheet(
            f"color: #FFFFFF; font-family: {theme.FONT_UI}; font-size: 17px; font-weight: 700;"
        )
        header_layout.addWidget(eyebrow)
        header_layout.addWidget(title)
        layout.addWidget(header)

        # 네비게이션 버튼
        nav_container = QWidget()
        nav_layout = QVBoxLayout(nav_container)
        nav_layout.setContentsMargins(12, 8, 12, 8)
        nav_layout.setSpacing(4)

        for key, label in NAV_ITEMS:
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(lambda _, k=key: self._on_click(k))
            self._buttons[key] = btn
            nav_layout.addWidget(btn)

        layout.addWidget(nav_container)
        layout.addStretch()

        # 하단 시스템 상태
        footer = QFrame()
        footer.setStyleSheet(f"border-top: 1px solid {theme.SIDEBAR_ACTIVE};")
        footer_layout = QVBoxLayout(footer)
        footer_layout.setContentsMargins(16, 14, 16, 14)
        status = QLabel("● 시스템 정상 · v0.3.0")
        status.setStyleSheet(
            f"color: {theme.SIDEBAR_TEXT}; font-family: {theme.FONT_MONO}; font-size: 11.5px;"
        )
        footer_layout.addWidget(status)
        layout.addWidget(footer)

        self._apply_button_style()
        self.set_active("dashboard")

    def _apply_button_style(self):
        for btn in self._buttons.values():
            btn.setStyleSheet(
                f"""
                QPushButton {{
                    text-align: left;
                    padding: 10px 12px;
                    border-radius: 8px;
                    border: none;
                    border-left: 3px solid transparent;
                    color: {theme.SIDEBAR_TEXT};
                    background: transparent;
                    font-family: {theme.FONT_UI};
                    font-size: 13.5px;
                    font-weight: 500;
                }}
                QPushButton:checked {{
                    background: {theme.SIDEBAR_ACTIVE};
                    color: #FFFFFF;
                    font-weight: 600;
                    border-left: 3px solid {theme.ACCENT};
                }}
                QPushButton:hover:!checked {{
                    background: #1B212B;
                }}
                """
            )

    def _on_click(self, key):
        self.set_active(key)
        self.screen_changed.emit(key)

    def set_active(self, key):
        for k, btn in self._buttons.items():
            btn.setChecked(k == key)
