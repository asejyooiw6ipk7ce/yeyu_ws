# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QLineEdit,
    QPushButton, QTabWidget
)
from PyQt5.QtCore import Qt
from .. import theme
from .card import Card


def _field(label, value, suffix=""):
    wrap = QVBoxLayout()
    wrap.setSpacing(6)

    lbl = QLabel(label)
    lbl.setStyleSheet(f"font-family: {theme.FONT_UI}; font-size: 12px; color: {theme.FAINT};")
    wrap.addWidget(lbl)

    row = QHBoxLayout()
    edit = QLineEdit(value)
    edit.setStyleSheet(
        f"""
        QLineEdit {{
            border: 1px solid {theme.BORDER}; border-radius: 8px;
            padding: 8px 12px; background: #FAFBFC;
            font-family: {theme.FONT_MONO}; font-size: 13px; color: {theme.INK};
        }}
        """
    )
    row.addWidget(edit)
    if suffix:
        suf = QLabel(suffix)
        suf.setStyleSheet(f"font-family: {theme.FONT_UI}; font-size: 11.5px; color: #B0B7C3;")
        row.addWidget(suf)
    wrap.addLayout(row)

    container = QWidget()
    container.setLayout(wrap)
    return container


class SettingsScreen(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(28, 28, 28, 28)

        card = Card()

        tabs = QTabWidget()
        tabs.setStyleSheet(
            f"""
            QTabWidget::pane {{ border: none; border-top: 1px solid {theme.BORDER}; }}
            QTabBar::tab {{
                padding: 8px 16px; font-family: {theme.FONT_UI}; font-size: 13px;
                color: {theme.FAINT}; background: transparent; border: none;
            }}
            QTabBar::tab:selected {{
                color: {theme.INK}; font-weight: 700;
                border-bottom: 2px solid {theme.ACCENT};
            }}
            """
        )

        tabs.addTab(self._comm_tab(), "통신")
        tabs.addTab(self._params_tab(), "코스 파라미터")
        tabs.addTab(self._storage_tab(), "저장")

        card.layout_().addWidget(tabs)

        btn_row = QHBoxLayout()
        save_btn = QPushButton("저장")
        save_btn.setCursor(Qt.PointingHandCursor)
        save_btn.setStyleSheet(
            f"""
            QPushButton {{
                background: {theme.INK}; color: white; border: none;
                border-radius: 8px; padding: 10px 20px; font-weight: 600; font-size: 13px;
            }}
            QPushButton:hover {{ background: #2A303B; }}
            """
        )
        reset_btn = QPushButton("기본값 복원")
        reset_btn.setCursor(Qt.PointingHandCursor)
        reset_btn.setStyleSheet(
            f"""
            QPushButton {{
                background: white; color: {theme.MUTED}; border: 1px solid {theme.BORDER};
                border-radius: 8px; padding: 10px 20px; font-weight: 600; font-size: 13px;
            }}
            QPushButton:hover {{ background: {theme.BG}; }}
            """
        )
        btn_row.addWidget(save_btn)
        btn_row.addWidget(reset_btn)
        btn_row.addStretch()

        card.layout_().addLayout(btn_row)
        outer.addWidget(card)
        outer.addStretch()

    @staticmethod
    def _grid_tab(fields):
        w = QWidget()
        grid = QGridLayout(w)
        grid.setContentsMargins(0, 16, 0, 0)
        grid.setSpacing(16)
        for i, (label, value, suffix) in enumerate(fields):
            grid.addWidget(_field(label, value, suffix), i // 2, i % 2)
        return w

    def _comm_tab(self):
        return self._grid_tab([
            ("ROS_DOMAIN_ID", "30", ""),
            ("로봇 IP 주소", "192.168.0.30", ""),
            ("원격 PC IP 주소", "192.168.0.10", ""),
            ("카메라 토픽", "/camera/image_raw", ""),
            ("상태 토픽", "/driving_status", ""),
            ("cmd_vel 토픽", "/cmd_vel", ""),
        ])

    def _params_tab(self):
        return self._grid_tab([
            ("라인 이탈 판정 프레임 수 (TBD-01)", "3", "frame"),
            ("경유점 도착 허용 오차 (TBD-02)", "0.03", "m"),
            ("직각주차 좌우 허용 오차 (TBD-03)", "0.035", "m"),
            ("신호 판별 연속 프레임 (TBD-04)", "3", "frame"),
            ("규정 속도 (TBD-05)", "0.15", "m/s"),
            ("코스별 최대 재시도 (TBD-06)", "3", "회"),
        ])

    def _storage_tab(self):
        return self._grid_tab([
            ("저장 방식", "SQLite", ""),
            ("로그 저장 경로", "/home/yeyu/logs", ""),
            ("시험 결과 보관 기간", "30", "일"),
            ("디버그 이미지 자동 저장", "사용", ""),
        ])
