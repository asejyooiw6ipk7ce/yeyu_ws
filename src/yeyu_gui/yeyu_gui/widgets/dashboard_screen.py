# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton, QSizePolicy
)
from PyQt5.QtCore import Qt
from .. import theme
from .card import Card
from .stage_stepper import StageStepper


class SummaryCard(Card):
    def __init__(self, icon_text, label, value, accent, parent=None):
        super().__init__(parent)
        row = QHBoxLayout()
        row.setSpacing(10)

        icon = QLabel(icon_text)
        icon.setFixedSize(34, 34)
        icon.setAlignment(Qt.AlignCenter)
        icon.setStyleSheet(
            f"background: {accent}22; color: {accent}; border-radius: 8px; font-size: 15px;"
        )
        row.addWidget(icon)

        col = QVBoxLayout()
        col.setSpacing(2)
        lbl = QLabel(label)
        lbl.setStyleSheet(f"font-family: {theme.FONT_UI}; font-size: 11.5px; color: {theme.FAINT};")
        val = QLabel(value)
        val.setStyleSheet(
            f"font-family: {theme.FONT_MONO}; font-size: 14.5px; font-weight: 700; color: {theme.INK};"
        )
        col.addWidget(lbl)
        col.addWidget(val)
        row.addLayout(col)
        row.addStretch()

        self.layout_().addLayout(row)
        self.layout_().setContentsMargins(14, 12, 14, 12)


class SafetyRow(QWidget):
    def __init__(self, label, value, ok=True, parent=None):
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        lbl = QLabel(label)
        lbl.setStyleSheet(f"font-family: {theme.FONT_UI}; font-size: 13px; color: {theme.MUTED};")
        val = QLabel(value)
        color = "#16A34A" if ok else "#DC2626"
        val.setStyleSheet(
            f"font-family: {theme.FONT_MONO}; font-size: 12.5px; font-weight: 700; color: {color};"
        )
        row.addWidget(lbl)
        row.addStretch()
        row.addWidget(val)


class DashboardScreen(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(28, 28, 28, 28)
        outer.setSpacing(20)

        # 구간 스테퍼 카드
        stepper_card = Card()
        stepper_card.add_label("코스 진행 상태 · LED 시퀀스 연동")
        self.stepper = StageStepper()
        stepper_card.layout_().addWidget(self.stepper)
        outer.addWidget(stepper_card)
        self.stepper.set_active(2)  # 예시: 크랭크 코스 진행 중

        # 요약 카드 6개
        grid = QGridLayout()
        grid.setSpacing(16)
        summary = [
            ("📍", "현재 구간", "크랭크 코스", "#38BDF8"),
            ("🗺", "위치 (odom)", "X 1.84 / Y 0.62", theme.INK),
            ("🔋", "배터리", "78% · 11.6V", "#16A34A"),
            ("📶", "네트워크", "연결됨 · 38ms", "#16A34A"),
            ("📷", "카메라", "연결됨 · 28 FPS", theme.INK),
            ("↺", "재시도 횟수", "2 / 3", "#D97706"),
        ]
        for i, (icon, label, value, accent) in enumerate(summary):
            card = SummaryCard(icon, label, value, accent)
            grid.addWidget(card, i // 3, i % 3)
        outer.addLayout(grid)

        # 카메라 + 안전상태
        bottom = QHBoxLayout()
        bottom.setSpacing(16)

        cam_card = Card()
        cam_card.add_label("실시간 카메라")
        cam_view = QLabel("/camera/image_raw · 28 FPS")
        cam_view.setAlignment(Qt.AlignCenter)
        cam_view.setMinimumHeight(220)
        cam_view.setStyleSheet(
            f"""
            background-color: #F1F2F4;
            color: {theme.FAINT};
            font-family: {theme.FONT_MONO};
            font-size: 12px;
            border-radius: 8px;
            """
        )
        cam_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        cam_card.layout_().addWidget(cam_view)
        bottom.addWidget(cam_card, 3)

        safety_card = Card()
        safety_card.add_label("안전 상태")
        safety_card.layout_().addWidget(SafetyRow("Wi-Fi 연결", "정상", ok=True))
        safety_card.layout_().addWidget(SafetyRow("전방 장애물", "감지 없음", ok=True))
        safety_card.layout_().addWidget(SafetyRow("긴급정지", "비활성", ok=True))

        estop_btn = QPushButton("■  비상정지")
        estop_btn.setCursor(Qt.PointingHandCursor)
        estop_btn.setMinimumHeight(38)
        estop_btn.setStyleSheet(
            """
            QPushButton {
                background: #DC2626; color: white; border: none;
                border-radius: 8px; font-weight: 700; font-size: 13px;
            }
            QPushButton:hover { background: #B91C1C; }
            """
        )
        safety_card.layout_().addWidget(estop_btn)
        safety_card.layout_().addStretch()
        bottom.addWidget(safety_card, 2)

        outer.addLayout(bottom)
