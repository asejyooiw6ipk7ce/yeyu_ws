# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit, QSizePolicy
)
from PyQt5.QtCore import Qt
from .. import theme
from .card import Card


class MetricRow(QWidget):
    def __init__(self, label, value, unit, warn=False, parent=None):
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        lbl = QLabel(label)
        lbl.setStyleSheet(f"font-family: {theme.FONT_UI}; font-size: 13px; color: {theme.MUTED};")
        color = "#D97706" if warn else "#16A34A"
        val = QLabel(f"{value}  ")
        val.setStyleSheet(
            f"font-family: {theme.FONT_MONO}; font-size: 14px; font-weight: 700; color: {color};"
        )
        unit_lbl = QLabel(unit)
        unit_lbl.setStyleSheet(f"font-family: {theme.FONT_UI}; font-size: 11px; color: #B0B7C3;")
        row.addWidget(lbl)
        row.addStretch()
        row.addWidget(val)
        row.addWidget(unit_lbl)


class ActionButton(QPushButton):
    def __init__(self, label, bg, hover=None, parent=None):
        super().__init__(label, parent)
        self.setCursor(Qt.PointingHandCursor)
        self.setMinimumHeight(44)
        hover = hover or bg
        self.setStyleSheet(
            f"""
            QPushButton {{
                background: {bg}; color: white; border: none;
                border-radius: 10px; font-weight: 700; font-size: 13px;
            }}
            QPushButton:hover {{ background: {hover}; }}
            """
        )


class MonitorScreen(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(28, 28, 28, 28)
        outer.setSpacing(20)

        top = QHBoxLayout()
        top.setSpacing(16)

        debug_card = Card()
        debug_card.add_label("디버그 영상 · /parking_debug_image")
        debug_view = QLabel("state=ALIGN_AXIS\nid=4  z=0.24m  x=-0.02m")
        debug_view.setAlignment(Qt.AlignCenter)
        debug_view.setMinimumHeight(220)
        debug_view.setStyleSheet(
            f"""
            background-color: #0F1115; color: #38BDF8;
            font-family: {theme.FONT_MONO}; font-size: 12px;
            border-radius: 8px; padding: 8px;
            """
        )
        debug_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        debug_card.layout_().addWidget(debug_view)
        top.addWidget(debug_card, 3)

        metric_card = Card()
        metric_card.add_label("구간별 실시간 수치")
        metric_card.layout_().addWidget(MetricRow("좌우 오차", "-0.020", "m"))
        metric_card.layout_().addWidget(MetricRow("거리 오차", "0.240", "m"))
        metric_card.layout_().addWidget(MetricRow("각도 오차", "0.041", "rad", warn=True))
        metric_card.layout_().addWidget(MetricRow("현재 속도", "0.070", "m/s"))
        metric_card.layout_().addWidget(MetricRow("재시도 카운트", "2 / 50", "", warn=True))
        metric_card.layout_().addStretch()
        top.addWidget(metric_card, 2)

        outer.addLayout(top)

        log_card = Card()
        log_card.add_label("구간 로그")
        log_view = QTextEdit()
        log_view.setReadOnly(True)
        log_view.setMaximumHeight(140)
        log_view.setStyleSheet(
            f"""
            QTextEdit {{
                border: none; background: transparent;
                font-family: {theme.FONT_MONO}; font-size: 12px; color: {theme.MUTED};
            }}
            """
        )
        log_view.setPlainText(
            "14:32:41  PARKING STATE: SEARCH_MARKER -> ALIGN_AXIS. reason=marker acquired\n"
            "14:32:44  PARKING STATE: ALIGN_AXIS -> FINAL_APPROACH. reason=axis aligned\n"
            "14:32:49  CRANK: 라인 미검출 3프레임, RECOVERY 진입\n"
            "14:32:52  CRANK STATE: RECOVERY -> SEARCH_LINE. reason=recovery finished"
        )
        log_card.layout_().addWidget(log_view)
        outer.addWidget(log_card)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)
        btn_row.addWidget(ActionButton("▶  시험 시작", "#16A34A", "#15803D"))
        btn_row.addWidget(ActionButton("⏸  일시정지", "#D97706", "#B45309"))
        btn_row.addWidget(ActionButton("↺  현재 구간 재시험", "#38BDF8", "#0EA5E9"))
        btn_row.addWidget(ActionButton("■  비상정지", "#DC2626", "#B91C1C"))
        outer.addLayout(btn_row)
