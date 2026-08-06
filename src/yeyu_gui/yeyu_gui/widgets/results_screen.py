# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTableWidget, QTableWidgetItem,
    QHeaderView, QGridLayout
)
from PyQt5.QtCore import Qt
from .. import theme
from .card import Card

RESULT_ROWS = [
    ("UR-004", "S자 코스 주행", "s_course", "PASS", 0, "라인 이탈 없이 완주"),
    ("UR-004", "크랭크 코스 주행", "crank", "FAIL", 2, "우측 코너 진입 시 라인 미검출 3프레임"),
    ("UR-006", "직각 주차 정렬", "parking", "PASS", 1, "좌우 오차 1.2cm, 거리 오차 0.8cm"),
    ("UR-007", "신호 색상 판별", "signal", "IN_PROGRESS", 0, "진행 중"),
    ("UR-008", "속도표지판 인식", "accel", "PENDING", 0, "대기"),
    ("UR-009", "가속구간 규정속도 준수", "accel", "PENDING", 0, "대기"),
]

STAGE_LABEL = {s["key"]: s for s in theme.STAGES}


class ResultsScreen(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(28, 28, 28, 28)
        outer.setSpacing(20)

        table_card = Card()
        table_card.add_label("시험 결과 요약 · UR-011")

        table = QTableWidget(len(RESULT_ROWS), 6)
        table.setHorizontalHeaderLabels(["ID", "시험 항목", "구간", "결과", "재시도", "비고"])
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setSelectionMode(QTableWidget.NoSelection)
        table.setShowGrid(False)
        table.horizontalHeader().setStretchLastSection(True)
        table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        table.horizontalHeader().setSectionResizeMode(5, QHeaderView.Stretch)
        table.setStyleSheet(
            f"""
            QTableWidget {{
                border: none; font-family: {theme.FONT_UI}; font-size: 13px;
                color: {theme.INK}; gridline-color: {theme.BORDER};
            }}
            QHeaderView::section {{
                background: transparent; border: none; border-bottom: 1px solid {theme.BORDER};
                padding: 8px; color: {theme.FAINT}; font-family: {theme.FONT_UI};
                font-size: 11.5px; font-weight: 600;
            }}
            """
        )

        for row_idx, (rid, name, stage_key, result, retry, note) in enumerate(RESULT_ROWS):
            table.setItem(row_idx, 0, self._mono_item(rid, theme.FAINT))
            table.setItem(row_idx, 1, self._ui_item(name, theme.INK, bold=True))

            stage = STAGE_LABEL.get(stage_key, {"label": stage_key, "color": theme.MUTED})
            stage_item = self._mono_item(f"● {stage['label']}", stage["color"])
            table.setItem(row_idx, 2, stage_item)

            style = theme.RESULT_STYLE[result]
            result_item = QTableWidgetItem(f"  {style['label']}  ")
            result_item.setForeground(_qcolor(style["fg"]))
            result_item.setTextAlignment(Qt.AlignCenter)
            table.setItem(row_idx, 3, result_item)

            table.setItem(row_idx, 4, self._mono_item(str(retry), theme.INK))
            table.setItem(row_idx, 5, self._ui_item(note, theme.MUTED))

        table.resizeRowsToContents()
        table_card.layout_().addWidget(table)
        outer.addWidget(table_card)

        # 요약 카운트
        counts = QGridLayout()
        counts.setSpacing(16)
        summary = [
            ("2", "PASS", "#16A34A"),
            ("1", "FAIL", "#DC2626"),
            ("1", "진행 중", "#D97706"),
            ("2", "대기", theme.FAINT),
        ]
        for i, (num, label, color) in enumerate(summary):
            card = Card()
            num_lbl = QLabel(num)
            num_lbl.setAlignment(Qt.AlignCenter)
            num_lbl.setStyleSheet(
                f"font-family: {theme.FONT_MONO}; font-size: 26px; font-weight: 700; color: {color};"
            )
            text_lbl = QLabel(label)
            text_lbl.setAlignment(Qt.AlignCenter)
            text_lbl.setStyleSheet(f"font-family: {theme.FONT_UI}; font-size: 12px; color: {theme.FAINT};")
            card.layout_().addWidget(num_lbl)
            card.layout_().addWidget(text_lbl)
            counts.addWidget(card, 0, i)
        outer.addLayout(counts)

    @staticmethod
    def _mono_item(text, color):
        item = QTableWidgetItem(text)
        item.setForeground(_qcolor(color))
        return item

    @staticmethod
    def _ui_item(text, color, bold=False):
        item = QTableWidgetItem(text)
        item.setForeground(_qcolor(color))
        return item


def _qcolor(hex_str):
    from PyQt5.QtGui import QColor
    return QColor(hex_str)
