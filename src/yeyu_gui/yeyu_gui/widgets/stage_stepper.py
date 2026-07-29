# -*- coding: utf-8 -*-
"""
구간 스테퍼(StageStepper)
로봇의 실제 LED 점등 순서(대기->S자->크랭크->직각주차->신호->가속->결과)를
그대로 화면 색상으로 반영하는 시그니처 위젯.
"""
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QLabel, QFrame, QSizePolicy
from PyQt5.QtCore import Qt
from .. import theme


class _Dot(QFrame):
    def __init__(self, color, active=False, parent=None):
        super().__init__(parent)
        size = 18 if active else 12
        self.setFixedSize(size, size)
        border = f"3px solid {color}88" if active else "none"
        self.setStyleSheet(
            f"""
            background: {color};
            border-radius: {size // 2}px;
            border: {border};
            """
        )


class StageStepper(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._row = QHBoxLayout(self)
        self._row.setContentsMargins(0, 0, 0, 0)
        self._row.setSpacing(0)
        self._active_index = 0
        self._build(0)

    def set_active(self, index):
        self._active_index = index
        self._build(index)

    def _build(self, active_index):
        # 기존 위젯 제거
        while self._row.count():
            item = self._row.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()

        stages = theme.STAGES
        for i, stage in enumerate(stages):
            is_active = i == active_index
            is_done = i < active_index
            color = stage["color"] if (is_active or is_done) else "#E3E6EB"

            col = QVBoxLayout()
            col.setSpacing(8)
            col.setContentsMargins(0, 0, 0, 0)

            dot_wrap = QHBoxLayout()
            dot_wrap.setContentsMargins(0, 0, 0, 0)
            dot_wrap.addStretch()
            dot_wrap.addWidget(_Dot(color, active=is_active))
            dot_wrap.addStretch()
            col.addLayout(dot_wrap)

            label = QLabel(stage["label"])
            label.setAlignment(Qt.AlignCenter)
            weight = 700 if is_active else 500
            text_color = theme.INK if is_active else (theme.MUTED if is_done else "#B0B7C3")
            label.setStyleSheet(
                f"""
                font-family: {theme.FONT_MONO};
                font-size: 10.5px;
                font-weight: {weight};
                color: {text_color};
                """
            )
            col.addWidget(label)

            col_widget = QWidget()
            col_widget.setLayout(col)
            self._row.addWidget(col_widget, 1)

            if i < len(stages) - 1:
                line_col = QVBoxLayout()
                line_col.setContentsMargins(0, 0, 0, 0)
                line_col.setSpacing(0)

                line = QFrame()
                line.setFixedHeight(2)
                line.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
                line_color = stages[i]["color"] if is_done else "#E3E6EB"
                line.setStyleSheet(f"background: {line_color};")

                dot_h = 18  # 최대 점 크기와 정렬
                line_row = QHBoxLayout()
                line_row.setContentsMargins(0, (dot_h - 2) // 2, 0, (dot_h - 2) // 2)
                line_row.addWidget(line)
                line_col.addLayout(line_row)
                line_col.addSpacing(26)  # 라벨 높이만큼 아래 여백 확보

                line_widget = QWidget()
                line_widget.setLayout(line_col)
                self._row.addWidget(line_widget, 0)
