# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import QFrame, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt
from .. import theme


class Card(QFrame):
    """공통 카드 컨테이너. 흰 배경 + 옅은 테두리 + 둥근 모서리."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("Card")
        self.setStyleSheet(
            f"""
            #Card {{
                background: {theme.PANEL};
                border: 1px solid {theme.BORDER};
                border-radius: 12px;
            }}
            """
        )
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(20, 18, 20, 18)
        self._layout.setSpacing(10)

    def layout_(self):
        return self._layout

    def add_label(self, text):
        lbl = QLabel(text)
        lbl.setStyleSheet(
            f"""
            color: {theme.FAINT};
            font-family: {theme.FONT_UI};
            font-size: 12px;
            font-weight: 600;
            text-transform: uppercase;
            """
        )
        # QSS text-transform은 지원되지 않으므로 직접 대문자 처리
        lbl.setText(text.upper() if text.isascii() else text)
        self._layout.addWidget(lbl)
        return lbl
