#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YEYU 자율주행 사전검증 로봇 GUI 실행 파일

실행 방법 (우분투):
    python3 -m pip install PyQt5
    python3 main.py
"""
import sys
from PyQt5.QtWidgets import QApplication
from yeyu_gui.main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")  # 배포판/데스크톱 환경(GTK 등)에 관계없이 일관된 룩 앤 필
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
