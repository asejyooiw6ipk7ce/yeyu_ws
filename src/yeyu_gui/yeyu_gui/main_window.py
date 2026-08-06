from datetime import datetime

from PyQt5.QtCore import Qt, QTimer, pyqtSlot
from PyQt5.QtGui import QColor, QImage, QPixmap
from PyQt5.QtWidgets import (
    QFrame, QGridLayout, QHBoxLayout, QHeaderView, QLabel, QMainWindow,
    QMessageBox, QPushButton, QSizePolicy, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)

from yeyu_gui.ros_bridge import DashboardRosNode, NO_OBSTACLE_READING

TOTAL_WAYPOINTS = 7   # driving_waypoint_node.py의 wp1~wp7 코스 길이

STAGE_ROWS = ['NAV_WAYPOINT', 'SIGNAL_WAIT', 'ACCEL_ZONE', 'PARKING']
STAGE_LABELS = {
    'NAV_WAYPOINT': '경로 주행',
    'SIGNAL_WAIT': '신호대기',
    'ACCEL_ZONE': '가속구간',
    'PARKING': '직각주차',
    'COMPLETE': '주행완료',
}
MODE_LABELS = {
    'NAV_WAYPOINT': '경로 주행 중',
    'SIGNAL_WAIT': '신호대기 중',
    'ACCEL_ZONE': '가속구간 통과 중',
    'PARKING': '직각주차 중',
    'NAV_TO_END': '도착점으로 이동 중',
    'COMPLETE': '주행완료',
}
RESULT_COLORS = {
    'PASS': QColor('#2e7d32'),
    'FAIL': QColor('#c62828'),
    'IN_PROGRESS': QColor('#757575'),
}
RETRY_TARGETS = {
    2: ('SIGNAL_WAIT', '신호대기 재시험'),
    3: ('ACCEL_ZONE', '가속구간 재시험'),
    4: ('PARKING', '직각주차 재시험'),
}

CARD_STYLE = """
QFrame#card {
    background-color: palette(base);
    border: 1px solid #d0d0d0;
    border-radius: 10px;
}
"""
TRAJECTORY_ACTIVE_STYLE = 'QPushButton { border: 2px solid #1565c0; font-weight: bold; }'
TRAJECTORY_IDLE_STYLE = 'QPushButton { border: 1px solid #b0b0b0; }'


class MainWindow(QMainWindow):

    def __init__(self, ros_node: DashboardRosNode):
        super().__init__()
        self.ros_node = ros_node
        self.active_trajectory = 1
        self.pending_retry_button = None

        self.setWindowTitle('주행 대시보드')
        self.resize(1100, 780)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        root.addLayout(self._build_top_bar())
        root.addLayout(self._build_cards())
        root.addWidget(self._build_camera())
        root.addLayout(self._build_middle_row(), stretch=1)
        root.addLayout(self._build_bottom_bar())

        self._wire_signals()

        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self._update_clock)
        self.clock_timer.start(1000)
        self._update_clock()

    # ================= 상단바 =================
    def _build_top_bar(self):
        layout = QHBoxLayout()
        title = QLabel('주행 대시보드')
        title.setStyleSheet('font-size: 20px; font-weight: bold;')
        layout.addWidget(title)
        layout.addStretch(1)

        self.connection_dot = QLabel('●')
        self.connection_dot.setStyleSheet('color: #c62828; font-size: 16px;')
        self.connection_text = QLabel('ROS 연결 끊김')
        layout.addWidget(self.connection_dot)
        layout.addWidget(self.connection_text)

        layout.addSpacing(20)
        self.clock_label = QLabel('--:--:--')
        layout.addWidget(self.clock_label)
        return layout

    def _update_clock(self):
        self.clock_label.setText(datetime.now().strftime('%H:%M:%S'))

    # ================= 카드 =================
    def _make_card(self, title: str) -> tuple:
        frame = QFrame()
        frame.setObjectName('card')
        frame.setStyleSheet(CARD_STYLE)
        layout = QVBoxLayout(frame)
        title_label = QLabel(title)
        title_label.setStyleSheet('color: #666; font-size: 12px;')
        value_label = QLabel('--')
        value_label.setStyleSheet('font-size: 18px; font-weight: bold;')
        layout.addWidget(title_label)
        layout.addWidget(value_label)
        return frame, value_label

    def _build_cards(self):
        layout = QGridLayout()
        card, self.mode_value = self._make_card('현재 모드')
        layout.addWidget(card, 0, 0)
        card, self.position_value = self._make_card('현재 위치')
        layout.addWidget(card, 0, 1)
        card, self.battery_value = self._make_card('배터리')
        layout.addWidget(card, 0, 2)
        card, self.obstacle_value = self._make_card('장애물 최소거리')
        layout.addWidget(card, 0, 3)
        # card, self.waypoint_value = self._make_card('현재 웨이포인트')
        # layout.addWidget(card, 0, 4)
        return layout

    # ================= 카메라 =================
    def _build_camera(self):
        self.camera_label = QLabel('카메라 영상 대기 중...')
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setFixedHeight(300)
        self.camera_label.setStyleSheet(
            'background-color: #202020; color: #aaa; border-radius: 8px;')
        self.camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        return self.camera_label

    # ================= 구간 진행상황 + 이상 이벤트 =================
    def _build_middle_row(self):
        layout = QHBoxLayout()

        stage_box = QVBoxLayout()
        stage_box.addWidget(QLabel('구간별 진행상황'))
        self.stage_table = QTableWidget(len(STAGE_ROWS), 3)
        self.stage_table.setHorizontalHeaderLabels(['구간', '상태', '이유'])
        self.stage_table.verticalHeader().setVisible(False)
        self.stage_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.stage_table.setEditTriggers(QTableWidget.NoEditTriggers)
        for row, stage in enumerate(STAGE_ROWS):
            self.stage_table.setItem(row, 0, QTableWidgetItem(STAGE_LABELS[stage]))
            self.stage_table.setItem(row, 1, QTableWidgetItem('대기중'))
            self.stage_table.setItem(row, 2, QTableWidgetItem(''))
        stage_box.addWidget(self.stage_table)
        layout.addLayout(stage_box, stretch=1)

        event_box = QVBoxLayout()
        event_box.addWidget(QLabel('이상 이벤트 로그'))
        self.event_table = QTableWidget(0, 3)
        self.event_table.setHorizontalHeaderLabels(['시간', '구간', '이유'])
        self.event_table.verticalHeader().setVisible(False)
        self.event_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.event_table.setEditTriggers(QTableWidget.NoEditTriggers)
        event_box.addWidget(self.event_table)
        layout.addLayout(event_box, stretch=1)

        return layout

    # ================= 하단: Trajectory 버튼 + 비상정지 =================
    def _build_bottom_bar(self):
        layout = QHBoxLayout()

        self.trajectory_buttons = {}

        btn1 = QPushButton('Trajectory 1\n정규 코스')
        btn1.setEnabled(False)   # 표시 전용 — 정규 코스는 자동 진행되며 재시작 대상이 아님
        btn1.setMinimumHeight(60)
        self.trajectory_buttons[1] = btn1
        layout.addWidget(btn1)

        for idx, (target, label) in RETRY_TARGETS.items():
            btn = QPushButton(f'Trajectory {idx}\n{label}')
            btn.setMinimumHeight(60)
            btn.clicked.connect(
                lambda _checked, i=idx, t=target: self._on_trajectory_clicked(i, t))
            self.trajectory_buttons[idx] = btn
            layout.addWidget(btn)

        self._refresh_trajectory_highlight()

        layout.addStretch(1)

        self.estop_button = QPushButton('■ 비상정지')
        self.estop_button.setMinimumSize(160, 60)
        self.estop_button.setStyleSheet(
            'QPushButton { background-color: #c62828; color: white; '
            'font-size: 16px; font-weight: bold; border-radius: 8px; }'
            'QPushButton:hover { background-color: #b71c1c; }')
        self.estop_button.clicked.connect(self._on_estop_clicked)
        layout.addWidget(self.estop_button)

        return layout

    def _refresh_trajectory_highlight(self):
        for idx, btn in self.trajectory_buttons.items():
            style = (TRAJECTORY_ACTIVE_STYLE if idx == self.active_trajectory
                     else TRAJECTORY_IDLE_STYLE)
            btn.setStyleSheet(btn.styleSheet() and '' or '')  # reset any inline style first
            btn.setStyleSheet(style)

    # ================= 시그널 연결 =================
    def _wire_signals(self):
        sig = self.ros_node.signals
        sig.ros_connected.connect(self._on_ros_connected)
        sig.driving_status.connect(self._on_driving_status)
        sig.odom.connect(self._on_odom)
        sig.battery.connect(self._on_battery)
        sig.obstacle.connect(self._on_obstacle)
        sig.image.connect(self._on_image)
        sig.estop_result.connect(self._on_estop_result)
        sig.retry_result.connect(self._on_retry_result)

    # ================= 슬롯 =================
    @pyqtSlot(bool)
    def _on_ros_connected(self, connected: bool):
        if connected:
            self.connection_dot.setStyleSheet('color: #2e7d32; font-size: 16px;')
            self.connection_text.setText('ROS 연결됨')
        else:
            self.connection_dot.setStyleSheet('color: #c62828; font-size: 16px;')
            self.connection_text.setText('ROS 연결 끊김')

    @pyqtSlot(dict)
    def _on_driving_status(self, status: dict):
        mode = status['mode']
        result = status['result']
        reason = status['reason']

        if mode == 'RETRY_COMPLETE':
            # reason에 재시험 대상 구간명이 실려 온다 (예: 'PARKING' -> '직각주차 재시험 종료')
            target_label = STAGE_LABELS.get(reason, reason)
            self.mode_value.setText(f'{target_label} 재시험 종료')
            return

        if mode == 'COMPLETE':
            # 구간별 성공/실패와 무관하게, 도착 자체를 상단 카드와 이벤트 로그에 남김
            self.mode_value.setText(MODE_LABELS.get('COMPLETE'))
            self._append_event('COMPLETE', reason)
            return

        self.mode_value.setText(MODE_LABELS.get(mode, mode or '--'))

        if mode in STAGE_ROWS:
            row = STAGE_ROWS.index(mode)
            result_text = {'PASS': '통과', 'FAIL': '실패', 'IN_PROGRESS': '진행 중'}.get(
                result, result or '대기')
            self.stage_table.setItem(row, 1, QTableWidgetItem(result_text))
            self.stage_table.setItem(row, 2, QTableWidgetItem(reason))
            color = RESULT_COLORS.get(result)
            if color is not None:
                self.stage_table.item(row, 1).setForeground(color)

        if result == 'FAIL':
            self._append_event(mode, f'{reason} → 재시험을 권장합니다.')

        active_target = RETRY_TARGETS.get(self.active_trajectory, (None,))[0]
        if result in ('PASS', 'FAIL') and mode == active_target:
            self.active_trajectory = 1
            self._refresh_trajectory_highlight()

    def _append_event(self, mode: str, reason: str):
        row = 0
        self.event_table.insertRow(row)
        self.event_table.setItem(row, 0, QTableWidgetItem(datetime.now().strftime('%H:%M:%S')))
        self.event_table.setItem(row, 1, QTableWidgetItem(STAGE_LABELS.get(mode, mode)))
        self.event_table.setItem(row, 2, QTableWidgetItem(reason))

    @pyqtSlot(float, float)
    def _on_odom(self, x: float, y: float):
        self.position_value.setText(f'X: {x:.2f} / Y: {y:.2f}')

    @pyqtSlot(float, float, int)
    def _on_battery(self, voltage: float, percentage: float, _status: int):
        if percentage < 0:
            self.battery_value.setText(f'{voltage:.1f}V')
            return
        pct = percentage * 100.0 if percentage <= 1.5 else percentage
        self.battery_value.setText(f'{pct:.0f}% / {voltage:.1f}V')

    @pyqtSlot(float)
    def _on_obstacle(self, min_range: float):
        if min_range == NO_OBSTACLE_READING:
            self.obstacle_value.setText('감지 없음')
            self.obstacle_value.setStyleSheet('font-size: 18px; font-weight: bold;')
            return
        self.obstacle_value.setText(f'{min_range:.2f} m')
        color = '#c62828' if min_range < 0.3 else '#212121'
        self.obstacle_value.setStyleSheet(f'font-size: 18px; font-weight: bold; color: {color};')

    @pyqtSlot(QImage)
    def _on_image(self, image: QImage):
        pixmap = QPixmap.fromImage(image).scaled(
            self.camera_label.width(), self.camera_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.camera_label.setPixmap(pixmap)

    # ================= 버튼 동작 =================
    def _on_estop_clicked(self):
        confirm = QMessageBox.question(
            self, '비상정지 확인', '비상정지를 실행하시겠습니까?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if confirm != QMessageBox.Yes:
            return
        self.estop_button.setEnabled(False)
        self.ros_node.call_emergency_stop()

    @pyqtSlot(bool, str)
    def _on_estop_result(self, success: bool, message: str):
        self.estop_button.setEnabled(True)
        if not success:
            QMessageBox.warning(self, '비상정지 실패', message)

    def _on_trajectory_clicked(self, idx: int, target: str):
        confirm = QMessageBox.question(
            self, '재시험 시작 확인', f'{RETRY_TARGETS[idx][1]}을(를) 시작하시겠습니까?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if confirm != QMessageBox.Yes:
            return
        self.pending_retry_button = idx
        self.trajectory_buttons[idx].setEnabled(False)
        self.ros_node.call_start_retry(target)

    @pyqtSlot(str, bool, str)
    def _on_retry_result(self, target: str, accepted: bool, message: str):
        idx = self.pending_retry_button
        self.pending_retry_button = None
        if idx is not None:
            self.trajectory_buttons[idx].setEnabled(True)
        if not accepted:
            QMessageBox.warning(self, '재시험 시작 실패', message)
            return
        if idx is not None:
            self.active_trajectory = idx
            self._refresh_trajectory_highlight()
