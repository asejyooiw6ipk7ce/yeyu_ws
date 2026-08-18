from datetime import datetime

from PyQt5.QtCore import Qt, QTimer, pyqtSlot
from PyQt5.QtGui import QColor, QImage, QPixmap, QFont, QPainter
from PyQt5.QtWidgets import (
    QFrame, QGridLayout, QHBoxLayout, QHeaderView, QLabel, QMainWindow,
    QMessageBox, QProgressBar, QPushButton, QSizePolicy, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,

)

import math
import numpy as np

from yeyu_gui.ros_bridge import DashboardRosNode, NO_OBSTACLE_READING


TOTAL_WAYPOINTS = 11

STAGE_ROWS = ['NAV_WAYPOINT', 'TRACING_CRANK', 'TRACING_S', 'SIGNAL_WAIT', 'ACCEL_ZONE', 'PARKING', ]   # [수정]
STAGE_LABELS = {
    'NAV_WAYPOINT': '경로 주행',
    'TRACING_CRANK': '크랭크 코스',   
    'TRACING_S': 'S 코스',           
    'SIGNAL_WAIT': '신호대기',
    'ACCEL_ZONE': '가속구간',
    'PARKING': '직각주차',
    'COMPLETE': '주행 완료',
}

MODE_LABELS = {
    'NAV_WAYPOINT': '경로 주행 중',
    'TRACING_CRANK': '크랭크 코스 진행 중',   
    'TRACING_S': 'S 코스 진행 중',           
    'SIGNAL_WAIT': '신호대기 중',
    'ACCEL_ZONE': '가속구간 통과 중',
    'PARKING': '직각주차 중',
    'NAV_TO_END': '도착점으로 이동 중',
    'COMPLETE': '주행 완료',
}
RESULT_TEXT = {'PASS': '통과', 'FAIL': '실패', 'IN_PROGRESS': '진행 중'}
RESULT_COLORS = {
    'PASS': QColor('#2e7d32'),
    'FAIL': QColor('#c62828'),
    'IN_PROGRESS': QColor('#f2a900'),
    'WAIT': QColor('#b7b2a8'),
}
RETRY_TARGETS = {
    2: ('TRACING_CRANK', '크랭크코스 재시험'),   # [수정]
    3: ('TRACING_S', 'S코스 재시험'),           
    4: ('SIGNAL_WAIT', '신호대기 재시험'),
    5: ('ACCEL_ZONE', '가속구간 재시험'),
    6: ('PARKING', '직각주차 재시험'),
}

DEBUG_PANEL_INFO = {   # [추가] 모드 → (패널 제목, 토픽 표시용 텍스트)
    'PARKING': ('주차 디버그', '/parking_debug_image'),
    'TRACING_S': ('S코스 디버그', '/s_course_debug_image'),
}
DEFAULT_DEBUG_TITLE = '디버그'
DEFAULT_DEBUG_TOPIC = '대기 중'

# ================= 팔레트 =================
BG = '#efeee9'
CARD_BG = '#ffffff'
CARD_BORDER = '#e2e0d8'
INK = '#1c1c1a'
MUTED = '#8b8879'
ACCENT = '#e0533e'          # 상단 로고 포인트 / 강조 레드
ACCENT_ORANGE = '#e0a13b'   # 배터리 바
CAMERA_BG = '#17181a'
CAMERA_HEADER = '#94a0a8'

MONO_FONT = 'Consolas, "JetBrains Mono", "Courier New", monospace'

CARD_STYLE = f"""
QFrame#card {{
    background-color: {CARD_BG};
    border: 1px solid {CARD_BORDER};
    border-radius: 12px;
}}
"""
CHIP_STYLE = f"""
QFrame#chip {{
    background-color: {CARD_BG};
    border: 1px solid {CARD_BORDER};
    border-radius: 12px;
}}
"""
PANEL_STYLE = f"""
QFrame#panel {{
    background-color: {CARD_BG};
    border: 1px solid {CARD_BORDER};
    border-radius: 12px;
}}
"""
CAMERA_STYLE = f"""
QFrame#camera {{
    background-color: {CAMERA_BG};
    border-radius: 12px;
}}
"""
class MiniMapWidget(QLabel):
    def __init__(self):
        super().__init__()
        self.map_pixmap = None
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.setStyleSheet('background-color: #101113; border-radius: 8px;')

    def set_map(self, qimage, resolution, origin_x, origin_y):
        self.map_pixmap = QPixmap.fromImage(qimage)
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.update()

    def set_robot_pose(self, x, y, yaw):
        self.robot_x, self.robot_y, self.robot_yaw = x, y, yaw
        self.update()

    def paintEvent(self, event):
        if self.map_pixmap is None:
            return
        painter = QPainter(self)
        scaled = self.map_pixmap.scaled(self.size(), Qt.KeepAspectRatio)
        painter.drawPixmap(0, 0, scaled)

        scale = scaled.width() / self.map_pixmap.width()
        map_h = self.map_pixmap.height()
        px = (self.robot_x - self.origin_x) / self.resolution * scale
        py = (map_h - (self.robot_y - self.origin_y) / self.resolution) * scale

        painter.setBrush(QColor('#e0533e'))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(int(px) - 6, int(py) - 6, 12, 12)

        arrow_len = 20
        ex = px + arrow_len * math.cos(-self.robot_yaw)
        ey = py + arrow_len * math.sin(-self.robot_yaw)
        painter.setPen(QColor('#e0533e'))
        painter.drawLine(int(px), int(py), int(ex), int(ey))

class MainWindow(QMainWindow):

    def __init__(self, ros_node: DashboardRosNode):
        super().__init__()
        self.ros_node = ros_node
        self.active_trajectory = 1
        self.pending_retry_button = None
        self._last_event_key = None
        self._current_debug_mode = None

        self.setWindowTitle('주행 대시보드')
        self.resize(1500, 1080)
        self.setStyleSheet(f"QMainWindow {{ background-color: {BG}; }}")

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(24, 20, 24, 20)
        root.setSpacing(16)

        root.addLayout(self._build_top_bar())
        root.addLayout(self._build_cards())
        root.addLayout(self._build_camera())
        root.addLayout(self._build_middle_row())
        root.addWidget(self._build_event_log())
        root.addLayout(self._build_bottom_bar())

        self._wire_signals()

        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self._update_clock)
        self.clock_timer.start(1000)
        self._update_clock()

    # ================= 상단바 =================
    def _build_top_bar(self):
        layout = QHBoxLayout()

        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        title = QLabel('주행 대시보드')
        title.setStyleSheet(f'color: {INK}; font-size: 26px; font-weight: 800;')
        subtitle = QLabel('AUTONOMOUS PRE-VALIDATION')
        subtitle.setStyleSheet(
            f'color: {MUTED}; font-size: 11px; font-weight: 600; letter-spacing: 3px;')
        title_row = QHBoxLayout()
        title_row.setSpacing(10)
        title_row.addWidget(title)
        title_row.addWidget(subtitle)
        title_row.addStretch(1)
        title_box.addLayout(title_row)
        layout.addLayout(title_box)

        layout.addStretch(1)

        self.connection_badge = QLabel('●  ROS 연결 끊김')
        self.connection_badge.setStyleSheet(
            'background-color: #fbe4e1; color: #c0392b; font-weight: 700; '
            'font-size: 12px; border-radius: 14px; padding: 6px 14px;')
        layout.addWidget(self.connection_badge)

        layout.addSpacing(18)
        self.clock_label = QLabel('--:--:--')
        self.clock_label.setStyleSheet(
            f'color: {INK}; font-size: 20px; font-weight: 600; font-family: {MONO_FONT};')
        layout.addWidget(self.clock_label)
        return layout

    def _update_clock(self):
        self.clock_label.setText(datetime.now().strftime('%H:%M:%S'))

    # ================= 카드 =================
    def _make_card(self, title: str) -> tuple:
        frame = QFrame()
        frame.setObjectName('card')
        frame.setStyleSheet(CARD_STYLE)
        frame.setFixedHeight(96)   # [변경] 카드 높이 고정
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(18, 12, 18, 12)
        layout.setSpacing(4)
        title_label = QLabel(title)
        title_label.setStyleSheet(f'color: {MUTED}; font-size: 12px; font-weight: 600;')
        value_label = QLabel('--')
        value_label.setStyleSheet(f'color: {INK}; font-size: 22px; font-weight: 800;')
        layout.addWidget(title_label)
        layout.addWidget(value_label)
        layout.addStretch(1)
        return frame, value_label

    def _build_cards(self):
        layout = QGridLayout()
        layout.setSpacing(14)

        card, self.mode_value = self._make_card('현재 모드')
        layout.addWidget(card, 0, 0)

        # 현재 위치
        pos_frame = QFrame()
        pos_frame.setObjectName('card')
        pos_frame.setStyleSheet(CARD_STYLE)
        pos_frame.setFixedHeight(96)   # [변경]
        pos_layout = QVBoxLayout(pos_frame)
        pos_layout.setContentsMargins(18, 12, 18, 12)
        pos_layout.setSpacing(4)
        pos_title = QLabel('현재 위치')
        pos_title.setStyleSheet(f'color: {MUTED}; font-size: 12px; font-weight: 600;')
        pos_row = QHBoxLayout()
        pos_row.setSpacing(10)
        self.pos_x_value = QLabel('--')
        self.pos_y_value = QLabel('--')
        for lbl, val in (('X', self.pos_x_value), ('Y', self.pos_y_value)):
            tag = QLabel(lbl)
            tag.setStyleSheet(f'color: {MUTED}; font-size: 12px; font-weight: 700;')
            val.setStyleSheet(f'color: {INK}; font-size: 20px; font-weight: 800; font-family: {MONO_FONT};')
            pos_row.addWidget(tag)
            pos_row.addWidget(val)
        pos_row.addStretch(1)
        pos_layout.addWidget(pos_title)
        pos_layout.addLayout(pos_row)
        pos_layout.addStretch(1)
        layout.addWidget(pos_frame, 0, 1)

        # 배터리
        batt_frame = QFrame()
        batt_frame.setObjectName('card')
        batt_frame.setStyleSheet(CARD_STYLE)
        batt_frame.setFixedHeight(96)   # [변경]
        batt_layout = QVBoxLayout(batt_frame)
        batt_layout.setContentsMargins(18, 12, 18, 12)
        batt_layout.setSpacing(6)
        batt_title = QLabel('배터리')
        batt_title.setStyleSheet(f'color: {MUTED}; font-size: 12px; font-weight: 600;')
        batt_row = QHBoxLayout()
        self.battery_pct_value = QLabel('--')
        self.battery_pct_value.setStyleSheet(f'color: {ACCENT_ORANGE}; font-size: 22px; font-weight: 800;')
        self.battery_v_value = QLabel('')
        self.battery_v_value.setStyleSheet(f'color: {MUTED}; font-size: 13px; font-weight: 600;')
        batt_row.addWidget(self.battery_pct_value)
        batt_row.addWidget(self.battery_v_value)
        batt_row.addStretch(1)
        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(0)
        self.battery_bar.setTextVisible(False)
        self.battery_bar.setFixedHeight(6)
        self.battery_bar.setStyleSheet(f"""
            QProgressBar {{ background-color: #eee9df; border-radius: 3px; }}
            QProgressBar::chunk {{ background-color: {ACCENT_ORANGE}; border-radius: 3px; }}
        """)
        batt_layout.addWidget(batt_title)
        batt_layout.addLayout(batt_row)
        batt_layout.addStretch(1)   # [추가] 진행바를 아래쪽에 딱 붙이기 위한 여백
        batt_layout.addWidget(self.battery_bar)
        layout.addWidget(batt_frame, 0, 2)

        card, self.obstacle_value = self._make_card('장애물 최소거리')
        layout.addWidget(card, 0, 3)

        # card, self.waypoint_value = self._make_card('현재 웨이포인트')
        # layout.addWidget(card, 0, 4)


        layout.setColumnStretch(0, 1)   # [추가] 현재 모드
        layout.setColumnStretch(1, 1)   # [추가] 현재 위치
        layout.setColumnStretch(2, 1)   # [추가] 배터리 — 숫자를 낮추면 더 좁아짐
        layout.setColumnStretch(3, 1)   # [추가] 장애물 최소거리
        # layout.setColumnStretch(4, 1)   # [추가] 현재 웨이포인트``

        return layout
    # ================= 카메라 =================
    def _make_camera_panel(self, title: str, topic: str):
        frame = QFrame()
        frame.setObjectName('camera')
        frame.setStyleSheet(CAMERA_STYLE)
        frame.setFixedSize(480, 360) 
        outer = QVBoxLayout(frame)
        outer.setContentsMargins(18, 14, 18, 14)
        outer.setSpacing(10)

        header = QHBoxLayout()
        title_lbl = QLabel(title)
        title_lbl.setStyleSheet('color: #f2f1ee; font-size: 14px; font-weight: 700;')
        topic_lbl = QLabel(topic)
        topic_lbl.setStyleSheet(f'color: {CAMERA_HEADER}; font-size: 12px; font-family: {MONO_FONT};')
        header.addWidget(title_lbl)
        header.addStretch(1)
        header.addWidget(topic_lbl)
        outer.addLayout(header)

        image_label = QLabel(f'{title} 영상 대기 중...')
        image_label.setAlignment(Qt.AlignCenter)
        image_label.setStyleSheet(f'background-color: #101113; color: {CAMERA_HEADER}; border-radius: 8px;')
        image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)   # 패널 안에서는 꽉 채움
        outer.addWidget(image_label)

        return frame, image_label , title_lbl, topic_lbl
    
    def _build_camera(self):
        layout = QHBoxLayout()
        layout.setSpacing(16)

        panel, self.camera_label, _, _ = self._make_camera_panel('카메라', '/camera/image_raw')
        layout.addWidget(panel)

        panel, self.debug_camera_label, self.debug_title_label, self.debug_topic_label = \
            self._make_camera_panel(DEFAULT_DEBUG_TITLE, DEFAULT_DEBUG_TOPIC)
        layout.addWidget(panel)

        # ---- 미니맵 추가 ----
        self.minimap = MiniMapWidget()
        self.minimap.setFixedSize(360, 360)
        layout.addWidget(self.minimap)
        # --------------------

        layout.addStretch(1) 

        return layout

    # ================= 구간별 진행상황 (칩 형태) =================
    def _build_middle_row(self):
        layout = QVBoxLayout()
        layout.setSpacing(10)

        header = QLabel('구간별 진행상황')
        header.setStyleSheet(f'color: {INK}; font-size: 15px; font-weight: 700;')
        layout.addWidget(header)

        chips_row = QHBoxLayout()
        chips_row.setSpacing(14)

        self.stage_dots = {}
        self.stage_status_labels = {}
        self.stage_reason = {}

        for stage in STAGE_ROWS:
            chip = QFrame()
            chip.setObjectName('chip')
            chip.setStyleSheet(CHIP_STYLE)
            chip_layout = QVBoxLayout(chip)
            chip_layout.setContentsMargins(18, 14, 18, 14)
            chip_layout.setSpacing(6)

            top_row = QHBoxLayout()
            label = QLabel(STAGE_LABELS[stage])
            label.setStyleSheet(f'color: {INK}; font-size: 14px; font-weight: 700;')
            dot = QLabel('●')
            dot.setStyleSheet(f'color: {RESULT_COLORS["WAIT"].name()}; font-size: 13px;')
            top_row.addWidget(label)
            top_row.addStretch(1)
            top_row.addWidget(dot)
            chip_layout.addLayout(top_row)

            status_label = QLabel('대기중')
            status_label.setStyleSheet(f'color: {MUTED}; font-size: 12px; font-weight: 600;')
            chip_layout.addWidget(status_label)

            chips_row.addWidget(chip, 1)
            self.stage_dots[stage] = dot
            self.stage_status_labels[stage] = status_label
            self.stage_reason[stage] = ''

        layout.addLayout(chips_row)
        return layout

    # ================= 이상 이벤트 로그 =================
    def _build_event_log(self):
        panel = QFrame()
        panel.setObjectName('panel')
        panel.setStyleSheet(PANEL_STYLE)
        outer = QVBoxLayout(panel)
        outer.setContentsMargins(18, 14, 18, 16)
        outer.setSpacing(10)

        header = QHBoxLayout()
        title = QLabel('이상 이벤트 로그')
        title.setStyleSheet(f'color: {INK}; font-size: 15px; font-weight: 700;')
        self.event_count_badge = QLabel('0 events')
        self.event_count_badge.setStyleSheet(
            f'color: {MUTED}; font-size: 11px; font-weight: 700; '
            'background-color: #f0eee6; border-radius: 10px; padding: 3px 10px;')
        header.addWidget(title)
        header.addStretch(1)
        header.addWidget(self.event_count_badge)
        outer.addLayout(header)

        self.event_table = QTableWidget(0, 3)
        self.event_table.setHorizontalHeaderLabels(['시간', '구간', '이유'])
        self.event_table.verticalHeader().setVisible(False)
        self.event_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.event_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.event_table.setShowGrid(False)
        self.event_table.setStyleSheet(f"""
            QTableWidget {{ border: none; color: {INK}; font-size: 12px; }}
            QHeaderView::section {{ background-color: transparent; color: {MUTED};
                border: none; font-size: 11px; font-weight: 700; padding: 4px; }}
        """)
        self.event_table.setMinimumHeight(160)
        outer.addWidget(self.event_table)

        self.event_empty_label = QLabel('기록된 이상 이벤트 없음')
        self.event_empty_label.setAlignment(Qt.AlignCenter)
        self.event_empty_label.setStyleSheet(f'color: {MUTED}; font-size: 13px; padding: 20px;')
        outer.addWidget(self.event_empty_label)

        return panel

    # ================= 하단: Trajectory 버튼 + 비상정지 =================
    def _make_trajectory_button(self, top_text: str, bottom_text: str) -> QPushButton:
        btn = QPushButton()
        btn.setMinimumHeight(64)
        layout_text = f'{top_text}\n{bottom_text}'
        btn.setText(layout_text)
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {CARD_BG};
                border: 1px solid {CARD_BORDER};
                border-radius: 10px;
                color: {INK};
                font-size: 12px;
                font-weight: 700;
                text-align: left;
                padding: 10px 16px;
            }}
            QPushButton:hover {{ border-color: #cfcabb; }}
        """)
        return btn

    def _build_bottom_bar(self):
        layout = QHBoxLayout()
        layout.setSpacing(12)

        self.trajectory_buttons = {}

        btn1 = self._make_trajectory_button('Trajectory 1', '정규 코스')
        btn1.setEnabled(False)
        self.trajectory_buttons[1] = btn1
        layout.addWidget(btn1)

        for idx, (target, label) in RETRY_TARGETS.items():
            btn = self._make_trajectory_button(f'Trajectory {idx}', label)
            btn.clicked.connect(
                lambda _checked, i=idx, t=target: self._on_trajectory_clicked(i, t))
            self.trajectory_buttons[idx] = btn
            layout.addWidget(btn)

        self._refresh_trajectory_highlight()

        layout.addStretch(1)

        self.estop_button = QPushButton('■  비상정지')
        self.estop_button.setMinimumSize(170, 64)
        self.estop_button.setStyleSheet(
            f'QPushButton {{ background-color: {ACCENT}; color: white; '
            'font-size: 15px; font-weight: 800; border-radius: 10px; }'
            'QPushButton:hover { background-color: #c8432f; }')
        self.estop_button.clicked.connect(self._on_estop_clicked)
        layout.addWidget(self.estop_button)

        return layout

    def _refresh_trajectory_highlight(self):
        active_style = f"""
            QPushButton {{
                background-color: #fdf3ee;
                border: 2px solid {ACCENT};
                border-radius: 10px;
                color: {INK};
                font-size: 12px;
                font-weight: 700;
                text-align: left;
                padding: 10px 16px;
            }}
        """
        idle_style = f"""
            QPushButton {{
                background-color: {CARD_BG};
                border: 1px solid {CARD_BORDER};
                border-radius: 10px;
                color: {INK};
                font-size: 12px;
                font-weight: 700;
                text-align: left;
                padding: 10px 16px;
            }}
            QPushButton:hover {{ border-color: #cfcabb; }}
        """
        for idx, btn in self.trajectory_buttons.items():
            btn.setStyleSheet(active_style if idx == self.active_trajectory else idle_style)

    # ================= 시그널 연결 =================
    def _wire_signals(self):
        sig = self.ros_node.signals
        sig.ros_connected.connect(self._on_ros_connected)
        sig.driving_status.connect(self._on_driving_status)
        sig.odom.connect(self._on_odom)
        sig.battery.connect(self._on_battery)
        sig.obstacle.connect(self._on_obstacle)
        sig.image.connect(self._on_image)
        sig.parking_debug_image.connect(lambda img: self._on_any_debug_image('PARKING', img))          # [변경]
        sig.s_course_debug_image.connect(lambda img: self._on_any_debug_image('TRACING_S', img))  # [추가]
        sig.estop_result.connect(self._on_estop_result)
        sig.retry_result.connect(self._on_retry_result)
        sig.map_data.connect(self.minimap.set_map)          # 추가
        sig.robot_pose.connect(self.minimap.set_robot_pose)

    def _on_any_debug_image(self, source_mode: str, image: QImage):   # [추가]
        """현재 driving mode와 일치하는 디버그 이미지만 화면에 그림. 
        다른 구간의 디버그 프레임이 뒤늦게 도착해도 무시."""
        if source_mode != self._current_debug_mode:
            return
        pixmap = QPixmap.fromImage(image).scaled(
            self.debug_camera_label.width(), self.debug_camera_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.debug_camera_label.setPixmap(pixmap)

    # ================= 슬롯 =================
    @pyqtSlot(bool)
    def _on_ros_connected(self, connected: bool):
        if connected:
            self.connection_badge.setText('●  ROS 연결됨')
            self.connection_badge.setStyleSheet(
                'background-color: #e5f3e6; color: #2e7d32; font-weight: 700; '
                'font-size: 12px; border-radius: 14px; padding: 6px 14px;')
        else:
            self.connection_badge.setText('●  ROS 연결 끊김')
            self.connection_badge.setStyleSheet(
                'background-color: #fbe4e1; color: #c0392b; font-weight: 700; '
                'font-size: 12px; border-radius: 14px; padding: 6px 14px;')

    @pyqtSlot(dict)
    def _on_driving_status(self, status: dict):
        mode = status['mode']
        result = status['result']
        reason = status['reason']
        wp_index = status['wp_index']

        if mode == 'RETRY_COMPLETE':
            target_label = STAGE_LABELS.get(reason, reason)
            self.mode_value.setText(f'{target_label} 재시험 종료')
            return

        if mode == 'COMPLETE':
            self.mode_value.setText(MODE_LABELS.get('COMPLETE'))
            self._append_event('COMPLETE', reason)
            return

        self.mode_value.setText(MODE_LABELS.get(mode, mode or '--'))
        # self.waypoint_value.setText(f'wp{wp_index + 1} / {TOTAL_WAYPOINTS}')


        # [추가] 모드가 바뀌면 오른쪽 디버그 패널도 그에 맞게 전환
        if mode in DEBUG_PANEL_INFO:
            if mode != self._current_debug_mode:
                title, topic = DEBUG_PANEL_INFO[mode]
                self.debug_title_label.setText(title)
                self.debug_topic_label.setText(topic)
                self.debug_camera_label.setText(f'{title} 영상 대기 중...')
                self.debug_camera_label.setPixmap(QPixmap())   # 이전 프레임 지움
                self._current_debug_mode = mode
        elif self._current_debug_mode is not None:
            # 디버그 대상이 아닌 모드로 넘어가면 패널을 비활성 표시로
            self.debug_title_label.setText(DEFAULT_DEBUG_TITLE)
            self.debug_topic_label.setText(DEFAULT_DEBUG_TOPIC)
            self.debug_camera_label.setText('디버그 영상 없음')
            self.debug_camera_label.setPixmap(QPixmap())
            self._current_debug_mode = None

        if mode in STAGE_ROWS:
            dot = self.stage_dots[mode]
            status_label = self.stage_status_labels[mode]
            status_label.setText(RESULT_TEXT.get(result, result or '대기중'))
            color = RESULT_COLORS.get(result, RESULT_COLORS['WAIT'])
            dot.setStyleSheet(f'color: {color.name()}; font-size: 13px;')
            status_label.setStyleSheet(f'color: {color.name()}; font-size: 12px; font-weight: 700;')
            self.stage_reason[mode] = reason
            self.stage_dots[mode].parent().setToolTip(reason)

        if result == 'FAIL':
            self._append_event(mode, f'{reason}')

        active_target = RETRY_TARGETS.get(self.active_trajectory, (None,))[0]
        if result in ('PASS', 'FAIL') and mode == active_target:
            self.active_trajectory = 1
            self._refresh_trajectory_highlight()

    def _append_event(self, mode: str, reason: str):
        event_key = (mode, reason)
        if event_key == self._last_event_key:
            return
        self._last_event_key = event_key

        row = 0
        self.event_table.insertRow(row)
        self.event_table.setItem(row, 0, QTableWidgetItem(datetime.now().strftime('%H:%M:%S')))
        self.event_table.setItem(row, 1, QTableWidgetItem(STAGE_LABELS.get(mode, mode)))
        self.event_table.setItem(row, 2, QTableWidgetItem(reason))

        count = self.event_table.rowCount()
        self.event_count_badge.setText(f'{count} events')
        self.event_empty_label.setVisible(count == 0)

    @pyqtSlot(float, float)
    def _on_odom(self, x: float, y: float):
        self.pos_x_value.setText(f'{x:.2f}')
        self.pos_y_value.setText(f'{y:.2f}')

    @pyqtSlot(float, float, int)
    def _on_battery(self, voltage: float, percentage: float, _status: int):
        if percentage < 0:
            self.battery_pct_value.setText('--')
            self.battery_v_value.setText(f'{voltage:.1f}V')
            self.battery_bar.setValue(0)
            return
        pct = percentage * 100.0 if percentage <= 1.5 else percentage
        self.battery_pct_value.setText(f'{pct:.0f}%')
        self.battery_v_value.setText(f'{voltage:.1f}V')
        self.battery_bar.setValue(int(max(0, min(100, pct))))

    @pyqtSlot(float)
    def _on_obstacle(self, min_range: float):
        if min_range == NO_OBSTACLE_READING:
            self.obstacle_value.setText('감지 없음')
            self.obstacle_value.setStyleSheet(f'color: {INK}; font-size: 22px; font-weight: 800;')
            return
        self.obstacle_value.setText(f'{min_range:.2f} m')
        color = '#c62828' if min_range < 0.05 else INK
        self.obstacle_value.setStyleSheet(f'color: {color}; font-size: 22px; font-weight: 800;')

    @pyqtSlot(QImage)
    def _on_image(self, image: QImage):
        pixmap = QPixmap.fromImage(image).scaled(
            self.camera_label.width(), self.camera_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.camera_label.setPixmap(pixmap)

    @pyqtSlot(QImage)
    def _on_parking_debug_image(self, image: QImage):
        pixmap = QPixmap.fromImage(image).scaled(
            self.debug_camera_label.width(), self.debug_camera_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.debug_camera_label.setPixmap(pixmap)

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
        print(f'[DEBUG] 버튼 클릭, idx={idx}, target={target!r}', flush=True)
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