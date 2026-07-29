# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QLineEdit,
    QPushButton, QTabWidget, QMessageBox
)
from PyQt5.QtCore import Qt
from .. import theme
from .. import settings_manager
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
    return container, edit  # 화면에 보여줄 위젯 반환 , QLineEdit 자체 같이 반환(edit.text()로 사용자가 입력한 값을 꺼내기 위해)


class SettingsScreen(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(28, 28, 28, 28)

        # 현재 저장된 값(없으면 기본값)을 먼저 불러온다.
        self._settings = settings_manager.load_settings()

        # 탭별 QLineEdit 참조를 여기에 모아둔다. {"comm": {"ros_domain_id": QLineEdit, ...}, ...}
        self._edits = {"comm": {}, "params": {}, "storage": {}}

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
        save_btn.clicked.connect(self._on_save)   # 저장 버튼 기능 연결

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

        reset_btn.clicked.connect(self._on_reset) # 기본값 복원 버튼 기능 추가 

        btn_row.addWidget(save_btn)
        btn_row.addWidget(reset_btn)
        btn_row.addStretch()

        # "✓ 저장됨" , "✓ 기본값으로 복원됨" 띄워줌
        self._status_label = QLabel("")
        self._status_label.setStyleSheet(
            f"font-family: {theme.FONT_UI}; font-size: 12px; color: #16A34A;"
        )
        btn_row.addWidget(self._status_label)

        card.layout_().addLayout(btn_row)
        outer.addWidget(card)
        outer.addStretch()

    def _grid_tab(self, tab_key, fields):
        """fields: [(설정_키, 라벨, 단위), ...]"""
        w = QWidget()
        grid = QGridLayout(w)
        grid.setContentsMargins(0, 16, 0, 0)
        grid.setSpacing(16)
        for i, (settings_key, label, suffix) in enumerate(fields):
            current_value = str(self._settings.get(tab_key, {}).get(settings_key, ""))
            container, edit = _field(label, current_value, suffix)
            self._edits[tab_key][settings_key] = edit
            grid.addWidget(container, i // 2, i % 2)
        return w

    def _comm_tab(self):
        return self._grid_tab("comm", [
            ("ros_domain_id", "ROS_DOMAIN_ID", ""),
            ("robot_ip", "로봇 IP 주소", ""),
            ("remote_pc_ip", "원격 PC IP 주소", ""),
            ("camera_topic", "카메라 토픽", ""),
            ("status_topic", "상태 토픽", ""),
            ("cmd_vel_topic", "cmd_vel 토픽", ""),
        ])   # 값은 settings_manager.load_settings()로 불러온 yaml파일에서 채워짐

    def _params_tab(self):
        return self._grid_tab("params", [
            ("line_lost_frames", "라인 이탈 판정 프레임 수 (TBD-01)", "frame"),
            ("waypoint_tolerance", "경유점 도착 허용 오차 (TBD-02)", "m"),
            ("parking_tolerance", "직각주차 좌우 허용 오차 (TBD-03)", "m"),
            ("signal_confirm_frames", "신호 판별 연속 프레임 (TBD-04)", "frame"),
            ("target_speed", "규정 속도 (TBD-05)", "m/s"),
            ("max_retry", "코스별 최대 재시도 (TBD-06)", "회"),
        ])

    def _storage_tab(self):
        return self._grid_tab("storage", [
            ("storage_type", "저장 방식", ""),
            ("log_path", "로그 저장 경로", ""),
            ("retention_days", "시험 결과 보관 기간", "일"),
            ("auto_save_debug_image", "디버그 이미지 자동 저장", ""),
        ])

    def _collect_values(self) -> dict:
        """모든 QLineEdit에서 현재 입력된 값을 읽어 dict로 만든다."""
        return {
            tab_key: {key: edit.text() for key, edit in edits.items()}
            for tab_key, edits in self._edits.items()
        }

    def _on_save(self):
        data = self._collect_values()
        settings_manager.save_settings(data)
        self._settings = data
        self._status_label.setText("✓ 저장됨")

    def _on_reset(self):
        confirm = QMessageBox.question(
            self, "기본값 복원", "모든 설정을 기본값으로 되돌릴까요?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if confirm != QMessageBox.Yes:
            return
        defaults = settings_manager.reset_to_default()
        self._settings = defaults
        for tab_key, edits in self._edits.items():
            for key, edit in edits.items():
                edit.setText(str(defaults.get(tab_key, {}).get(key, "")))
        self._status_label.setText("✓ 기본값으로 복원됨")
