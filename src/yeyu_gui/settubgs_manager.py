# -*- coding: utf-8 -*-
"""
설정값 저장/불러오기 담당 모듈.
GUI를 껐다 켜도 값이 유지되도록 홈 디렉토리 아래 YAML 파일 하나에 저장한다.

저장 위치: ~/.config/yeyu_gui/settings.yaml
"""
from pathlib import Path
import yaml

CONFIG_DIR = Path.home() / ".config" / "yeyu_gui"
CONFIG_PATH = CONFIG_DIR / "settings.yaml"

# 파일이 아직 없을 때(최초 실행) 쓸 기본값.
# settings_screen.py에 있던 하드코딩 값을 그대로 옮겨온 것.
DEFAULT_SETTINGS = {
    "comm": {
        "ros_domain_id": "40",
        "robot_ip": "192.168.230.100",
        "remote_pc_ip": "192.168.230.10",
        "camera_topic": "/camera/image_raw/Compressed",
        "status_topic": "/driving_status",
        "cmd_vel_topic": "/cmd_vel",
    },
    "params": {
        "line_lost_frames": "3",
        "waypoint_tolerance": "0.03",
        "parking_tolerance": "0.035",
        "signal_confirm_frames": "3",
        "target_speed": "0.15",
        "max_retry": "3",
    },
    "storage": {
        "storage_type": "SQLite",
        "log_path": "/home/yeyu_ws/logs",
        "retention_days": "30",
        "auto_save_debug_image": "사용",
    },
}


def load_settings() -> dict:
    """YAML 파일이 있으면 읽어서 반환, 없으면 기본값 반환."""
    if CONFIG_PATH.exists():
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if data:
            return data
    return DEFAULT_SETTINGS


def save_settings(data: dict) -> None:
    """설정값 dict를 YAML 파일에 저장. 폴더가 없으면 생성."""
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        yaml.dump(data, f, allow_unicode=True, sort_keys=False)


def reset_to_default() -> dict:
    """기본값으로 되돌리고 파일에도 반영."""
    save_settings(DEFAULT_SETTINGS)
    return DEFAULT_SETTINGS
