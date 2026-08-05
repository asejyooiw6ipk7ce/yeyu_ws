# -*- coding: utf-8 -*-
"""
디자인 토큰: 색상, 폰트, 코스 구간(Stage) 정의.
Stage 색상은 URS(사용자요구사항명세서)에 정의된 실제 LED 점등 순서와 동일하게 맞춤:
대기(빨강) -> S자(보라) -> 크랭크(하늘) -> 직각주차(노랑) -> 신호(초록) -> 가속(분홍) -> 결과(빨강)
"""

# ---- 색상 팔레트 -----------------------------------------------------------
BG = "#F5F6F8"
PANEL = "#FFFFFF"
INK = "#14181F"
MUTED = "#5B6472"
FAINT = "#8A93A1"
BORDER = "#E3E6EB"
SIDEBAR_BG = "#14181F"
SIDEBAR_ACTIVE = "#242A35"
SIDEBAR_TEXT = "#9AA3B2"
ACCENT = "#38BDF8"

PASS_BG, PASS_FG = "#EAF7EE", "#16A34A"
FAIL_BG, FAIL_FG = "#FCEAEA", "#DC2626"
WARN_BG, WARN_FG = "#FEF3E2", "#D97706"
PENDING_BG, PENDING_FG = "#F1F2F4", "#8A93A1"

# ---- 폰트 -------------------------------------------------------------------
FONT_UI = "Inter, Noto Sans KR, Sans-serif"
FONT_MONO = "JetBrains Mono, D2Coding, Consolas, monospace"

# ---- 코스 구간 (Stage) -------------------------------------------------------
STAGES = [
    {"key": "idle", "label": "대기", "color": "#E33D3D"},
    {"key": "s_course", "label": "S자 코스", "color": "#8B5CF6"},
    {"key": "crank", "label": "크랭크 코스", "color": "#38BDF8"},
    {"key": "parking", "label": "직각주차", "color": "#F5B700"},
    {"key": "signal", "label": "신호판별", "color": "#22C55E"},
    {"key": "accel", "label": "가속구간", "color": "#EC4899"},
    {"key": "result", "label": "결과", "color": "#E33D3D"},
]

RESULT_STYLE = {
    "PASS": {"bg": PASS_BG, "fg": PASS_FG, "label": "PASS"},
    "FAIL": {"bg": FAIL_BG, "fg": FAIL_FG, "label": "FAIL"},
    "IN_PROGRESS": {"bg": WARN_BG, "fg": WARN_FG, "label": "진행 중"},
    "PENDING": {"bg": PENDING_BG, "fg": PENDING_FG, "label": "대기"},
}
