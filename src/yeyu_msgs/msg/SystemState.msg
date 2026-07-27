# main_node가 발행 — 전체 시스템 운용 모드 (Table 6 기준)
# state 값: IDLE, STARTUP_CHECK, NAV_WAYPOINT, LINE_TRACING,
#           OBSTACLE_RESPONSE, PARKING, SIGNAL_WAIT, ACCEL_ZONE,
#           RESULT_SUMMARY, RETRY, E_STOP
string state
builtin_interfaces/Time stamp