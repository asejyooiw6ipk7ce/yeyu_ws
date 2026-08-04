# driving_node가 발행, main_node가 구독 — 세부 주행 결과 보고
# mode 값: NAV_WAYPOINT, OBSTACLE_RESPONSE, PARKING, SIGNAL_WAIT, ACCEL_ZONE
# result 값: IN_PROGRESS, PASS, FAIL
string mode
string result
string reason
int32 wp_index
float32 error_lateral      # 좌우 오차(m), SR-F-014 (직각주차 3cm 이내)
float32 error_distance     # 거리 오차(m), SR-F-014 (5cm 이내)
int32 retry_count          # SR-F-016 (최대 3회)