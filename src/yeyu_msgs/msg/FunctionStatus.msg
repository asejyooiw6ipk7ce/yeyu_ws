# function_node가 발행 — S자·크랭크 라인트레이싱 상태
# course 값: S_CURVE, CRANK
# status 값: TRACKING, LOST, RECOVERING
string course
string status
int32 lost_frame_count     # SR-F-009 (5프레임 연속 미검출 기준)
int32 retry_count          # SR-F-010/011 (최대 3회)