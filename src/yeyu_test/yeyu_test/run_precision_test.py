#!/usr/bin/env python3
"""
run_precision_test.py
------------------------------------------------------------
로봇 이동 정밀도 반복 측정 시험 러너

절차 (요구사항 1~10 대응):
  1. 로봇에 이동 명령 전달 (precision_motion.PrecisionMover)
  2. 줄자·각도기로 실제 이동/회전량을 사람이 직접 측정하여 입력
  3. 직진 1m x10, 90도 회전 x10, 180도 회전 x10, 왕복 이동 x10 지원
  4. trial마다 CSV에 append (평균/최대/최소/표준편차는 종료 시 자동 출력)
  5. wheel_radius / wheel_separation 변경 전후 비교를 위해
     CSV 파일명에 타임스탬프를 붙여 매 실행마다 별도 저장

실행 전제: ROS2 Humble 환경에서 TurtleBot3 bringup(odom, cmd_vel)이 떠 있어야 함
  $ ros2 launch turtlebot3_bringup robot.launch.py   (로봇 측)
  $ python3 run_precision_test.py                     (같은 네트워크의 PC 또는 라즈베리파이)
------------------------------------------------------------
"""
import csv
import os
import statistics
from datetime import datetime

import rclpy
from precision_motion import PrecisionMover

RESULT_DIR = os.path.expanduser('~/precision_test_results')
os.makedirs(RESULT_DIR, exist_ok=True)


def ask_float(prompt: str) -> float:
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("숫자로 입력해주세요. 예: 0.99")


def _write_csv(path, rows, fieldnames):
    write_header = not os.path.exists(path)
    with open(path, 'a', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if write_header:
            writer.writeheader()
        writer.writerows(rows)
    print(f"\n결과 저장됨: {path}")


def _print_stats(errors, unit):
    print("\n--- 통계 요약 ---")
    print(f"trial 수        : {len(errors)}")
    print(f"평균 오차       : {statistics.mean(errors):+.4f} {unit}")
    print(f"최대(절대값) 오차: {max(errors, key=abs):+.4f} {unit}")
    print(f"최소(절대값) 오차: {min(errors, key=abs):+.4f} {unit}")
    if len(errors) > 1:
        print(f"표준편차(반복도): {statistics.pstdev(errors):.4f} {unit}")
    print("-----------------\n")


# ---------------- 시험 항목별 함수 ----------------

def run_straight_test(mover, distance_m, trials, csv_path):
    rows = []
    for i in range(1, trials + 1):
        input(f"\n[직진 시험 {i}/{trials}] 로봇을 시작선에 정렬 후 Enter...")
        odom_val = mover.move_straight(distance_m)
        print(f"  (참고) 오도메트리 기준 이동량: {odom_val:.4f} m")
        measured = ask_float("  줄자로 측정한 실제 이동 거리(m): ")
        note = input("  메모(바닥재질/배터리 등, 생략가능): ")
        error = measured - distance_m
        rows.append({
            'trial': i, 'commanded_m': distance_m, 'measured_m': measured,
            'error_m': round(error, 4), 'note': note,
            'timestamp': datetime.now().isoformat(timespec='seconds'),
        })
        print(f"  -> 오차: {error:+.4f} m")
    _write_csv(csv_path, rows, ['trial', 'commanded_m', 'measured_m', 'error_m', 'note', 'timestamp'])
    _print_stats([r['error_m'] for r in rows], unit='m')


def run_rotate_test(mover, angle_deg, trials, csv_path):
    rows = []
    for i in range(1, trials + 1):
        input(f"\n[회전 시험 {i}/{trials}] 로봇 정면 방향 기준선 정렬 후 Enter...")
        odom_val = mover.rotate(angle_deg)
        print(f"  (참고) 오도메트리 기준 회전량: {odom_val:.2f} deg")
        measured = ask_float("  각도기로 측정한 실제 회전 각도(deg): ")
        note = input("  메모(생략가능): ")
        error = measured - angle_deg
        rows.append({
            'trial': i, 'commanded_deg': angle_deg, 'measured_deg': measured,
            'error_deg': round(error, 3), 'note': note,
            'timestamp': datetime.now().isoformat(timespec='seconds'),
        })
        print(f"  -> 오차: {error:+.3f} deg")
    _write_csv(csv_path, rows, ['trial', 'commanded_deg', 'measured_deg', 'error_deg', 'note', 'timestamp'])
    _print_stats([r['error_deg'] for r in rows], unit='deg')


def run_roundtrip_test(mover, distance_m, trials, csv_path):
    rows = []
    for i in range(1, trials + 1):
        input(f"\n[왕복 시험 {i}/{trials}] 출발점(A)에 정렬 후 Enter...")
        mover.move_straight(distance_m)
        input("  도착점(B) 도달. 위치 표시 후 Enter -> 복귀 시작...")
        mover.move_straight(-distance_m)
        print("  복귀 완료.")
        measured = ask_float("  출발점(A) 대비 최종 위치 오차(m, 초과=+/미달=-): ")
        note = input("  메모(생략가능): ")
        rows.append({
            'trial': i, 'one_way_m': distance_m, 'final_error_m': measured,
            'note': note, 'timestamp': datetime.now().isoformat(timespec='seconds'),
        })
    _write_csv(csv_path, rows, ['trial', 'one_way_m', 'final_error_m', 'note', 'timestamp'])
    _print_stats([r['final_error_m'] for r in rows], unit='m')


def main():
    rclpy.init()
    mover = PrecisionMover()

    print("=== TurtleBot3 Burger 이동 정밀도 시험 ===")
    print("1) 직진 1m 시험")
    print("2) 90도 회전 시험")
    print("3) 180도 회전 시험")
    print("4) 왕복 이동 시험")
    choice = input("선택: ").strip()
    trials = int(input("반복 횟수(기본 10, 최소 10회 권장): ") or "10")
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')

    try:
        if choice == '1':
            run_straight_test(mover, 1.0, trials, os.path.join(RESULT_DIR, f'straight_{ts}.csv'))
        elif choice == '2':
            run_rotate_test(mover, 90.0, trials, os.path.join(RESULT_DIR, f'rotate90_{ts}.csv'))
        elif choice == '3':
            run_rotate_test(mover, 180.0, trials, os.path.join(RESULT_DIR, f'rotate180_{ts}.csv'))
        elif choice == '4':
            d = ask_float("왕복 편도 거리(m): ")
            run_roundtrip_test(mover, d, trials, os.path.join(RESULT_DIR, f'roundtrip_{ts}.csv'))
        else:
            print("잘못된 선택입니다.")
    finally:
        mover.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()