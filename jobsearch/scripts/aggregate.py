#!/usr/bin/env python3
"""여러 공고의 evaluation.md를 모아 비교하는 집계 스크립트.

applications/*/evaluation.md 각각의 YAML frontmatter(score, grade, gaps)를
읽어서:
  1. 점수순 정렬 표
  2. 부족 항목(gap) 빈도 순위 — 여러 공고에서 공통으로 부족하다고 나오는
     항목이 진짜 채워야 할 스킬이라는 판단 근거가 된다.
를 터미널에 출력한다. frontmatter를 파싱하는 이유는, evaluation.md 본문은
사람이 읽기 위한 자연어라 자동 집계에 쓰기엔 형식이 불안정하기 때문이다
(/jobsearch:evaluate 명령이 항상 이 frontmatter 형식으로 파일을 쓴다).

사용법:
    python3 jobsearch/scripts/aggregate.py
"""
from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
from typing import Optional

import yaml

FRONTMATTER_DELIM = "---"


def parse_frontmatter(path: Path) -> Optional[dict]:
    text = path.read_text(encoding="utf-8")
    if not text.startswith(FRONTMATTER_DELIM):
        return None
    parts = text.split(FRONTMATTER_DELIM, 2)
    if len(parts) < 3:
        return None
    try:
        data = yaml.safe_load(parts[1])
    except yaml.YAMLError:
        return None
    if not isinstance(data, dict):
        return None
    data["_path"] = path
    return data


def main() -> None:
    parser = argparse.ArgumentParser(description="공고 평가 결과 집계")
    parser.add_argument("--applications-dir", default="jobsearch/applications")
    parser.add_argument("--top-gaps", type=int, default=10)
    args = parser.parse_args()

    apps_dir = Path(args.applications_dir)
    eval_files = sorted(apps_dir.glob("*/evaluation.md"))
    if not eval_files:
        print(f"평가 파일을 찾을 수 없습니다: {apps_dir}/*/evaluation.md")
        print("먼저 /jobsearch:evaluate 로 공고를 평가하세요.")
        return

    records = []
    for f in eval_files:
        data = parse_frontmatter(f)
        if data is None:
            print(f"! frontmatter 파싱 실패, 스킵: {f}")
            continue
        records.append(data)

    if not records:
        print("파싱 가능한 evaluation.md가 없습니다.")
        return

    records.sort(key=lambda d: d.get("score") or 0, reverse=True)
    print(f"\n=== 점수순 정렬 ({len(records)}건) ===")
    for d in records:
        score = d.get("score", "?")
        grade = d.get("grade", "?")
        print(f"  {str(score):>4}  ({grade:>2})  {d.get('company', '?')} / {d.get('title', '?')}")

    gap_counter: Counter[str] = Counter()
    gap_priority: dict[str, str] = {}
    for d in records:
        for gap in d.get("gaps") or []:
            if not isinstance(gap, dict):
                continue
            req = gap.get("requirement")
            if not req:
                continue
            gap_counter[req] += 1
            gap_priority[req] = gap.get("priority", "?")

    print(f"\n=== 부족 항목 빈도 상위 {args.top_gaps} ===")
    if not gap_counter:
        print("  (gaps 데이터 없음)")
    for req, count in gap_counter.most_common(args.top_gaps):
        print(f"  {count}회  [{gap_priority.get(req, '?')}]  {req}")


if __name__ == "__main__":
    main()
