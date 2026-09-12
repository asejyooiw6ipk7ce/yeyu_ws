#!/usr/bin/env python3
"""원티드(wanted.co.kr) 채용공고 스크래퍼.

주의: 원티드의 정식 공개 API가 아니라, 웹사이트 프론트엔드가 사용하는
내부 API(www.wanted.co.kr/api/v4/*)를 호출한다. 원티드가 API 구조를
바꾸면 이 스크립트도 깨진다. `search_jobs`가 "예상한 응답 구조를 찾지
못했습니다" 에러를 내면:

  1. 브라우저에서 https://www.wanted.co.kr/search?query=<키워드> 를 연다.
  2. 개발자도구(F12) > Network 탭에서 실제 호출되는 요청 URL/응답 JSON을 확인한다.
  3. 이 파일의 SEARCH_URL / JOB_DETAIL_URL 과 파싱 경로를 그에 맞춰 고친다.

사용법:
    python3 scrape_wanted.py "로봇 시뮬레이션" --limit 20
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import requests

sys.path.insert(0, str(Path(__file__).parent))
from common import find_existing_job, write_job_markdown  # noqa: E402

SEARCH_URL = "https://www.wanted.co.kr/api/v4/search"
JOB_DETAIL_URL = "https://www.wanted.co.kr/api/v4/jobs/{job_id}"
HEADERS = {
    "User-Agent": "Mozilla/5.0 (compatible; jobsearch-bot/1.0)",
    "Accept": "application/json",
}
SOURCE = "wanted"


def search_jobs(keyword: str, limit: int) -> list[dict]:
    params = {"query": keyword, "tab": "position", "country": "kr"}
    resp = requests.get(SEARCH_URL, params=params, headers=HEADERS, timeout=10)
    resp.raise_for_status()
    data = resp.json()
    try:
        items = data["position"]["data"]
    except (KeyError, TypeError) as e:
        raise RuntimeError(
            "예상한 응답 구조(position.data)를 찾지 못했습니다. "
            f"실제 응답 최상위 키: {list(data.keys()) if isinstance(data, dict) else type(data)}. "
            "원티드 API 응답 구조가 바뀐 것 같습니다 — 스크립트 상단 주석을 참고해 수정하세요."
        ) from e
    return items[:limit]


def fetch_job_detail(job_id) -> dict:
    resp = requests.get(JOB_DETAIL_URL.format(job_id=job_id), headers=HEADERS, timeout=10)
    resp.raise_for_status()
    return resp.json()


def job_to_markdown_body(detail: dict) -> str:
    job = detail.get("job", detail) if isinstance(detail, dict) else {}
    fields = [
        ("소개", job.get("intro")),
        ("주요업무", job.get("main_tasks")),
        ("자격요건", job.get("requirements")),
        ("우대사항", job.get("preferred_points")),
        ("혜택 및 복지", job.get("benefits")),
    ]
    parts = [f"## {label}\n{value}" for label, value in fields if value]
    return "\n\n".join(parts) if parts else "(본문을 파싱하지 못했습니다. 원본 URL을 직접 확인하세요.)"


def main() -> None:
    parser = argparse.ArgumentParser(description="원티드 채용공고 스크래퍼")
    parser.add_argument("keyword", help="검색 키워드 (예: '로봇 시뮬레이션')")
    parser.add_argument("--limit", type=int, default=20, help="가져올 공고 개수 (기본 20)")
    parser.add_argument("--jobs-dir", default="jobsearch/jobs", help="공고 저장 폴더")
    parser.add_argument("--delay", type=float, default=0.5, help="요청 간 대기 시간(초)")
    args = parser.parse_args()

    jobs_dir = Path(args.jobs_dir)

    print(f"[wanted] '{args.keyword}' 검색 중...")
    try:
        items = search_jobs(args.keyword, args.limit)
    except Exception as e:  # noqa: BLE001
        print(f"[wanted] 검색 실패: {e}")
        return
    print(f"[wanted] {len(items)}건 검색됨")

    saved, skipped, failed = 0, 0, 0
    for item in items:
        company = (item.get("company") or {}).get("name") or item.get("company_name") or "unknown"
        title = item.get("position") or item.get("title") or "untitled"
        job_id = item.get("id")

        existing = find_existing_job(jobs_dir, company, title)
        if existing:
            print(f"  - 스킵(중복): {company} / {title} -> {existing.name}")
            skipped += 1
            continue

        url = f"https://www.wanted.co.kr/wd/{job_id}" if job_id else ""
        try:
            detail = fetch_job_detail(job_id)
            body = job_to_markdown_body(detail)
        except Exception as e:  # noqa: BLE001
            print(f"  ! 상세 조회 실패({company}/{title}): {e} — 목록 정보만 저장")
            body = "(상세 페이지 조회 실패. URL을 직접 열어 확인하세요.)"
            failed += 1

        path = write_job_markdown(
            jobs_dir=jobs_dir,
            company=company,
            title=title,
            url=url,
            source=SOURCE,
            deadline="",
            body=body,
        )
        print(f"  + 저장: {path.name}")
        saved += 1
        time.sleep(args.delay)

    print(f"[wanted] 완료 — 저장 {saved}건, 중복 스킵 {skipped}건, 상세 조회 실패 {failed}건")


if __name__ == "__main__":
    main()
