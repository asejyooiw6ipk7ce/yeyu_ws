#!/usr/bin/env python3
"""사람인(saramin.co.kr) 채용공고 스크래퍼 — HTML 파싱 방식.

사람인은 검색용 공식 공개 API가 없어 검색 결과 페이지 HTML을 직접
파싱한다. 사람인이 페이지 구조(클래스명 등)를 바꾸면 곧바로 깨지는
구조이므로, 실행 결과가 0건이거나 이상하면 가장 먼저 아래 SELECTORS를
의심할 것.

셀렉터 갱신 방법:
  1. 브라우저에서 https://www.saramin.co.kr/zf_user/search/recruit?searchword=<키워드> 를 연다.
  2. 개발자도구(F12)로 공고 카드 하나를 선택해 실제 태그/클래스명을 확인한다.
  3. 아래 SELECTORS 값을 그에 맞게 고친다.

사용법:
    python3 scrape_saramin.py "로봇 시뮬레이션" --limit 20
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import requests
from bs4 import BeautifulSoup

sys.path.insert(0, str(Path(__file__).parent))
from common import find_existing_job, write_job_markdown  # noqa: E402

SEARCH_URL = "https://www.saramin.co.kr/zf_user/search/recruit"
HEADERS = {"User-Agent": "Mozilla/5.0 (compatible; jobsearch-bot/1.0)"}
SOURCE = "saramin"

# TODO(검증 필요): 실제 페이지 구조와 다를 수 있음. 위 "셀렉터 갱신 방법" 참고.
SELECTORS = {
    "item": "div.item_recruit",
    "title": "h2.job_tit a",
    "company": "strong.corp_name a",
    "deadline": "span.job_date .date",
}


def search_jobs(keyword: str, limit: int) -> list[dict]:
    results: list[dict] = []
    page = 1
    while len(results) < limit:
        params = {"searchword": keyword, "recruitPage": page}
        resp = requests.get(SEARCH_URL, params=params, headers=HEADERS, timeout=10)
        resp.raise_for_status()
        soup = BeautifulSoup(resp.text, "html.parser")
        items = soup.select(SELECTORS["item"])
        if not items:
            break
        for item in items:
            title_el = item.select_one(SELECTORS["title"])
            company_el = item.select_one(SELECTORS["company"])
            deadline_el = item.select_one(SELECTORS["deadline"])
            if not title_el or not company_el:
                continue
            href = title_el.get("href", "")
            url = href if href.startswith("http") else f"https://www.saramin.co.kr{href}"
            results.append(
                {
                    "title": title_el.get_text(strip=True),
                    "company": company_el.get_text(strip=True),
                    "url": url,
                    "deadline": deadline_el.get_text(strip=True) if deadline_el else "",
                }
            )
            if len(results) >= limit:
                break
        page += 1
        time.sleep(0.5)
    return results


def fetch_job_body(url: str) -> str:
    resp = requests.get(url, headers=HEADERS, timeout=10)
    resp.raise_for_status()
    soup = BeautifulSoup(resp.text, "html.parser")
    # TODO(검증 필요): 상세 페이지 본문만 정확히 뽑는 셀렉터가 아니라
    # 페이지 전체 텍스트에서 스크립트/네비 등을 제거한 근사치다. 정확도가
    # 중요하면 실제 상세 페이지 구조를 확인해 본문 컨테이너 셀렉터로 교체할 것.
    for tag in soup(["script", "style", "nav", "header", "footer"]):
        tag.decompose()
    text = soup.get_text("\n", strip=True)
    return text[:5000]


def main() -> None:
    parser = argparse.ArgumentParser(description="사람인 채용공고 스크래퍼")
    parser.add_argument("keyword")
    parser.add_argument("--limit", type=int, default=20)
    parser.add_argument("--jobs-dir", default="jobsearch/jobs")
    parser.add_argument("--delay", type=float, default=0.5)
    args = parser.parse_args()

    jobs_dir = Path(args.jobs_dir)
    print(f"[saramin] '{args.keyword}' 검색 중...")
    try:
        items = search_jobs(args.keyword, args.limit)
    except Exception as e:  # noqa: BLE001
        print(f"[saramin] 검색 실패: {e}")
        print("사이트 구조가 바뀌었을 수 있습니다 — 스크립트 상단 SELECTORS를 확인하세요.")
        return

    if not items:
        print("[saramin] 0건 — SELECTORS가 현재 사이트 구조와 안 맞을 수 있습니다.")
        return

    saved, skipped, failed = 0, 0, 0
    for item in items:
        existing = find_existing_job(jobs_dir, item["company"], item["title"])
        if existing:
            print(f"  - 스킵(중복): {item['company']} / {item['title']}")
            skipped += 1
            continue
        try:
            body = fetch_job_body(item["url"])
        except Exception as e:  # noqa: BLE001
            print(f"  ! 본문 조회 실패: {e}")
            body = "(본문 조회 실패. 원본 URL을 직접 확인하세요.)"
            failed += 1

        path = write_job_markdown(
            jobs_dir=jobs_dir,
            company=item["company"],
            title=item["title"],
            url=item["url"],
            source=SOURCE,
            deadline=item.get("deadline", ""),
            body=body,
        )
        print(f"  + 저장: {path.name}")
        saved += 1
        time.sleep(args.delay)

    print(f"[saramin] 완료 — 저장 {saved}건, 중복 스킵 {skipped}건, 본문 실패 {failed}건")


if __name__ == "__main__":
    main()
