"""jobsearch 스크래퍼들이 공유하는 유틸리티.

공고 저장 파일명 규칙, dedup(회사명+제목 기준) 체크, frontmatter 포함
마크다운 작성을 한 곳에서 관리한다. 스크래퍼(scrape_*.py)마다 이 로직을
따로 구현하면 사이트별로 파일명/중복 처리 방식이 어긋날 수 있어 공통화했다.
"""
from __future__ import annotations

import re
import unicodedata
from datetime import datetime
from pathlib import Path
from typing import Optional


def slugify(text: str) -> str:
    """회사명/직무명을 파일명 조각으로 변환. 한글은 그대로 유지한다."""
    text = unicodedata.normalize("NFC", text or "").strip().lower()
    text = re.sub(r"[^\w\s가-힣-]", "", text)
    text = re.sub(r"[\s_]+", "-", text)
    return text.strip("-") or "unknown"


def find_existing_job(jobs_dir: Path, company: str, title: str) -> Optional[Path]:
    """같은 회사+제목 공고가 이미 저장돼 있으면 그 경로를 반환한다 (날짜 무관).

    스크랩 시점이 달라도 같은 공고면 회사명+제목이 같다고 보고 dedup한다.
    """
    if not jobs_dir.exists():
        return None
    prefix = f"{slugify(company)}_{slugify(title)}_"
    matches = sorted(jobs_dir.glob(f"{prefix}*.md"))
    return matches[0] if matches else None


def write_job_markdown(
    jobs_dir: Path,
    company: str,
    title: str,
    url: str,
    source: str,
    deadline: str,
    body: str,
    scraped_at: Optional[str] = None,
) -> Path:
    """jobs/{company}_{title}_{date}.md 형식으로 공고를 저장한다."""
    jobs_dir.mkdir(parents=True, exist_ok=True)
    scraped_at = scraped_at or datetime.now().strftime("%Y-%m-%d")
    filename = f"{slugify(company)}_{slugify(title)}_{scraped_at}.md"
    path = jobs_dir / filename

    def esc(value: str) -> str:
        return (value or "").replace('"', '\\"')

    frontmatter = (
        "---\n"
        f'company: "{esc(company)}"\n'
        f'title: "{esc(title)}"\n'
        f'url: "{esc(url)}"\n'
        f'scraped_at: "{scraped_at}"\n'
        f'source: "{esc(source)}"\n'
        f'deadline: "{esc(deadline)}"\n'
        "---\n\n"
    )
    path.write_text(frontmatter + f"# {title} — {company}\n\n" + body.strip() + "\n", encoding="utf-8")
    return path
