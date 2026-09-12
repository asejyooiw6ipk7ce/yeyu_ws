# jobsearch — 한국 채용공고 구직 자동화 도구

`MadsLorentzen/ai-job-search`(덴마크 채용 포털 대상, Claude Code 기반)의
구조(프로필 파일 + 단계별 명령 + 드래프터-리뷰어)를 가져와 한국 채용
사이트(원티드/사람인/잡코리아)에 맞게 다시 설계한 버전.

## 파이프라인

```
/jobsearch:setup          profile.md 채우기 (문답형)
        │
        ▼
/jobsearch:scrape         공고 스크랩 → jobs/{company}_{title}_{date}.md
        │
        ▼
/jobsearch:evaluate       공고 1건 × profile.md → applications/{company}_{title}/evaluation.md
        │                 (점수 + 매칭 경험 + 부족 항목 + 우회 가능 여부, 고정 포맷)
        ▼
/jobsearch:apply          resume.md, cover_letter.md 초안 작성
        │
        ▼
/jobsearch:review         리뷰어 페르소나로 초안 검수 → review_notes.md
        │
        ▼
/jobsearch:interview-prep 부족 항목 기반 예상 질문 + 답변 전략 → interview_prep.md
```

여러 공고를 `/jobsearch:evaluate`로 평가한 뒤에는:

```bash
python3 jobsearch/scripts/aggregate.py
```

로 점수순 정렬표와 "여러 공고에서 공통으로 부족한 스킬" 빈도를 볼 수 있다.

## 폴더 구조

```
jobsearch/
├── profile.md                          # 내 경력/스킬
├── jobs/
│   └── {company}_{title}_{date}.md     # 스크랩한 공고 (YAML frontmatter + 본문)
├── applications/
│   └── {company}_{title}/
│       ├── evaluation.md               # 적합도 평가 + 갭 분석 (YAML frontmatter 포함)
│       ├── resume.md
│       ├── cover_letter.md
│       ├── review_notes.md
│       └── interview_prep.md
└── scripts/
    ├── common.py                       # dedup + 파일 저장 공통 유틸
    ├── scrape_wanted.py                # 원티드: 비공식 내부 API 사용
    ├── scrape_saramin.py               # 사람인: HTML 파싱 (취약)
    ├── scrape_jobkorea.py              # 잡코리아: HTML 파싱 (취약)
    └── aggregate.py                    # evaluation.md들 모아 비교
```

슬래시 명령 정의는 `.claude/commands/jobsearch/*.md`에 있다.

## 설치

```bash
pip install -r jobsearch/requirements.txt
```

## 사이트별 스크래퍼 안정성

| 사이트 | 방식 | 안정성 |
|---|---|---|
| 원티드 | 비공식 내부 API (`www.wanted.co.kr/api/v4/*`) | 정식 공개 API가 아니므로 원티드가 구조를 바꾸면 깨짐. 스크립트가 방어적으로 에러를 내도록 작성돼 있어, 깨지면 바로 알 수 있음 |
| 사람인 | 검색 결과 페이지 HTML 파싱 | 공식 API 없음. 클래스명 등 셀렉터에 의존해 가장 취약. `scrape_saramin.py` 상단 `SELECTORS`를 주기적으로 검증 필요 |
| 잡코리아 | 검색 결과 페이지 HTML 파싱 | 사람인과 동일한 이유로 취약 |

스크래퍼가 0건을 가져오거나 에러를 내면 십중팔구 대상 사이트의 마크업/API가
바뀐 것이다 — 브라우저 개발자도구로 실제 요청/구조를 확인하고 각 스크립트
상단의 URL/SELECTORS를 갱신할 것.

## 스펙의 미결정 사항에 대한 결정

- **적합도 점수: 숫자(0-100) + 등급(상/중/하) 둘 다 사용.** 등급만 쓰면
  `aggregate.py`가 여러 공고를 세밀하게 정렬할 수 없어서 숫자를 1차
  기준으로 삼고, 등급은 점수에서 자동 산출(80+: 상, 50-79: 중, 그 외: 하)해
  사람이 훑어보기 쉽게 병기한다.
- **스크래핑 우선순위: 원티드 → 사람인/잡코리아.** 스펙에서 이미 명시된
  순서(공개 API 유무 기준)를 그대로 따름.
- **파이프라인 형태: Claude Code 슬래시 명령.** 원본 프로젝트와 동일한
  방식을 유지해 `/jobsearch:evaluate` 같은 형태로 바로 실행 가능하게 함.
  실제 스크래핑처럼 결정적(deterministic) 로직이 필요한 부분은 Python
  스크립트로 분리하고, 명령은 그 스크립트를 호출 + 결과 해석을 맡는다.
- **비교 대시보드: 별도 뷰 없이 터미널 표로 충분하다고 판단.**
  `aggregate.py`가 정렬된 텍스트 표를 출력하는 선에서 우선 구현. 공고
  수가 많아져 터미널 출력으로 부족해지면 이 스크립트를 확장하거나 별도
  뷰를 추가하는 방향을 그때 고려.

## evaluation.md 포맷이 고정된 이유

`aggregate.py`는 `evaluation.md` 상단의 YAML frontmatter(`score`, `grade`,
`gaps`)를 파싱해서 집계한다. 자연어 본문만 파싱하면 표현이 조금만 달라져도
깨지기 쉬워서, 집계에 필요한 값은 frontmatter에 구조화해 넣고 본문은 사람이
읽기 위한 설명으로 분리했다. `/jobsearch:evaluate` 명령은 항상 이 frontmatter
형식을 지켜서 파일을 쓰도록 지시돼 있다.
