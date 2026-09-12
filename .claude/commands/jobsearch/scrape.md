---
description: "채용 사이트에서 공고를 스크랩해 jobs/ 에 저장한다"
argument-hint: "<검색 키워드> [--source wanted|saramin|jobkorea] [--limit N]"
---

사용자가 지정한 키워드로 채용 공고를 스크랩해 `jobsearch/jobs/`에 저장한다.

입력: $ARGUMENTS (검색 키워드, 선택적으로 --source, --limit 옵션)

## 절차

1. $ARGUMENTS에서 키워드와 옵션을 파악한다. --source가 없으면 기본값은
   `wanted`다 (공개 API 기반이라 가장 안정적).
2. 해당하는 스크립트를 Bash로 실행한다 (저장소 루트에서 실행 가정):
   - wanted: `python3 jobsearch/scripts/scrape_wanted.py "<키워드>" --limit <N>`
   - saramin: `python3 jobsearch/scripts/scrape_saramin.py "<키워드>" --limit <N>`
   - jobkorea: `python3 jobsearch/scripts/scrape_jobkorea.py "<키워드>" --limit <N>`
3. saramin/jobkorea는 공식 API가 아닌 HTML 파싱이라 셀렉터가 깨져 있을 수
   있다. 스크립트가 0건을 가져오거나 에러를 내면:
   - 에러 메시지를 그대로 사용자에게 보여준다.
   - 스크립트 상단 SELECTORS/TODO 주석을 확인해 셀렉터를 고쳐야 할 수도
     있다고 안내한다. 직접 셀렉터를 추측해서 고치려 하지 말고, 사용자에게
     실제 페이지 HTML(또는 브라우저 개발자도구 스크린샷)을 요청한다.
4. 실행 후 결과 요약(저장 N건, 중복 스킵 N건, 실패 N건)을 사용자에게
   보고한다.
5. 여러 소스를 동시에 요청받으면 순서대로 실행하고 마지막에 합산 요약을
   보여준다.

## 중복 제거
스크립트가 회사명+제목 기준으로 자동 처리한다 (별도 조치 불필요).
