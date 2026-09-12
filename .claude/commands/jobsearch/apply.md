---
description: "evaluation.md를 바탕으로 맞춤 이력서/자소서 초안을 작성한다"
argument-hint: "<회사명 또는 applications/ 폴더명>"
---

`jobsearch/applications/{company}_{title}/evaluation.md`를 바탕으로 맞춤
이력서(`resume.md`)와 자소서(`cover_letter.md`) 초안을 작성한다.

입력: $ARGUMENTS (대상 지원 폴더)

## 절차

1. 대상 `applications/{company}_{title}/` 폴더의 `evaluation.md`를 읽는다.
   없으면 먼저 `/jobsearch:evaluate`부터 하라고 안내하고 중단한다.
2. `jobsearch/profile.md`와 해당 `jobs/` 원본 공고도 함께 읽는다.
3. **resume.md** 작성 규칙:
   - evaluation.md의 "매칭되는 경험"에 있는 항목들을 공고의 표현/키워드에
     맞춰 재서술한다 (있지도 않은 경험을 새로 지어내지 않는다 — 어디까지나
     같은 사실을 다른 각도/단어로 설명한다).
   - 정량적 성과가 있으면 반드시 앞으로 끌어온다.
   - "부족한 항목" 중 우회 가능한 것은 인접 경험으로 자연스럽게 연결해
     서술하되, 없는 경험을 있다고 쓰지 않는다.
   - 우회 불가능한 부족 항목은 이력서에서 억지로 언급하지 않는다 (침묵도
     전략이다).
4. **cover_letter.md** 작성 규칙:
   - 공고/회사의 구체적 키워드를 반영해 "아무 회사에나 보내는 자소서"처럼
     보이지 않게 한다.
   - 지원 동기는 회사의 실제 사업/기술 방향과 사용자의 경험을 연결한다.
   - 과장, 상투적 표현("열정적으로", "책임감을 가지고" 같은 근거 없는
     수식어)을 최소화한다 — 대신 구체적 사례로 증명한다.
5. 두 파일 모두 `applications/{company}_{title}/`에 저장한다.
6. 작성 후 사용자에게 `/jobsearch:review`로 검수를 받으라고 안내한다.
