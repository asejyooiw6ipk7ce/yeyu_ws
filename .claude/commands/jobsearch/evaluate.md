---
description: "profile.md와 공고 하나를 비교해 적합도/갭 분석을 evaluation.md로 만든다"
argument-hint: "<jobs/ 파일명 또는 회사명 키워드>"
---

`jobsearch/profile.md`와 `jobsearch/jobs/`의 공고 하나를 비교해 적합도를
평가하고, `jobsearch/applications/{company}_{title}/evaluation.md`를 만든다.

입력: $ARGUMENTS (대상 공고 — jobs/ 파일명 또는 회사명으로 특정)

## 절차

1. `jobsearch/profile.md`를 읽는다. 비어 있으면 먼저 `/jobsearch:setup`을
   실행하라고 안내하고 중단한다.
2. $ARGUMENTS로 대상 공고를 `jobsearch/jobs/`에서 찾는다. 특정이 안 되면
   후보 목록을 보여주고 확인을 받는다.
3. 공고의 자격요건/우대사항/주요업무를 하나씩 프로필과 대조한다.
4. `jobsearch/applications/{company}_{title}/` 폴더를 만들고 (없으면)
   `evaluation.md`를 **아래 형식 그대로** 작성한다. 이 형식은 집계 스크립트
   (`jobsearch/scripts/aggregate.py`)가 YAML frontmatter를 파싱하므로
   절대 구조를 바꾸지 않는다.

```markdown
---
company: "{회사명}"
title: "{직무명}"
score: {0-100 정수}
grade: "{score>=80: 상 / score>=50: 중 / 그 외: 하}"
evaluated_at: "{YYYY-MM-DD}"
gaps:
  - requirement: "{부족한 요구사항}"
    priority: "{높음|중간|낮음}"
    workaround: "{우회 가능 설명 또는 '진짜 학습/경험 필요'}"
---

## 적합도 평가 — {company} / {title}

**종합 점수**: {score}/100 ({grade})

### 매칭되는 경험
- {경험명}: 공고의 [{요구사항}]과 매칭 — 근거: {구체적 근거}

### 부족한 항목
- **{요구사항}** (우선순위: {높음|중간|낮음})
  - 프로필에 없음/약함
  - 우회 가능 여부: {설명}
```

## 점수 산정 기준 (일관성을 위해 항상 이 기준 사용)
- 공고의 핵심 요구사항(자격요건에 명시된 것) 각각에 매칭 여부를 확인한다.
- 핵심 요구사항 매칭 비율이 점수의 큰 축이 되고, 우대사항 매칭은 가산점
  정도로 취급한다.
- 인접 경험으로 우회 가능한 항목은 완전히 없는 것보다 감점을 적게 준다.
- 점수를 부풀리지 않는다 — 이 점수는 사용자가 여러 공고 중 어디에 시간을
  쓸지 판단하는 데 쓰인다. 과장하면 판단을 그르친다.

## 완료 후
평가가 끝나면 결과를 요약해서 보여주고, 여러 건을 평가했다면
`python3 jobsearch/scripts/aggregate.py` 실행을 제안한다.
