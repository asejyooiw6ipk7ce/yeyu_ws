---
description: "evaluation.md의 부족 항목을 기반으로 예상 면접 질문과 답변 전략을 만든다"
argument-hint: "<회사명 또는 applications/ 폴더명>"
---

`evaluation.md`의 "부족한 항목"과 `resume.md`를 바탕으로 예상 면접 질문과
답변 전략을 `interview_prep.md`로 작성한다.

입력: $ARGUMENTS (대상 지원 폴더)

## 절차

1. `applications/{company}_{title}/evaluation.md`, `resume.md`,
   `jobsearch/profile.md`, 원본 공고를 읽는다.
2. **부족 항목 기반 압박 질문**: evaluation.md의 `gaps` 각각에 대해
   면접관이 파고들 법한 질문을 만든다. 우선순위가 "높음"인 항목부터
   반드시 다룬다. 각 질문마다:
   - 전략을 제시한다 (인접 경험으로 연결 / 학습 계획 제시 / 솔직히
     인정하고 빠른 습득력 강조 — 우회 가능 여부에 따라 다르게 선택).
   - 실제로 말할 수 있는 답변 초안을 써준다 (원론적인 조언이 아니라
     사용자의 실제 경험을 근거로).
3. **매칭 항목 기반 심화 질문**: resume.md의 강점에 대해 "더 자세히
   설명해달라"는 심화 질문을 만들고, **STAR 구조**(Situation-Task-
   Action-Result)로 답변 초안을 작성한다.
4. `applications/{company}_{title}/interview_prep.md`에 아래 형식으로 저장:

```markdown
## 예상 질문 — {company} / {title}

### 부족 항목 기반 압박 질문
- Q: [부족 항목]에 대한 경험이 없어 보이는데?
  - 전략: ...
  - 답변 초안: ...

### 매칭 항목 기반 심화 질문
- Q: [강점]에 대해 더 자세히 설명해달라
  - 답변 초안 (STAR):
    - S: ...
    - T: ...
    - A: ...
    - R: ...
```

5. 거짓말이나 없는 경험을 지어내는 답변은 절대 만들지 않는다 — 압박
   질문일수록 "솔직히 인정 + 빠른 습득 계획"이 더 나은 전략일 때가 많다.
