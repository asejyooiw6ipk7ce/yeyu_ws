1) 브랜치 구성
'''Makedown
main
develop
feature/navigation
feature/vision
feature/sensor
feature/gui
feature/mission
feature/automation
'''
main 브랜치에는 동작이 확인된 소스만 병합합니다.
개발은 develop 또는 기능 브랜치에서 진행합니다.
기능 구현 후 다른 팀원이 검토하고 병합합니다.
로봇에서 직접 수정한 소스를 반드시 GitHub에 반영합니다.
빌드 파일은 저장소에 등록하지 않습니다.

.gitignore에는 다음 내용을 포함합니다.
'''Makedown
build/
install/
log/
__pycache__/
*.pyc
.vscode/
.idea/
'''

2) Commit 메시지 예시
'''Makedown
feat: add ArUco docking controller
fix: correct wheel separation parameter
docs: add system requirement document
test: add waypoint repeatability result
refactor: separate navigation and mission nodes
'''

4) README 필수 내용
프로젝트 소개
서비스 시나리오
하드웨어 구성
소프트웨어 구성
패키지 구조
설치 방법
빌드 방법
실행 방법
자동 실행 방법
토픽, 서비스, 액션 목록
지도와 경유점 설명
시험 결과
알려진 문제
팀원 역할
