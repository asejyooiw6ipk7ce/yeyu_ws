// ================================================================
// PM2 ecosystem 설정 파일
// 사용법:
//   pm2 start ecosystem.config.js       # 전체 시작
//   pm2 status                          # 상태 확인
//   pm2 logs                            # 전체 로그 실시간 확인
//   pm2 logs 01-bringup                 # 특정 노드 로그만 확인
//   pm2 restart <name>                  # 특정 노드 재시작
//   pm2 save                            # 현재 프로세스 목록 저장(재부팅 시 복원용)
// ================================================================
const os = require('os');
const path = require('path');

const HOME = os.homedir();
const WS = path.join(HOME, 'yeyu_ws', 'src', 'yeyu_launch');   // yeyu_launch까지 포함
const NODES_DIR = path.join(WS, 'nodes');
const LOG_DIR = path.join(WS, 'logs');

// 모든 앱에 공통으로 적용할 옵션
const common = {
  interpreter: 'bash',
  autorestart: true,          // 죽으면 자동 재시작
  max_restarts: 50,           // 짧은 시간 내 반복 크래시 시 최대 재시도 횟수
  min_uptime: '10s',          // 이 시간 이상 떠 있어야 "정상 실행"으로 간주
  restart_delay: 3000,        // 재시작 전 대기 시간(ms)
  exp_backoff_restart_delay: 200, // 반복 실패 시 재시작 간격을 점점 늘림
  merge_logs: true,
  time: true,                 // 로그 각 줄에 타임스탬프 추가
};

function nodeApp(name, scriptFile) {
  return {
    name,
    script: path.join(NODES_DIR, scriptFile),
    cwd: WS,
    out_file: path.join(LOG_DIR, `${name}-out.log`),
    error_file: path.join(LOG_DIR, `${name}-error.log`),
    ...common,
  };
}

module.exports = {
  apps: [
    nodeApp('01-bringup', '01_bringup.sh'),
    nodeApp('02-camera', '02_camera.sh'),
    nodeApp('03-sensor', '03_sensor.sh'),
    nodeApp('04-audio', '04_audio.sh'),
    nodeApp('05-navigation', '05_navigation.sh'),
    nodeApp('06-driving', '06_driving.sh'),
    // GUI 대시보드는 요청에 따라 제외. 나중에 추가하려면 아래 주석 해제:
    // nodeApp('07-gui', '07_gui.sh'),
  ],
};