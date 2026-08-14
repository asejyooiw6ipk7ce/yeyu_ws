// ================================================================
// yeyu_ws/src/yeyu_launch/ecosystem.config.js
//
// PM2는 supervisor 프로세스 "하나만" 감시합니다.
// 6개 노드는 supervisor가 만드는 tmux 세션(yeyu_robot) 안에서
// 각자 자체 재시작 루프로 돌아갑니다.
//
// 사용법:
//   cd ~/yeyu_ws/src/yeyu_launch
//   pm2 start ecosystem.config.js
//   pm2 status                              # supervisor 프로세스 상태
//   ./turtlebot3_tmux.sh status              # 개별 노드(창) 상태
//   ./turtlebot3_tmux.sh attach              # tmux 세션에 직접 접속
// ================================================================
const path = require("path");

const scriptDir = __dirname;
const logDir = path.join(scriptDir, "logs");

module.exports = {
  apps: [
    {
      name: "turtlebot3-robot",
      script: path.join(scriptDir, "turtlebot3_pm2_supervisor.sh"),
      interpreter: "/bin/bash",
      cwd: scriptDir,

      exec_mode: "fork",
      instances: 1,
      autorestart: true,
      watch: false,

      restart_delay: 5000,
      min_uptime: "30s",
      max_restarts: 20,
      kill_timeout: 15000,

      time: true,
      merge_logs: true,

      out_file: path.join(logDir, "turtlebot3-supervisor-out.log"),
      error_file: path.join(logDir, "turtlebot3-supervisor-error.log"),
    },
  ],
};