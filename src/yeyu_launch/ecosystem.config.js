// ================================================================
// yeyu_ws/src/yeyu_launch/ecosystem.config.js
// 구조: yeyu_launch/{nodes, lib, logs, scripts, ecosystem.config.js}
// supervisor 실제 파일은 scripts/ 안에 있음
// ================================================================
const path = require("path");

const launchDir = __dirname;                                   // yeyu_launch 폴더 자신
const supervisorScript = path.join(launchDir, "scripts", "yeyu_pm2_supervisor.sh");
const logDir = path.join(launchDir, "logs");

module.exports = {
  apps: [
    {
      name: "yeyu-robot",
      script: supervisorScript,
      interpreter: "/bin/bash",
      cwd: launchDir,

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

      out_file: path.join(logDir, "yeyu-supervisor-out.log"),
      error_file: path.join(logDir, "yeyu-supervisor-error.log"),
    },
  ],
};