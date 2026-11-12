#!/usr/bin/env bash
# Start the phone control page on PC2 (moves nothing: teleop is only launched from the page).
# Used by `tools/h2_arm_only.sh web` and by the unitree crontab at boot:
#   @reboot /home/unitree/h2-upstream-pr321/pc2_start_web.sh
R=/home/unitree/h2-upstream-pr321
PY=/home/unitree/anaconda3/envs/tv/bin/python
LOG=/home/unitree/h2-logs/web_server.log
mkdir -p /home/unitree/h2-logs
if pgrep -f "python quest_h2_[w]eb.py" >/dev/null; then echo "web page already running"; exit 0; fi
# The FSM reader binds DDS to eth0 once at start, so wait for the robot network (up to 2 min).
for _ in $(seq 60); do ip -4 addr show eth0 2>/dev/null | grep -q "inet 192.168.123." && break; sleep 2; done
echo "$(date '+%F %T') starting control page ($([ -t 1 ] && echo manual || echo boot/remote))" >>"$LOG"
cd "$R" || exit 1
setsid nohup "$PY" quest_h2_web.py --port 8787 >>"$LOG" 2>&1 </dev/null &
sleep 2; tail -n 1 "$LOG"
