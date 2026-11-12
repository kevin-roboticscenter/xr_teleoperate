#!/usr/bin/env bash
# Keep the Quest (8012) and phone control page (8787) forwards up on the lab laptop
# Every laptop IPv4 address:<port> -> the selected H2 Ethernet/Wi-Fi address.
# Started at boot by the laptop's crontab:
#   @reboot $HOME/h2-socat/laptop_h2_socat.sh
# Listens across Wi-Fi changes, (re)starts missing forwards. Log: ~/h2-socat/socat.log
BIND=${H2_FORWARD_BIND:-0.0.0.0}
LINK_TOOL=${H2_LINK_TOOL:-$HOME/.local/bin/h2-link}
PC2=$("$LINK_TOOL" host) || exit 1
read -r -a PORTS <<< "${H2_FORWARD_PORTS:-8012 8787}"
DIR=$HOME/h2-socat
LOG=$DIR/socat.log
mkdir -p "$DIR"
exec 9>"$DIR/.lock"
flock -n 9 || { echo "already running"; exit 0; }
exec >>"$LOG" 2>&1 </dev/null
echo "$(date '+%F %T') supervisor start"
echo "H2 destination: $PC2"
declare -A CHILDREN
SLEEP_PID=
cleanup() {
  trap - EXIT TERM INT
  for child in "${CHILDREN[@]}"; do kill "$child" 2>/dev/null || true; done
  [ -z "$SLEEP_PID" ] || kill "$SLEEP_PID" 2>/dev/null || true
}
trap 'cleanup; exit 0' TERM INT
trap cleanup EXIT
while true; do
  for p in "${PORTS[@]}"; do
    if [ -z "${CHILDREN[$p]:-}" ] || ! kill -0 "${CHILDREN[$p]}" 2>/dev/null; then
      echo "$(date '+%F %T') starting forward $p"
      if [ "$p" = 8787 ]; then
        setsid python3 "$HOME/h2-wuji-teleop/wuji_glove/dashboard_gateway.py" --host "$BIND" --port "$p" 9>&- &
      else
        setsid socat TCP-LISTEN:$p,bind=$BIND,fork,reuseaddr TCP:$PC2:$p,connect-timeout=5 9>&- &
      fi
      CHILDREN[$p]=$!
    fi
  done
  sleep 5 9>&- &
  SLEEP_PID=$!
  wait "$SLEEP_PID" || true
  SLEEP_PID=
done
