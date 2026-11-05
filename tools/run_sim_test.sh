#!/bin/bash
# Headless (GUI'siz) uçtan uca test: Gazebo + Nav2 başlatır, nav_goal_test.py'yi
# çalıştırır, her şeyi kapatır. Çıkış kodu testin çıkış kodudur.
#
#   tools/run_sim_test.sh            # 1 tur
#   tools/run_sim_test.sh --rounds 3
#
# Loglar: log/sim_test/{sim,test}.log

WS="$(cd "$(dirname "$0")/.." && pwd)"
LOG_DIR="$WS/log/sim_test"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
# Başka çalışan ROS sistemleriyle karışmasın.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

# Launch kendi süreç grubunda çalışır; kapatırken tüm grubu (gzserver dahil) öldür.
cleanup() {
  kill -INT -- -"$LAUNCH_PID" 2>/dev/null
  sleep 5
  kill -9 -- -"$LAUNCH_PID" 2>/dev/null
}
trap cleanup EXIT

setsid ros2 launch my_robot_description bringup.launch.py gui:=false rviz:=false \
  > "$LOG_DIR/sim.log" 2>&1 &
LAUNCH_PID=$!

timeout 1800 python3 "$WS/tools/nav_goal_test.py" "$@" 2>&1 | tee "$LOG_DIR/test.log"
exit "${PIPESTATUS[0]}"
