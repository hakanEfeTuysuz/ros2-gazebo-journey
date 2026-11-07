#!/bin/bash
# Headless test: Gazebo + obstacle_avoidance düğümü (Nav2 yok), avoidance_monitor.py ile ölç.
#   tools/run_avoidance_test.sh --duration 90
# Loglar: log/avoidance_test/{sim,node,test}.log

WS="$(cd "$(dirname "$0")/.." && pwd)"
LOG_DIR="$WS/log/avoidance_test"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

cleanup() {
  kill -INT -- -"$SIM_PID" -"$NODE_PID" 2>/dev/null
  sleep 5
  kill -9 -- -"$SIM_PID" -"$NODE_PID" 2>/dev/null
}
trap cleanup EXIT

setsid ros2 launch my_robot_description gazebo.launch.py gui:=false > "$LOG_DIR/sim.log" 2>&1 &
SIM_PID=$!
setsid ros2 run my_robot_core obstacle_avoidance --ros-args -p use_sim_time:=true \
  > "$LOG_DIR/node.log" 2>&1 &
NODE_PID=$!

python3 "$WS/tools/avoidance_monitor.py" "$@" 2>&1 | tee "$LOG_DIR/test.log"
exit "${PIPESTATUS[0]}"
