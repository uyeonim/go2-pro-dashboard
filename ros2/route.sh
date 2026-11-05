#!/bin/bash
# 경로 만들기 / 따라가기 (run_nav.sh 또는 run_nav_live.sh를 켠 상태에서 다른 WSL 창에서)
#   wsl -d Ubuntu-24.04 bash /mnt/c/Users/yeonwoo/robotdog/ros2/route.sh record lab_loop
#   wsl -d Ubuntu-24.04 bash /mnt/c/Users/yeonwoo/robotdog/ros2/route.sh follow lab_loop [--loop] [--live]
#   wsl -d Ubuntu-24.04 bash /mnt/c/Users/yeonwoo/robotdog/ros2/route.sh list
set -e
H="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
source ~/go2_ws/install/setup.bash
case "$1" in
  record) shift; python3 "$H/record_route.py" "$@" ;;
  follow) shift; python3 "$H/follow_route.py" "$@" ;;
  list)   ls -1 "$H/../routes"/*.yaml 2>/dev/null | xargs -n1 basename 2>/dev/null || echo "저장한 경로가 없어요" ;;
  *) echo "사용법: route.sh record <이름> | follow <이름> [--loop] [--live] | list"; exit 1 ;;
esac
