#!/bin/bash
# ② 저장한 지도 위에서 위치 찾기(AMCL) + Nav2 이동 — 같은 경로를 반복할 때
#   wsl -d Ubuntu-24.04 bash /mnt/c/Users/yeonwoo/robotdog/ros2/run_nav.sh lab
#   (maps/lab.yaml 을 씀 — save_map.sh lab 으로 만든 지도)
# RViz가 뜨면 "2D Pose Estimate"로 로봇의 실제 위치와 방향을 지도에 한 번 찍어 주세요.
set -e
source "$(dirname "$0")/nav_common.sh"
NAME="${1:?지도 이름을 주세요. 예: run_nav.sh lab}"
MAP="$DIR/maps/$NAME.yaml"
[ -f "$MAP" ] || { echo "❌ 지도가 없어요: $MAP  (먼저 save_map.sh $NAME)"; ls "$DIR/maps" 2>/dev/null; exit 1; }
MAP_YAML="$MAP" apply_safe_speed
safety_banner
echo "🗺 지도: $MAP"
cd ~/go2_ws
# navigation.launch.py는 joystick:=true일 때만 twist_mux를 켜서, Nav2 명령이 로봇에 가려면 true여야 함
ros2 launch go2_robot_sdk navigation.launch.py map:="$MAP" joystick:=true foxglove:=false
