#!/bin/bash
# ③ 저장한 지도 + SLAM 위치추정 + Nav2 (추천)
#   wsl -d Ubuntu-24.04 bash /mnt/c/Users/yeonwoo/robotdog/ros2/run_nav_map.sh lab
#   (maps/lab.posegraph 를 씀 — save_map.sh lab 으로 만든 지도)
# AMCL(run_nav.sh)보다 위치를 훨씬 덜 놓침: 지도 만들 때 쓴 SLAM 스캔매칭으로 위치를 잡음.
# 시작: 로봇을 "지도 딸 때 출발한 자리·방향"에 두고 켜면 자동으로 맞음.
#       다른 데서 켰으면 RViz "2D Pose Estimate"로 대충 찍어 주면 스캔매칭이 정확히 맞춰 줌.
set -e
source "$(dirname "$0")/nav_common.sh"
NAME="${1:?지도 이름을 주세요. 예: run_nav_map.sh lab}"
PG="$DIR/maps/$NAME"
[ -f "$PG.posegraph" ] || { echo "❌ $PG.posegraph 가 없어요 (SLAM 켠 채로 save_map.sh $NAME 로 저장)"; exit 1; }
apply_safe_speed
safety_banner
echo "🗺 지도(posegraph): $PG"

# SDK의 SLAM 설정을 복사해서 "위치추정 모드"로
LOC=/tmp/go2_localization_params.yaml
sed -E \
  -e 's/^(\s*mode:).*/\1 localization/' \
  -e 's/^(\s*transform_timeout:).*/\1 1.0/' \
  "$SDK/config/mapper_params_online_async.yaml" > "$LOC"
sed -i "/^\s*mode: localization/a\\    map_file_name: $PG\\n    map_start_at_dock: true" "$LOC"

cd ~/go2_ws
# 위치추정(slam_toolbox) 따로 띄우고, 로봇+Nav2는 SLAM 없이
ros2 launch slam_toolbox localization_launch.py slam_params_file:="$LOC" use_sim_time:=false &
LOC_PID=$!
trap 'kill $LOC_PID 2>/dev/null' EXIT
ros2 launch go2_robot_sdk robot.launch.py \
  slam:=false nav2:=true teleop:=true joystick:=false foxglove:=false
