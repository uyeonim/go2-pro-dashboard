#!/bin/bash
# ① 실시간 지도 + Nav2 이동 — 저장한 지도 없이 바로 시험
#   wsl -d Ubuntu-24.04 bash /mnt/c/Users/yeonwoo/robotdog/ros2/run_nav_live.sh
# RViz 위쪽 "Nav2 Goal" 버튼 → 지도에서 목표 지점을 클릭하고 끌어서 방향 지정 → 로봇이 이동
# 먼저 리모컨으로 로봇을 세워 두세요 (L2 + A, 필요하면 START).
set -e
source "$(dirname "$0")/nav_common.sh"
apply_safe_speed
safety_banner
cd ~/go2_ws
ros2 launch go2_robot_sdk robot.launch.py \
  slam:=true nav2:=true teleop:=true joystick:=false foxglove:=false
