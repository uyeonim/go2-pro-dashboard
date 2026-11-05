#!/bin/bash
# 로봇에 연결해서 SLAM(slam_toolbox) + RViz 실행 → 리모컨으로 몰면서 지도 만들기
#   bash /mnt/c/Users/yeonwoo/robotdog/ros2/run_slam.sh
# 로봇은 WebRTC를 한 번에 하나만 받으니, 대시보드(dashboard.py)와 폰 앱은 꺼 두세요.
set -e
DIR="$(cd "$(dirname "$0")/.." && pwd)"   # robotdog 프로젝트 폴더

# .env에서 로봇 IP와 AES 키 읽기 (Windows 줄바꿈 \r 제거)
if [ -f "$DIR/.env" ]; then
  while IFS='=' read -r k v; do
    v="${v%$'\r'}"
    case "$k" in
      UNITREE_ROBOT_IP) export ROBOT_IP="$v" ;;
      UNITREE_AES_128_KEY) export ROBOT_AES_KEY="$v" ;;
    esac
  done < <(grep -E '^[A-Z0-9_]+=' "$DIR/.env")
fi
[ -n "$ROBOT_IP" ] || { echo "❌ $DIR/.env 에 UNITREE_ROBOT_IP가 없어요"; exit 1; }
[ -n "$ROBOT_AES_KEY" ] || echo "⚠ AES 키가 없어요. 펌웨어 1.1.15 이상이면 연결이 안 돼요."
export CONN_TYPE=webrtc

source /opt/ros/jazzy/setup.bash
source ~/go2_ws/install/setup.bash
cd ~/go2_ws

echo "로봇 $ROBOT_IP 에 연결해서 SLAM + RViz를 시작해요."
echo "RViz가 뜨면 리모컨으로 로봇을 천천히 몰아서 방을 한 바퀴 도세요."
echo "지도 저장: 다른 WSL 창에서  bash $DIR/ros2/save_map.sh 이름"
ros2 launch go2_robot_sdk robot.launch.py nav2:=false foxglove:=false joystick:=false teleop:=false
