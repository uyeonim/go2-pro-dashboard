#!/bin/bash
# run_*.sh 가 공통으로 불러 쓰는 설정 (직접 실행하지 않음)
#  - .env에서 로봇 IP / AES 키 읽기
#  - ROS2 환경 불러오기
#  - Nav2 속도를 안전한 값으로 낮추기 (드라이버 기본값은 3.0 m/s로 실내에서 위험)

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # robotdog 프로젝트 폴더
SDK=~/go2_ws/src/go2_ros2_sdk/go2_robot_sdk

# ── .env 읽기 (Windows 줄바꿈 \r 제거) ──
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
[ -f ~/go2_ws/install/setup.bash ] || { echo "❌ 아직 설치가 안 됐어요: bash $DIR/ros2/setup_wsl.sh"; exit 1; }
source ~/go2_ws/install/setup.bash

# ── Nav2 안전 속도 ──
# 바꾸려면 실행 앞에 붙이기:  MAX_V=0.5 MAX_W=1.0 bash run_nav_live.sh
MAX_V="${MAX_V:-0.35}"   # 전진 최대 속도 (m/s)
MAX_W="${MAX_W:-0.8}"    # 회전 최대 속도 (rad/s)
apply_safe_speed() {
  local f="$SDK/config/nav2_params.yaml"
  [ -f "$f.orig" ] || cp "$f" "$f.orig"          # 원본은 한 번만 백업
  cp "$f.orig" "$f"                               # 매번 원본에서 다시 적용
  sed -i -E \
    -e "s/^(\s*max_vel_x:).*/\1 $MAX_V/" \
    -e "s/^(\s*max_speed_xy:).*/\1 $MAX_V/" \
    -e "s/^(\s*max_vel_theta:).*/\1 $MAX_W/" \
    -e "s/^(\s*acc_lim_x:).*/\1 0.8/" \
    -e "s/^(\s*acc_lim_theta:).*/\1 1.5/" \
    -e "s/^(\s*decel_lim_x:).*/\1 -0.8/" \
    -e "s/^(\s*decel_lim_theta:).*/\1 -1.5/" \
    "$f"
  # --symlink-install이 아니라 복사본이 설치돼 있으면 그것도 맞춤
  local inst=~/go2_ws/install/go2_robot_sdk/share/go2_robot_sdk/config/nav2_params.yaml
  [ -f "$inst" ] && [ ! -L "$inst" ] && cp "$f" "$inst"
  echo "🛡 Nav2 속도 제한: 전진 ${MAX_V} m/s, 회전 ${MAX_W} rad/s"
}

safety_banner() {
  cat <<'EOF'
────────────────────────────────────────────────────────
 ⚠ 로봇이 실제로 움직여요
   · 넓고 장애물 없는 곳, 사람이 가까이 있지 않게
   · 리모컨을 손에 들고: 비상정지 = L2 + B
   · 이 창에서 Ctrl+C → 0.5초 안에 Nav2 명령이 끊겨 로봇이 멈춰요
   · 대시보드(dashboard.py)와 폰 앱은 꺼 두세요 (로봇 연결은 1개만)
────────────────────────────────────────────────────────
EOF
}
