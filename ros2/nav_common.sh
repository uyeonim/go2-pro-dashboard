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
    -e 's#plugin: "(nav2_[a-z_]+)/([A-Za-z0-9]+)"#plugin: "\1::\2"#' \
    "$f"
  # 무선(WebRTC)이라 로봇 데이터가 ~1초 늦게 옴 → 시간 허용치를 넉넉히 (기본값이면 "too old"로 목표 포기)
  sed -i -E 's/^(\s*transform_tolerance:).*/\1 1.0/' "$f"
  sed -i -E '/^controller_server:/,/ros__parameters:/ s/(ros__parameters:.*)/\1\n    costmap_update_timeout: 3.0/' "$f"
  sed -i -E '/^\s+(local|global)_costmap:\s*$/,/ros__parameters:/ s/^(\s*)ros__parameters:.*/&\n\1  transform_tolerance: 1.0/' "$f"
  # 느림 해결: 제어 주기 3Hz → 10Hz (3Hz면 로봇이 멈칫멈칫), sim_time 끄기
  sed -i -E -e 's/^(\s*controller_frequency:).*/\1 10.0/' -e 's/^(\s*use_sim_time:).*/\1 False/' "$f"
  # 위치 추정(AMCL): 찍어준 위치를 덜 믿고 라이다로 더 자주 고치게
  sed -i -E \
    -e 's/^(\s*update_min_d:).*/\1 0.1/' \
    -e 's/^(\s*update_min_a:).*/\1 0.1/' \
    -e 's/^(\s*recovery_alpha_slow:).*/\1 0.001/' \
    -e 's/^(\s*recovery_alpha_fast:).*/\1 0.1/' \
    -e 's/^(\s*laser_likelihood_max_dist:).*/\1 2.0/' \
    "$f"
  # Jazzy bt_navigator는 기본 BT 플러그인을 자동으로 불러옴 → 목록이 있으면 "already registered"로 죽음
  sed -i '/^\s*plugin_lib_names:/,/^$/{/^$/!d}' "$f"
  # Jazzy는 플러그인 이름을 "패키지::클래스"로만 받음 (원본의 "패키지/클래스"는 planner가 죽음)
  # 지도 경로: run_nav.sh가 MAP_YAML을 주면 map_server에 직접 넣음
  [ -n "$MAP_YAML" ] && sed -i -E "s#^(\s*yaml_filename:).*#\1 \"$MAP_YAML\"#" "$f"
  # Jazzy Nav2에 새로 생긴 노드 설정 (원본 SDK 파일엔 없어서 시작이 멈춤)
  cat >> "$f" <<'YAML'

collision_monitor:
  ros__parameters:
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
    state_topic: "collision_monitor_state"
    transform_tolerance: 1.0
    source_timeout: 3.0
    base_shift_correction: True
    stop_pub_timeout: 2.0
    polygons: ["FrontStop"]
    FrontStop:
      type: "polygon"
      points: "[[0.6, 0.25], [0.6, -0.25], [0.38, -0.25], [0.38, 0.25]]"
      action_type: "stop"
      min_points: 4
      visualize: True
      polygon_pub_topic: "collision_front_stop"
      # 꺼 둠: 라이다가 로봇 자기 몸/다리를 장애물로 봐서 계속 "정지"를 걸었음 (리모컨 L2+B로 비상정지)
      enabled: False
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "scan"
      min_height: 0.05
      max_height: 1.0
      enabled: False

docking_server:
  ros__parameters:
    dock_plugins: ["simple_dock"]
    simple_dock:
      plugin: "opennav_docking::SimpleChargingDock"
YAML
  # twist_mux: Jazzy 기본값(use_stamped=true)이면 Nav2/드라이버(Twist)와 형식이 달라 명령이 로봇에 안 감
  local tm="$SDK/config/twist_mux.yaml"
  grep -q use_stamped "$tm" || sed -i '/^\/twist_mux:/,/ros__parameters:/ s/ros__parameters:.*/ros__parameters:\n    use_stamped: false/' "$tm"
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
