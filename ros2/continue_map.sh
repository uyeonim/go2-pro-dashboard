#!/bin/bash
# 저장해 둔 지도(maps/<이름>.posegraph)를 불러와서 이어서 그리기
# 사용법: run_slam.sh 를 먼저 켜고, 로봇을 "처음 지도 딸 때 출발한 자리·방향"에 둔 뒤
#         bash continue_map.sh lab
set -e
NAME=${1:-lab}
MAPS=/mnt/c/Users/yeonwoo/robotdog/maps
source /opt/ros/jazzy/setup.bash
source ~/go2_ws/install/setup.bash
if [ ! -f "$MAPS/$NAME.posegraph" ]; then
  echo "❌ $MAPS/$NAME.posegraph 없음"; exit 1
fi
# match_type 1 = 처음 출발 지점에서 시작
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph \
  "{filename: '$MAPS/$NAME', match_type: 1}"
echo "✅ $NAME 지도 불러옴. 이제 안 간 곳을 돌고, 끝나면 save_map.sh $NAME 로 다시 저장"
