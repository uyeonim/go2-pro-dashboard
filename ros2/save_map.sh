#!/bin/bash
# 지금까지 만든 SLAM 지도를 프로젝트 maps/ 폴더에 저장 (run_slam.sh가 켜져 있을 때)
#   bash /mnt/c/Users/yeonwoo/robotdog/ros2/save_map.sh lab_map
# 결과: maps/<이름>.pgm(지도 이미지) + .yaml(해상도/원점) + .png(바로 보기용)
set -e
DIR="$(cd "$(dirname "$0")/.." && pwd)"
NAME="${1:-map_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$DIR/maps"
source /opt/ros/jazzy/setup.bash
ros2 run nav2_map_server map_saver_cli -f "$DIR/maps/$NAME" --ros-args -p save_map_timeout:=10.0
python3 -c "import cv2,sys; cv2.imwrite(sys.argv[1]+'.png', cv2.imread(sys.argv[1]+'.pgm', -1))" "$DIR/maps/$NAME" \
  && echo "✅ 저장: maps/$NAME.png (.pgm, .yaml)"
