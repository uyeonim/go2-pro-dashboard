#!/bin/bash
# 카메라 영상에서 사물 인식 (사람, 의자, 컵 등 COCO 80종) — run_slam.sh를 켠 상태에서 다른 WSL 창에서 실행
#   bash /mnt/c/Users/yeonwoo/robotdog/ros2/run_detector.sh          # 기본 신뢰도 0.6
#   bash /mnt/c/Users/yeonwoo/robotdog/ros2/run_detector.sh 0.8      # 더 확실한 것만
# 결과: /detected_objects 토픽 + 상자가 그려진 영상 창(/annotated_image)
# 처음 실행할 때 인식 모델(약 70MB)을 내려받아요.
set -e
THRESH="${1:-0.6}"
source /opt/ros/jazzy/setup.bash
source ~/go2_ws/install/setup.bash

DEVICE=cpu
python3 -c "import torch,sys; sys.exit(0 if torch.cuda.is_available() else 1)" 2>/dev/null && DEVICE=cuda
echo "사물 인식 시작 (장치: $DEVICE, 신뢰도 기준: $THRESH)"
[ "$DEVICE" = cpu ] && echo "⚠ GPU를 못 찾아서 CPU로 돌아요. 느릴 수 있어요."

# 상자가 그려진 영상 창을 함께 띄우고, 이 스크립트가 끝나면 같이 닫음
ros2 run image_tools showimage --ros-args -r /image:=/annotated_image &
VIEW=$!
trap 'kill $VIEW 2>/dev/null' EXIT

ros2 run coco_detector coco_detector_node --ros-args \
  -p device:=$DEVICE -p detection_threshold:=$THRESH -p publish_annotated_image:=true
