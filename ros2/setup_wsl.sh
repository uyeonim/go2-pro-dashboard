#!/bin/bash
# Go2 ROS2(Jazzy) SLAM 환경 설치 — WSL Ubuntu 24.04에서 한 번만 실행
#   wsl -d Ubuntu-24.04
#   bash /mnt/c/Users/yeonwoo/robotdog/ros2/setup_wsl.sh
# 로봇 없이 실행해도 돼요. 처음에 sudo 비밀번호를 한 번 물어봐요.
set -e
source /opt/ros/jazzy/setup.bash

echo "[1/5] ROS 패키지 설치 (sudo 비밀번호를 물어봐요)"
sudo apt update
sudo apt install -y python3-pip python3-rosdep python3-colcon-common-extensions git clang portaudio19-dev \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server \
  ros-jazzy-pointcloud-to-laserscan ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-foxglove-bridge \
  ros-jazzy-image-tools ros-jazzy-vision-msgs ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-twist-mux

echo "[2/5] Go2 ROS2 드라이버 소스 (~/go2_ws/src/go2_ros2_sdk)"
mkdir -p ~/go2_ws/src
cd ~/go2_ws/src
[ -d go2_ros2_sdk ] || git clone --recurse-submodules https://github.com/abizovnuralem/go2_ros2_sdk.git

echo "[3/5] 의존 패키지 (rosdep)"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update
cd ~/go2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

echo "[4/6] 파이썬 패키지"
grep -v -E '^(torch|torchvision)' src/go2_ros2_sdk/requirements.txt > /tmp/go2_requirements.txt
python3 -m pip install --user --break-system-packages -r /tmp/go2_requirements.txt

# 사물 인식(coco_detector)용 PyTorch — 약 3GB. 지도 만들기에는 필요 없음.
#   건너뛰려면:  SKIP_TORCH=1 bash setup_wsl.sh
# RTX 50 시리즈(Blackwell)는 CUDA 12.8 이상 빌드가 필요해서 cu128 버전을 받음.
if [ "${SKIP_TORCH:-0}" != "1" ]; then
  echo "[5/6] PyTorch (GPU, 사물 인식용 · 약 3GB라 시간이 걸려요)"
  python3 -m pip install --user --break-system-packages torch torchvision \
    --index-url https://download.pytorch.org/whl/cu128
  python3 -c "import torch; ok=torch.cuda.is_available(); print('  GPU 사용 가능:', ok, '-', torch.cuda.get_device_name(0) if ok else 'CPU로 동작')"
else
  echo "[5/6] PyTorch 건너뜀 (SKIP_TORCH=1)"
fi

echo "[6/6] 빌드"
colcon build --symlink-install

echo
echo "✅ 설치 완료. 로봇 앞에서:"
echo "   지도 만들기:  bash /mnt/c/Users/yeonwoo/robotdog/ros2/run_slam.sh"
echo "   사물 인식:    bash /mnt/c/Users/yeonwoo/robotdog/ros2/run_detector.sh   (run_slam.sh를 켠 상태에서 다른 창)"
