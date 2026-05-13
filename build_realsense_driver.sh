#!/usr/bin/env bash
set -e

echo "=== librealsense 자동 빌드 시작 ==="

# 1. 필요한 패키지 설치
echo "=== 필수 패키지 설치 중 (sudo 필요) ==="
sudo apt-get update

sudo apt-get install -y \
    python3-dev python3-numpy \
    libtbb2 libtbb-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libdc1394-dev \
    libssl-dev libusb-1.0-0-dev libudev-dev \
    pkg-config libgtk-3-dev cmake

# 2. 작업 디렉토리 준비
ROOT_DIR="$HOME/realsense"
REPO_DIR="$ROOT_DIR/librealsense"

echo "=== 작업 디렉토리: $ROOT_DIR ==="
mkdir -p "$ROOT_DIR"
cd "$ROOT_DIR"

# 3. librealsense 클론 (이미 있으면 스킵)
if [ -d "$REPO_DIR/.git" ]; then
    echo "=== librealsense 디렉토리 이미 존재: git pull 수행 ==="
    cd "$REPO_DIR"
    git pull
else
    echo "=== librealsense 저장소 클론 ==="
    git clone https://github.com/IntelRealSense/librealsense.git
    cd "$REPO_DIR"
fi

# 4. udev 규칙 설정
echo "=== udev 규칙 설정 (sudo 필요) ==="
sudo ./scripts/setup_udev_rules.sh

# 5. 빌드 디렉토리 생성
echo "=== 빌드 디렉토리 생성 ==="
mkdir -p build
cd build

# 6. CMake 설정 (예제 빌드 + EasyLogging 끔 + OpenGL 끔)
echo "=== CMake 설정 ==="
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=true \
  -DBUILD_EASYLOGGINGPP=FALSE \
  -DBUILD_WITH_OPENGL=OFF

# 7. 컴파일
echo "=== make -j$(nproc) ==="
make -j"$(nproc)"

# 8. 설치
echo "=== sudo make install ==="
sudo make install

echo "=== librealsense 빌드 및 설치 완료! ==="