#!/usr/bin/env bash
set -euo pipefail
cd /workspace

g++ -O2 -std=c++17 -pthread \
  /workspace/sensor_container/src/main.cpp \
  -o /workspace/alpamayo_sensor_container \
  $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 libswscale libavutil) \
  -I/opt/nvidia/deepstream/deepstream/sources/includes \
  -L/opt/nvidia/deepstream/deepstream/lib \
  -lnvbufsurface -lz

echo "built /workspace/alpamayo_sensor_container"
