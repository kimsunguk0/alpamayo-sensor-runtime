#!/usr/bin/env bash
set -euo pipefail
cd /workspace

mkdir -p /workspace/build/bin

g++ -O2 -std=c++17 -pthread \
  /workspace/raw_recorder/src/alpamayo_raw_recorder.cpp \
  -o /workspace/build/bin/alpamayo_raw_recorder \
  $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-video-1.0)

echo "built /workspace/build/bin/alpamayo_raw_recorder"
