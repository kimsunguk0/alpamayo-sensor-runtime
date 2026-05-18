#!/usr/bin/env bash
set -euo pipefail
CONFIG="${1:-/workspace/raw_recorder/config/raw_recorder.yaml}"
exec /workspace/build/bin/alpamayo_raw_recorder --config "$CONFIG"
