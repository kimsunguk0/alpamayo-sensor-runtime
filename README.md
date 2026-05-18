# Alpamayo Sensor Runtime

This workspace contains two Jetson/Thor pipelines:

- **Raw recorder**: saves persistent 4K camera chunks plus IMU/GNSS CSV/parquet for dataset collection.
- **Live sensor container**: publishes planner-ready 10 Hz samples over HTTP for the planner/inference container.

## Hardware

Current bring-up target:

```text
Compute       Jetson Thor
Cameras       Sensing camera modules, 4K 30 FPS, GMSL/V4L2 path
Camera roles  front_wide, front_tele, left, right
IMU/INS       Xsens MTi-760
GNSS          SMC-3300 receiver data stream
```

The live container expects camera devices such as `/dev/video0` and IMU/GNSS serial input such as `/dev/ttyUSB0`.

## Repository Layout

```text
/workspace
  Dockerfile
  README.md
  raw_recorder/
    src/alpamayo_raw_recorder.cpp
    config/raw_recorder.yaml
    tools/alpamayo_spool_finalize.py
  sensor_container/
    src/
    config/
  scripts/
    build_raw_recorder.sh
    run_raw_recorder.sh
    build_sensor_container.sh
    check_sensor_http.sh
  tools/
    dataset/
    diagnostics/
    prototypes/
  docs/
  datasample/              # local recordings, git-ignored
  build/                   # compiled binaries, git-ignored
```

## Build

```bash
/workspace/scripts/build_raw_recorder.sh
/workspace/scripts/build_sensor_container.sh
```

Build outputs:

```text
/workspace/build/bin/alpamayo_raw_recorder
/workspace/alpamayo_sensor_container
```

## Dataset Recording

Use the raw recorder when you want files saved to disk.

```bash
/workspace/scripts/run_raw_recorder.sh
```

Equivalent explicit command:

```bash
/workspace/build/bin/alpamayo_raw_recorder \
  --config /workspace/raw_recorder/config/raw_recorder.yaml
```

Default output:

```text
/workspace/datasample/<session_id>/
  session_meta.json
  calib/
  sensors/
    camera_front/chunks/*.mkv
    camera_front/frames.csv
    camera_front_tele/chunks/*.mkv
    camera_left/chunks/*.mkv
    camera_right/chunks/*.mkv
    imu/imu.csv
    imu/imu.parquet
    gnss_ins/gnss_ins.csv
    gnss_ins/gnss_ins.parquet
  sample_index_10hz.parquet
```

Important raw-recorder config fields live in:

```text
/workspace/raw_recorder/config/raw_recorder.yaml
```

Camera order for the raw recorder is fixed:

```text
front, front_tele, left, right
```

If `duration_sec: 0`, recording runs until `Ctrl+C`.

## Live Planner Samples

Use the live sensor container when the planner container needs `/latest` HTTP samples.

```bash
/workspace/alpamayo_sensor_container \
  --config /workspace/sensor_container/config/live_4k_2cam.yaml
```

Health check from the same network namespace:

```bash
/workspace/scripts/check_sensor_http.sh http://127.0.0.1:18080
```

Endpoints:

```text
GET /healthz  # JSON health summary
GET /latest   # latest planner NPZ sample, or HTTP 503 if no valid sample exists
```

The live container supports 1 to 4 active cameras. `front_wide` is accepted as an alias for `front`.

Planner-facing camera IDs:

```text
left       -> 0
front      -> 1
right      -> 2
front_tele -> 6
```

Recommended live configs:

```text
/workspace/sensor_container/config/live_4k_1cam.yaml
/workspace/sensor_container/config/live_4k_2cam.yaml
/workspace/sensor_container/config/live_4k_4cam.yaml
```

## Planner Container Networking

If the sensor container runs with host networking, `127.0.0.1` only works from another host-network container.

From a bridge-network planner container, test these and use the URL that succeeds:

```bash
python3 - <<'PY'
import urllib.request
for url in [
    'http://127.0.0.1:18080/healthz',
    'http://172.17.0.1:18080/healthz',
    'http://10.235.142.216:18080/healthz',
]:
    try:
        print(url, urllib.request.urlopen(url, timeout=2).read().decode()[:300])
    except Exception as exc:
        print(url, 'FAIL', exc)
PY
```

## Tools

Dataset helpers:

```text
/workspace/tools/dataset/
```

Camera/GStreamer/V4L2/IMU bring-up snippets:

```text
/workspace/tools/diagnostics/
```

Old exploratory scripts:

```text
/workspace/tools/prototypes/
```

## Git Hygiene

Commit source, configs, scripts, docs, and tools. Do not commit local recordings, logs, generated binaries, caches, or virtualenvs.

Ignored by default:

```text
datasample/
build/
*.mkv, *.mp4, *.parquet
*.log
.venv*/
__pycache__/
```
