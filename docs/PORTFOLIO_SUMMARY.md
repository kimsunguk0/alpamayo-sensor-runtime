# Alpamayo Sensor Runtime Portfolio Summary

## Project Overview

This project built a Jetson/Thor-side sensor runtime for autonomous-driving
planner integration. The system has two main execution paths:

- Live sensor container: ingests cameras and GNSS/INS, builds synchronized
  planner-ready samples, and serves them over HTTP.
- Raw recorder: records synchronized 4K camera video, IMU, and GNSS/INS data
  into a replayable dataset layout for offline validation and portfolio demos.

The work started from a DeepStream-based Docker runtime and evolved into a
working sensor-data platform with live planner input generation, latency
diagnostics, yaw/heading analysis, raw dataset capture, and video visualization.

## Runtime Environment

- Base image: NVIDIA DeepStream 8.0 samples multiarch
- Target hardware: Jetson Thor
- Camera path: GMSL/V4L2, UYVY, 4K, 30 FPS source
- Camera roles: `front_wide`, `front_tele`, `left`, `right`
- Navigation sensors: Xsens / GNSS/INS serial stream
- Build outputs:
  - `/workspace/alpamayo_sensor_container`
  - `/workspace/build/bin/alpamayo_raw_recorder`

## Live Sensor Container

The live container prepares synchronized samples for the planner container.

Core responsibilities:

- Open 1, 2, or 4 camera streams from `/dev/video*`.
- Downsample planner input frames to `576x320`.
- Maintain fixed camera ordering and planner camera IDs.
- Parse GNSS/INS data and build ego pose/orientation history.
- Construct Alpamayo-style planner samples at 10 Hz / 15 Hz ingest cadence.
- Serve:
  - `GET /healthz`
  - `GET /latest`

Planner sample contract:

- `image_frames`: `[Cam, 4, 3, 320, 576]`, `uint8`
- `ego_history_xyz`: `[1, 1, 16, 3]`, `float32`
- `ego_history_rot`: `[1, 1, 16, 3, 3]`, `float32`
- `camera_indices`
- `relative_timestamps`
- `absolute_timestamps`
- `t0_us`
- `fixed_delta_seconds`
- `clip_id`
- `camera_order`

Important integration behavior:

- `t0` is selected on a fixed 100 ms grid.
- Camera frames are selected at `t0-300 ms`, `t0-200 ms`, `t0-100 ms`, `t0`.
- Ego history covers 1.5 seconds at 100 ms spacing.
- `/latest` returns the newest valid sample, not a queue.

## Latency Diagnostics

Planner-side logs showed that the largest remaining delay was not planner
overhead but stale sample timestamps. The sensor container was extended with
debug metadata to break the latency into measurable pieces.

Added diagnostics:

- `now_us`
- `latest_t0_us`
- `latest_camera_utc_us`
- `latest_pose_us`
- `newest_possible_us`
- `newest_possible_us - latest_t0_us`
- `now_us - latest_t0_us`
- `sample_built_at_us`
- `sample_published_at_us`
- selected camera frame UTC and timing errors
- `pose_t0_err_us`
- `orientation_t0_err_us`
- camera/pose/orientation pushed wall-clock timestamps

Findings:

- Grid floor was not the main bottleneck in the observed runs.
- Samples were often built around `t0 + 100 ms`.
- CPU resize could take roughly 40-50 ms.
- Sensor push latency was around 55-60 ms.
- Planner polling could add roughly 30 ms of stale time.

Optimizations / changes:

- Added `--debug` mode for optional diagnostics.
- Added push-wall timing fields for camera, pose, and orientation.
- Added builder timing breakdown.
- Improved builder wake behavior using event-style wakeups.
- Added a faster 4K-to-planner frame path for live processing.

## Yaw / Heading Analysis

To validate vehicle orientation quality at low speeds, yaw and GNSS-derived
heading were logged and plotted.

Implemented:

- Sensor yaw/heading CSV logger.
- Planner 10 Hz yaw/heading CSV logger.
- CSV plotting script:
  - `/workspace/tools/diagnostics/plot_yaw_heading_csv.py`

Use case:

- Compare INS yaw against GNSS velocity/position-derived heading.
- Inspect reliability at low speeds around 5-10 km/h.
- Separate sensor yaw quality from planner sample timing.

Observed interpretation:

- RTK GNSS position is accurate, but heading from motion is still speed and
  dynamics dependent.
- At 5-10 km/h, GNSS heading can be usable when moving steadily, but it becomes
  noisy during stops, low acceleration, or poor motion geometry.
- INS yaw is generally the better primary yaw source when properly initialized.

## Raw Recorder

The raw recorder captures persistent datasets for offline replay and validation.

Dataset layout:

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
    gnss_ins/gnss_ins.csv
  sample_index_10hz.parquet
```

Implemented / debugged:

- 4-camera chunked video recording with per-frame CSV indices.
- IMU/GNSS CSV logging.
- Post-recording parquet finalization.
- `sample_index_10hz.parquet` generation.
- Camera warmup gate before clean capture.
- Device mapping validation for current Thor camera layout.
- 4K15 recording mode using `record_fps: 15` while keeping `camera_fps: 30`
  for V4L2 negotiation.

Important recording lesson:

- Several cameras only exposed 4K30 at the V4L2 capability level.
- Setting capture directly to 15 FPS could break negotiation.
- The working solution was:
  - capture at 4K30
  - drop frames in pipeline with `videorate drop-only=true`
  - encode/store at 15 FPS

Recent captured dataset:

```text
/workspace/datasample/2026-06-24-test1
```

This session contains:

- 4 camera streams
- 20 video chunks
- about 5 GB total data
- camera frame CSV/parquet files
- IMU and GNSS/INS CSV/parquet files
- `sample_index_10hz.parquet`

## Docker / Container Transfer

Two integration modes were considered.

Same-host live integration:

- Run camera/sensor container and planner container with host networking.
- Planner reads:

```text
http://127.0.0.1:18080/latest
```

Dataset-copy integration:

- Copy dataset folder from recording machine to local PC or planner machine.
- For a Docker container on the same PC:

```bash
docker cp 2026-06-24-test1 thor_trt:/workspace/datasample/
```

- For a remote Thor machine:

```bash
scp -r 2026-06-24-test1 USER@THOR_IP:/workspace/datasample/
```

## Portfolio Demo Video

A demo renderer was added:

```text
/workspace/tools/dataset/render_camera_path_video.py
```

It creates a single mp4 that combines:

- camera image stream
- GNSS/INS reconstructed route
- ego-relative past trajectory
- ego-relative future trajectory
- speed / yaw overlay

Example:

```bash
/workspace/.venv-alpamayo/bin/python \
  /workspace/tools/dataset/render_camera_path_video.py \
  --dataset-dir /workspace/datasample/2026-06-24-test1 \
  --camera camera_front \
  --output /workspace/portfolio_outputs/2026-06-24-test1_camera_path_overlay.mp4
```

Short preview render:

```bash
/workspace/.venv-alpamayo/bin/python \
  /workspace/tools/dataset/render_camera_path_video.py \
  --dataset-dir /workspace/datasample/2026-06-24-test1 \
  --camera camera_front \
  --max-seconds 10 \
  --output /workspace/portfolio_outputs/preview_10s.mp4
```

## Portfolio Talking Points

- Built a production-style bridge between live vehicle sensors and a
  planner/inference container.
- Defined and implemented a stable planner sample contract.
- Solved timestamp synchronization and latency decomposition problems.
- Implemented real-time 4K camera ingestion and planner-resolution sampling.
- Built raw data recording with replayable video chunks and sensor metadata.
- Debugged hardware-specific camera FPS/device mapping issues on Jetson Thor.
- Added GNSS/INS yaw-heading validation tools.
- Produced portfolio-friendly visualization from recorded data.

