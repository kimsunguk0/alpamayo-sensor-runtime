# Sensor Container Configs

Recommended entry points:

- `live_4k_1cam.yaml`: front wide only, `/dev/video2`, `v4l2_preproc`, 10 Hz planner ingest
- `live_4k_2cam.yaml`: front wide + left, `/dev/video1`, `/dev/video7`, yaw/heading CSV diagnostics enabled
- `live_4k_4cam.yaml`: front wide, front tele, left, right, `/dev/video2`, `/dev/video3`, `/dev/video4`, `/dev/video7`
- `sensor_container.example.yaml`: generic documented example/default
- `sensor_container.poseless.yaml`: local bring-up config when pose is intentionally optional

`debug_diagnostics_enabled` adds timing details to `/healthz` and `/latest` response headers. `yaw_heading_log_*` writes local CSV diagnostics under `/workspace/yaw_heading_logs`, which is intentionally git-ignored.

Older `test_*.yaml` files are bring-up snapshots kept for debugging specific hardware situations.
