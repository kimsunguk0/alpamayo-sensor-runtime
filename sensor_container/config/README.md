# Sensor Container Configs

Recommended entry points:

- `live_4k_1cam.yaml`: front wide only, `/dev/video0`
- `live_4k_2cam.yaml`: front wide + front tele, `/dev/video0`, `/dev/video3`
- `live_4k_4cam.yaml`: front wide, front tele, left, right
- `sensor_container.example.yaml`: generic documented example/default
- `sensor_container.poseless.yaml`: local bring-up config when pose is intentionally optional

Older `test_*.yaml` files are bring-up snapshots kept for debugging specific hardware situations.
