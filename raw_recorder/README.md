# Raw Recorder

Persistent dataset recorder for 4K cameras plus IMU/GNSS.

## Layout

```text
raw_recorder/
  src/alpamayo_raw_recorder.cpp
  config/raw_recorder.yaml
  tools/alpamayo_spool_finalize.py
```

## Build

```bash
/workspace/scripts/build_raw_recorder.sh
```

Output:

```text
/workspace/build/bin/alpamayo_raw_recorder
```

## Run

```bash
/workspace/scripts/run_raw_recorder.sh
```

Or explicitly:

```bash
/workspace/build/bin/alpamayo_raw_recorder \
  --config /workspace/raw_recorder/config/raw_recorder.yaml
```

Recordings are written under `/workspace/datasample/<session_id>/` by default.
