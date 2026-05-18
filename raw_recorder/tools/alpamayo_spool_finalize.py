#!/usr/bin/env python3
"""Finalize Alpamayo raw-recorder CSV spool files into parquet outputs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import pyarrow as pa
import pyarrow.parquet as pq


CAMERA_ORDER = ["front", "front_tele", "left", "right"]


def parse_bool(value: str) -> bool | None:
    value = value.strip().lower()
    if value == "":
        return None
    if value in {"1", "true", "yes"}:
        return True
    if value in {"0", "false", "no"}:
        return False
    raise ValueError(f"invalid bool value: {value!r}")


def parse_value(value: str, kind: str) -> Any:
    if value == "":
        return None
    if kind == "int":
        return int(value)
    if kind == "float":
        return float(value)
    if kind == "bool":
        return parse_bool(value)
    return value


def read_csv_typed(path: Path, schema: dict[str, str]) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        rows: list[dict[str, Any]] = []
        for row in reader:
            typed = {}
            for key, value in row.items():
                kind = schema.get(key, "str")
                typed[key] = parse_value(value, kind)
            rows.append(typed)
        return rows


def write_parquet(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    table = pa.Table.from_pylist(rows)
    pq.write_table(table, path, compression="zstd")


def nearest_row_id(rows: list[dict[str, Any]], target_ns: int, key: str = "timestamp_utc_ns") -> int:
    timestamps = [row[key] for row in rows]
    lo = 0
    hi = len(timestamps)
    while lo < hi:
        mid = (lo + hi) // 2
        if timestamps[mid] < target_ns:
            lo = mid + 1
        else:
            hi = mid
    candidates = []
    if lo < len(rows):
        candidates.append(rows[lo])
    if lo > 0:
        candidates.append(rows[lo - 1])
    best = min(candidates, key=lambda row: abs(row[key] - target_ns))
    return best["row_id"] if "row_id" in best else best["frame_id"]


def imu_window_row_ids(rows: list[dict[str, Any]], target_ns: int, history_ns: int = 200_000_000) -> tuple[int, int]:
    timestamps = [row["timestamp_utc_ns"] for row in rows]
    start_ns = target_ns - history_ns
    start_idx = 0
    while start_idx < len(rows) and timestamps[start_idx] < start_ns:
        start_idx += 1
    end_idx = start_idx
    while end_idx < len(rows) and timestamps[end_idx] <= target_ns:
        end_idx += 1
    if end_idx == start_idx:
        nearest = nearest_row_id(rows, target_ns)
        return nearest, nearest
    return rows[start_idx]["row_id"], rows[end_idx - 1]["row_id"]


def build_sample_index(session_dir: Path, camera_names: list[str], hz: int = 10) -> None:
    camera_tables = {
        name: pq.read_table(session_dir / "sensors" / f"camera_{name}" / "frames.parquet").to_pylist()
        for name in camera_names
    }
    imu_rows = pq.read_table(session_dir / "sensors" / "imu" / "imu.parquet").to_pylist()
    gnss_rows = pq.read_table(session_dir / "sensors" / "gnss_ins" / "gnss_ins.parquet").to_pylist()

    if any(len(rows) == 0 for rows in camera_tables.values()) or not imu_rows or not gnss_rows:
        return

    start_ns = max(
        max(rows[0]["timestamp_utc_ns"] for rows in camera_tables.values()),
        imu_rows[0]["timestamp_utc_ns"],
        gnss_rows[0]["timestamp_utc_ns"],
    )
    end_ns = min(
        min(rows[-1]["timestamp_utc_ns"] for rows in camera_tables.values()),
        imu_rows[-1]["timestamp_utc_ns"],
        gnss_rows[-1]["timestamp_utc_ns"],
    )

    rows: list[dict[str, Any]] = []
    step_ns = round(1_000_000_000 / hz)
    t0_ns = start_ns
    sample_id = 0
    while t0_ns <= end_ns:
        row = {
            "sample_id": sample_id,
            "t0_utc_ns": t0_ns,
            "gnss_ins_row_id": nearest_row_id(gnss_rows, t0_ns),
        }
        imu_start_id, imu_end_id = imu_window_row_ids(imu_rows, t0_ns)
        row["imu_row_start_id"] = imu_start_id
        row["imu_row_end_id"] = imu_end_id
        for camera_name, camera_rows in camera_tables.items():
            row[f"{camera_name}_frame_id"] = nearest_row_id(camera_rows, t0_ns)
        rows.append(row)
        sample_id += 1
        t0_ns += step_ns

    write_parquet(session_dir / "sample_index_10hz.parquet", rows)


def finalize_session(session_dir: Path, cleanup_spool: bool, build_index: bool) -> None:
    meta = json.loads((session_dir / "session_meta.json").read_text(encoding="utf-8"))
    camera_schema = {
        "frame_id": "int",
        "chunk_id": "int",
        "frame_index_in_chunk": "int",
        "timestamp_utc_ns": "int",
        "timestamp_monotonic_ns": "int",
        "width": "int",
        "height": "int",
        "exposure_time_us": "int",
        "gain": "float",
        "dropped_frame": "bool",
        "trigger_id": "int",
        "seq": "int",
        "flags": "int",
        "file_size_bytes": "int",
        "gst_pts_ns": "int",
    }
    imu_schema = {
        "row_id": "int",
        "timestamp_utc_ns": "int",
        "timestamp_monotonic_ns": "int",
        "ax": "float",
        "ay": "float",
        "az": "float",
        "gx": "float",
        "gy": "float",
        "gz": "float",
        "roll_deg": "float",
        "pitch_deg": "float",
        "yaw_deg": "float",
        "temperature": "float",
        "seq": "int",
    }
    gnss_schema = {
        "row_id": "int",
        "timestamp_utc_ns": "int",
        "timestamp_monotonic_ns": "int",
        "lat": "float",
        "lon": "float",
        "alt": "float",
        "fix_type": "int",
        "num_sats": "int",
        "hdop": "float",
        "vdop": "float",
        "x_local": "float",
        "y_local": "float",
        "z_local": "float",
        "qw": "float",
        "qx": "float",
        "qy": "float",
        "qz": "float",
        "vx": "float",
        "vy": "float",
        "vz": "float",
        "pc": "int",
        "utc_valid": "int",
    }

    cleanup_paths: list[Path] = []
    for camera_name in meta["camera_names"]:
        csv_path = session_dir / "sensors" / f"camera_{camera_name}" / "frames.csv"
        parquet_path = session_dir / "sensors" / f"camera_{camera_name}" / "frames.parquet"
        rows = read_csv_typed(csv_path, camera_schema)
        write_parquet(parquet_path, rows)
        cleanup_paths.append(csv_path)

    imu_csv = session_dir / "sensors" / "imu" / "imu.csv"
    imu_parquet = session_dir / "sensors" / "imu" / "imu.parquet"
    write_parquet(imu_parquet, read_csv_typed(imu_csv, imu_schema))
    cleanup_paths.append(imu_csv)

    gnss_csv = session_dir / "sensors" / "gnss_ins" / "gnss_ins.csv"
    gnss_parquet = session_dir / "sensors" / "gnss_ins" / "gnss_ins.parquet"
    write_parquet(gnss_parquet, read_csv_typed(gnss_csv, gnss_schema))
    cleanup_paths.append(gnss_csv)

    if build_index:
        build_sample_index(session_dir, meta["camera_names"])

    if cleanup_spool:
        for path in cleanup_paths:
            path.unlink(missing_ok=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--cleanup-spool", action="store_true")
    parser.add_argument("--skip-sample-index", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    finalize_session(
        session_dir=args.session_dir,
        cleanup_spool=args.cleanup_spool,
        build_index=not args.skip_sample_index,
    )


if __name__ == "__main__":
    main()
