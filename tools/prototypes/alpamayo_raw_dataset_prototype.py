#!/usr/bin/env python3
"""Single-file Alpamayo raw dataset prototype with dummy data and self-test."""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pyarrow as pa
import pyarrow.parquet as pq
from PIL import Image, ImageDraw, ImageFont


CAMERA_ORDER = ["front", "front_tele", "front_left", "front_right"]
DEFAULT_OUTPUT_ROOT = Path("/workspace/datasample")
CAMERA_COLORS = {
    "front_left": (210, 84, 84),
    "front": (78, 145, 235),
    "front_right": (68, 176, 119),
    "front_tele": (215, 174, 64),
}


@dataclass(frozen=True)
class SessionConfig:
    dataset_root: Path
    session_id: str
    vehicle_id: str = "dummy_vehicle_01"
    timezone: str = "UTC"
    camera_fps: int = 30
    imu_hz: int = 100
    gnss_ins_hz: int = 10
    duration_sec: int = 6
    chunk_sec: int = 2
    width: int = 320
    height: int = 180
    start_utc_ns: int = 1774612800000000000
    start_monotonic_ns: int = 5000000000000

    @property
    def session_dir(self) -> Path:
        return self.dataset_root / self.session_id

    @property
    def frames_per_chunk(self) -> int:
        return self.camera_fps * self.chunk_sec


def ns_offset_for_index(index: int, hz: int) -> int:
    return round(index * 1_000_000_000 / hz)


def ensure_clean_dir(path: Path) -> None:
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def write_parquet(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    table = pa.Table.from_pylist(rows)
    pq.write_table(table, path, compression="zstd")


def run_checked(cmd: list[str], cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        check=True,
        text=True,
        capture_output=True,
    )


def ffmpeg_encode_png_sequence(input_pattern: Path, fps: int, output_path: Path) -> None:
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-framerate",
        str(fps),
        "-i",
        str(input_pattern),
        "-c:v",
        "ffv1",
        "-level",
        "3",
        str(output_path),
    ]
    run_checked(cmd)


def ffprobe_frame_count(video_path: Path) -> int:
    cmd = [
        "ffprobe",
        "-v",
        "error",
        "-count_frames",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=nb_read_frames",
        "-of",
        "default=noprint_wrappers=1:nokey=1",
        str(video_path),
    ]
    result = run_checked(cmd)
    return int(result.stdout.strip())


def ffmpeg_decode_check(video_path: Path) -> None:
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-i",
        str(video_path),
        "-f",
        "null",
        "-",
    ]
    run_checked(cmd)


def camera_dir(session_dir: Path, camera_name: str) -> Path:
    return session_dir / "sensors" / f"camera_{camera_name}"


def render_dummy_frame(
    camera_name: str,
    frame_id: int,
    timestamp_utc_ns: int,
    width: int,
    height: int,
) -> Image.Image:
    base = CAMERA_COLORS[camera_name]
    image = Image.new("RGB", (width, height), base)
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    for y in range(0, height, 12):
        shade = int(25 * math.sin((frame_id + y) * 0.15))
        color = tuple(max(0, min(255, c + shade)) for c in base)
        draw.rectangle((0, y, width, min(height - 1, y + 6)), fill=color)

    tx = frame_id * 9
    draw.rectangle((tx % width, height - 22, (tx % width) + 28, height - 6), fill=(245, 245, 245))
    draw.text((10, 10), f"{camera_name}", fill=(255, 255, 255), font=font)
    draw.text((10, 28), f"frame_id={frame_id}", fill=(255, 255, 255), font=font)
    draw.text((10, 46), f"utc_ns={timestamp_utc_ns}", fill=(255, 255, 255), font=font)
    return image


def build_session_meta(config: SessionConfig) -> dict[str, Any]:
    return {
        "session_id": config.session_id,
        "vehicle_id": config.vehicle_id,
        "timezone": config.timezone,
        "time_sync": {
            "master_clock": "utc",
            "utc_source": "dummy_gnss",
            "store_monotonic": True,
        },
        "camera_names": CAMERA_ORDER,
        "camera_fps": config.camera_fps,
        "imu_hz": config.imu_hz,
        "gnss_ins_hz": config.gnss_ins_hz,
    }


def build_camera_intrinsics(config: SessionConfig) -> dict[str, Any]:
    intrinsics = {}
    for name in CAMERA_ORDER:
        intrinsics[f"camera_{name}"] = {
            "image_size": [config.width, config.height],
            "focal_length_px": None,
            "principal_point_px": None,
            "distortion_model": None,
        }
    return intrinsics


def build_sensor_mounts() -> dict[str, Any]:
    return {
        "camera_front_left": {
            "description": "front-left windshield area",
            "nominal_fov_deg": 120,
        },
        "camera_front": {
            "description": "front center",
            "nominal_fov_deg": 120,
        },
        "camera_front_right": {
            "description": "front-right windshield area",
            "nominal_fov_deg": 120,
        },
        "camera_front_tele": {
            "description": "front tele",
            "nominal_fov_deg": 30,
        },
        "imu": {"description": "vehicle body mounted"},
        "gnss_ins": {"description": "roof antenna or fused navigation output"},
    }


def generate_camera_stream(config: SessionConfig, camera_name: str) -> list[dict[str, Any]]:
    cam_path = camera_dir(config.session_dir, camera_name)
    chunks_dir = cam_path / "chunks"
    chunks_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, Any]] = []
    total_frames = config.camera_fps * config.duration_sec
    frames_per_chunk = config.frames_per_chunk

    for chunk_id, chunk_start in enumerate(range(0, total_frames, frames_per_chunk)):
        chunk_end = min(total_frames, chunk_start + frames_per_chunk)
        chunk_path = chunks_dir / f"chunk_{chunk_id:04d}.mkv"

        with tempfile.TemporaryDirectory(prefix=f"{camera_name}_{chunk_id:04d}_") as temp_dir_name:
            temp_dir = Path(temp_dir_name)
            for local_index, frame_id in enumerate(range(chunk_start, chunk_end)):
                offset_ns = ns_offset_for_index(frame_id, config.camera_fps)
                timestamp_utc_ns = config.start_utc_ns + offset_ns
                timestamp_monotonic_ns = config.start_monotonic_ns + offset_ns

                image = render_dummy_frame(
                    camera_name=camera_name,
                    frame_id=frame_id,
                    timestamp_utc_ns=timestamp_utc_ns,
                    width=config.width,
                    height=config.height,
                )
                image.save(temp_dir / f"frame_{local_index:06d}.png")

                rows.append(
                    {
                        "frame_id": frame_id,
                        "chunk_id": chunk_id,
                        "frame_index_in_chunk": local_index,
                        "timestamp_utc_ns": timestamp_utc_ns,
                        "timestamp_monotonic_ns": timestamp_monotonic_ns,
                        "width": config.width,
                        "height": config.height,
                        "exposure_time_us": 8000 + (frame_id % 7) * 100,
                        "gain": round(1.0 + (frame_id % 5) * 0.1, 3),
                        "dropped_frame": False,
                        "trigger_id": frame_id,
                    }
                )

            ffmpeg_encode_png_sequence(temp_dir / "frame_%06d.png", config.camera_fps, chunk_path)

    write_parquet(cam_path / "frames.parquet", rows)
    return rows


def generate_imu_rows(config: SessionConfig) -> list[dict[str, Any]]:
    total_rows = config.imu_hz * config.duration_sec
    rows: list[dict[str, Any]] = []
    for row_id in range(total_rows):
        t_ns = ns_offset_for_index(row_id, config.imu_hz)
        t_sec = t_ns / 1_000_000_000
        rows.append(
            {
                "row_id": row_id,
                "timestamp_utc_ns": config.start_utc_ns + t_ns,
                "timestamp_monotonic_ns": config.start_monotonic_ns + t_ns,
                "ax": round(0.35 * math.sin(2.0 * math.pi * 0.9 * t_sec), 6),
                "ay": round(0.22 * math.cos(2.0 * math.pi * 0.6 * t_sec), 6),
                "az": round(9.81 + 0.05 * math.sin(2.0 * math.pi * 0.2 * t_sec), 6),
                "gx": round(0.01 * math.sin(2.0 * math.pi * 0.4 * t_sec), 6),
                "gy": round(0.015 * math.cos(2.0 * math.pi * 0.5 * t_sec), 6),
                "gz": round(0.02 * math.sin(2.0 * math.pi * 0.3 * t_sec), 6),
                "temperature": round(36.5 + 0.02 * t_sec, 3),
                "status": "ok",
                "seq": row_id,
            }
        )
    write_parquet(config.session_dir / "sensors" / "imu" / "imu.parquet", rows)
    return rows


def generate_gnss_rows(config: SessionConfig) -> list[dict[str, Any]]:
    total_rows = config.gnss_ins_hz * config.duration_sec
    rows: list[dict[str, Any]] = []
    base_lat = 37.4219999
    base_lon = -122.0840575
    for row_id in range(total_rows):
        t_ns = ns_offset_for_index(row_id, config.gnss_ins_hz)
        t_sec = t_ns / 1_000_000_000
        x_local = 2.5 * t_sec
        y_local = 0.4 * math.sin(0.8 * t_sec)
        yaw = 0.02 * math.sin(0.2 * t_sec)
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        rows.append(
            {
                "row_id": row_id,
                "timestamp_utc_ns": config.start_utc_ns + t_ns,
                "timestamp_monotonic_ns": config.start_monotonic_ns + t_ns,
                "lat": base_lat + x_local * 1e-5,
                "lon": base_lon + y_local * 1e-5,
                "alt": 12.4 + 0.05 * math.sin(0.1 * t_sec),
                "fix_type": 4,
                "num_sats": 17,
                "hdop": 0.7,
                "vdop": 1.0,
                "x_local": round(x_local, 6),
                "y_local": round(y_local, 6),
                "z_local": 0.0,
                "qw": round(qw, 9),
                "qx": 0.0,
                "qy": 0.0,
                "qz": round(qz, 9),
                "vx": 2.5,
                "vy": round(0.32 * math.cos(0.8 * t_sec), 6),
                "vz": 0.0,
                "ins_status": "fused_ok",
            }
        )
    write_parquet(config.session_dir / "sensors" / "gnss_ins" / "gnss_ins.parquet", rows)
    return rows


def generate_dummy_session(config: SessionConfig) -> dict[str, Any]:
    ensure_clean_dir(config.session_dir)

    write_json(config.session_dir / "session_meta.json", build_session_meta(config))
    write_json(config.session_dir / "calib" / "camera_intrinsics.json", build_camera_intrinsics(config))
    write_json(config.session_dir / "calib" / "sensor_mounts_placeholder.json", build_sensor_mounts())

    camera_rows = {}
    for camera_name in CAMERA_ORDER:
        camera_rows[camera_name] = generate_camera_stream(config, camera_name)

    imu_rows = generate_imu_rows(config)
    gnss_rows = generate_gnss_rows(config)

    summary = {
        "session_dir": str(config.session_dir),
        "camera_frames_per_camera": len(next(iter(camera_rows.values()))),
        "imu_rows": len(imu_rows),
        "gnss_rows": len(gnss_rows),
    }
    return summary


def parquet_rows(path: Path) -> list[dict[str, Any]]:
    table = pq.read_table(path)
    return table.to_pylist()


def assert_required_columns(rows: list[dict[str, Any]], required: list[str], label: str) -> None:
    if not rows:
        raise ValueError(f"{label}: no rows found")
    row_keys = set(rows[0].keys())
    missing = [column for column in required if column not in row_keys]
    if missing:
        raise ValueError(f"{label}: missing required columns: {missing}")


def assert_strictly_increasing(values: list[int], label: str) -> None:
    if any(curr <= prev for prev, curr in zip(values, values[1:])):
        raise ValueError(f"{label}: values are not strictly increasing")


def compute_rate_hz(rows: list[dict[str, Any]], timestamp_column: str) -> float:
    if len(rows) < 2:
        return 0.0
    dt_ns = rows[-1][timestamp_column] - rows[0][timestamp_column]
    if dt_ns <= 0:
        return 0.0
    return (len(rows) - 1) * 1_000_000_000 / dt_ns


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


def build_sample_index(session_dir: Path, hz: int = 10) -> Path:
    camera_tables = {
        name: parquet_rows(camera_dir(session_dir, name) / "frames.parquet") for name in CAMERA_ORDER
    }
    imu_rows = parquet_rows(session_dir / "sensors" / "imu" / "imu.parquet")
    gnss_rows = parquet_rows(session_dir / "sensors" / "gnss_ins" / "gnss_ins.parquet")

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

    step_ns = round(1_000_000_000 / hz)
    rows: list[dict[str, Any]] = []
    sample_id = 0
    t0_ns = start_ns
    while t0_ns <= end_ns:
        row = {
            "sample_id": sample_id,
            "t0_utc_ns": t0_ns,
            "gnss_ins_row_id": nearest_row_id(gnss_rows, t0_ns),
        }
        imu_start_id, imu_end_id = imu_window_row_ids(imu_rows, t0_ns)
        row["imu_row_start_id"] = imu_start_id
        row["imu_row_end_id"] = imu_end_id
        for camera_name, cam_rows in camera_tables.items():
            row[f"{camera_name}_frame_id"] = nearest_row_id(cam_rows, t0_ns)
        rows.append(row)
        sample_id += 1
        t0_ns += step_ns

    output_path = session_dir / "sample_index_10hz.parquet"
    write_parquet(output_path, rows)
    return output_path


def validate_camera_stream(session_dir: Path, camera_name: str, expected_fps: int) -> dict[str, Any]:
    path = camera_dir(session_dir, camera_name) / "frames.parquet"
    rows = parquet_rows(path)
    assert_required_columns(
        rows,
        [
            "frame_id",
            "chunk_id",
            "frame_index_in_chunk",
            "timestamp_utc_ns",
            "timestamp_monotonic_ns",
            "width",
            "height",
        ],
        f"camera_{camera_name}",
    )

    frame_ids = [row["frame_id"] for row in rows]
    utc_ns = [row["timestamp_utc_ns"] for row in rows]
    monotonic_ns = [row["timestamp_monotonic_ns"] for row in rows]
    if frame_ids != list(range(len(rows))):
        raise ValueError(f"camera_{camera_name}: frame_id gap detected")
    assert_strictly_increasing(utc_ns, f"camera_{camera_name}.timestamp_utc_ns")
    assert_strictly_increasing(monotonic_ns, f"camera_{camera_name}.timestamp_monotonic_ns")

    frames_by_chunk: dict[int, list[dict[str, Any]]] = {}
    for row in rows:
        frames_by_chunk.setdefault(row["chunk_id"], []).append(row)

    for chunk_id, chunk_rows in frames_by_chunk.items():
        expected_index = list(range(len(chunk_rows)))
        actual_index = [row["frame_index_in_chunk"] for row in chunk_rows]
        if actual_index != expected_index:
            raise ValueError(f"camera_{camera_name}: chunk {chunk_id} frame_index_in_chunk mismatch")
        video_path = camera_dir(session_dir, camera_name) / "chunks" / f"chunk_{chunk_id:04d}.mkv"
        if not video_path.exists():
            raise FileNotFoundError(video_path)
        ffmpeg_decode_check(video_path)
        frame_count = ffprobe_frame_count(video_path)
        if frame_count != len(chunk_rows):
            raise ValueError(
                f"camera_{camera_name}: chunk {chunk_id} frame count mismatch "
                f"(index={len(chunk_rows)} decode={frame_count})"
            )

    rate_hz = compute_rate_hz(rows, "timestamp_utc_ns")
    if not (expected_fps * 0.95 <= rate_hz <= expected_fps * 1.05):
        raise ValueError(f"camera_{camera_name}: unexpected rate {rate_hz:.3f} Hz")

    return {
        "rows": len(rows),
        "chunks": len(frames_by_chunk),
        "rate_hz": round(rate_hz, 3),
    }


def validate_series(path: Path, label: str, expected_hz: int, required_columns: list[str]) -> dict[str, Any]:
    rows = parquet_rows(path)
    assert_required_columns(rows, required_columns, label)
    utc_ns = [row["timestamp_utc_ns"] for row in rows]
    monotonic_ns = [row["timestamp_monotonic_ns"] for row in rows]
    assert_strictly_increasing(utc_ns, f"{label}.timestamp_utc_ns")
    assert_strictly_increasing(monotonic_ns, f"{label}.timestamp_monotonic_ns")
    rate_hz = compute_rate_hz(rows, "timestamp_utc_ns")
    if not (expected_hz * 0.95 <= rate_hz <= expected_hz * 1.05):
        raise ValueError(f"{label}: unexpected rate {rate_hz:.3f} Hz")
    return {"rows": len(rows), "rate_hz": round(rate_hz, 3)}


def validate_dataset(session_dir: Path) -> dict[str, Any]:
    meta = json.loads((session_dir / "session_meta.json").read_text(encoding="utf-8"))
    summary = {
        "session_id": meta["session_id"],
        "time_sync_master_clock": meta["time_sync"]["master_clock"],
        "cameras": {},
    }

    for camera_name in meta["camera_names"]:
        summary["cameras"][camera_name] = validate_camera_stream(
            session_dir=session_dir,
            camera_name=camera_name,
            expected_fps=meta["camera_fps"],
        )

    summary["imu"] = validate_series(
        session_dir / "sensors" / "imu" / "imu.parquet",
        "imu",
        meta["imu_hz"],
        [
            "timestamp_utc_ns",
            "timestamp_monotonic_ns",
            "ax",
            "ay",
            "az",
            "gx",
            "gy",
            "gz",
        ],
    )
    summary["gnss_ins"] = validate_series(
        session_dir / "sensors" / "gnss_ins" / "gnss_ins.parquet",
        "gnss_ins",
        meta["gnss_ins_hz"],
        ["timestamp_utc_ns", "timestamp_monotonic_ns", "lat", "lon", "alt"],
    )

    sample_index_path = session_dir / "sample_index_10hz.parquet"
    if sample_index_path.exists():
        sample_index_rows = parquet_rows(sample_index_path)
        summary["sample_index_10hz"] = {"rows": len(sample_index_rows)}

    return summary


def run_self_test(output_root: Path, session_id: str, duration_sec: int, chunk_sec: int) -> dict[str, Any]:
    config = SessionConfig(
        dataset_root=output_root,
        session_id=session_id,
        duration_sec=duration_sec,
        chunk_sec=chunk_sec,
    )
    create_summary = generate_dummy_session(config)
    sample_index_path = build_sample_index(config.session_dir)
    validation_summary = validate_dataset(config.session_dir)
    return {
        "create_summary": create_summary,
        "sample_index_path": str(sample_index_path),
        "validation_summary": validation_summary,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    demo_parser = subparsers.add_parser("demo", help="Create a dummy Alpamayo-style raw dataset session")
    demo_parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    demo_parser.add_argument("--session-id", default="session_20260327_run_0001")
    demo_parser.add_argument("--duration-sec", type=int, default=6)
    demo_parser.add_argument("--chunk-sec", type=int, default=2)

    validate_parser = subparsers.add_parser("validate", help="Validate an existing raw dataset session")
    validate_parser.add_argument("session_dir", type=Path)

    self_test_parser = subparsers.add_parser("self-test", help="Generate, index, and validate a dummy session")
    self_test_parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    self_test_parser.add_argument("--session-id", default="session_20260327_run_0001")
    self_test_parser.add_argument("--duration-sec", type=int, default=6)
    self_test_parser.add_argument("--chunk-sec", type=int, default=2)

    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.command == "demo":
        config = SessionConfig(
            dataset_root=args.output_root,
            session_id=args.session_id,
            duration_sec=args.duration_sec,
            chunk_sec=args.chunk_sec,
        )
        payload = generate_dummy_session(config)
    elif args.command == "validate":
        payload = validate_dataset(args.session_dir)
    else:
        payload = run_self_test(
            output_root=args.output_root,
            session_id=args.session_id,
            duration_sec=args.duration_sec,
            chunk_sec=args.chunk_sec,
        )

    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
