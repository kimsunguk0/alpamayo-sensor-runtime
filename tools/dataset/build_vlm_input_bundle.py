import argparse
import csv
import json
import math
import shutil
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


DATASET_DIR = Path("/home/nvidia/datasample/2025-03-31-test2")
IMAGE_FRAME_OFFSETS_NS = [-300_000_000, -200_000_000, -100_000_000, 0]
HISTORY_DT_NS = 100_000_000
HISTORY_LEN = 16
FUTURE_DT_NS = 100_000_000
FUTURE_DURATION_NS = 6_400_000_000

# Camera order is fixed by the runtime pipeline.
CAMERA_ORDER = [
    ("camera_front", 0, "front"),
    ("camera_front_tele", 1, "front_tele"),
    ("camera_left", 2, "left"),
    ("camera_right", 3, "right"),
]
CAMERA_DISPLAY_NAMES = {
    "front": "front wide",
    "front_tele": "front tele",
    "left": "left",
    "right": "right",
}


@dataclass
class CameraPick:
    sensor_name: str
    camera_id: int
    semantic_name: str
    target_time_ns: int
    selected_time_ns: int
    frame_index_in_chunk: int
    output_path: str


def get_time_field(time_base: str) -> str:
    if time_base not in {"monotonic", "utc"}:
        raise ValueError(f"Unsupported time base: {time_base}")
    return f"timestamp_{time_base}_ns"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--chunk-id", type=int, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--dataset-dir", type=Path, default=DATASET_DIR)
    parser.add_argument("--t0-margin-ns", type=int, default=200_000_000)
    parser.add_argument("--t0-seconds-from-chunk-start", type=float, default=None)
    parser.add_argument("--time-base", choices=["monotonic", "utc"], default="monotonic")
    return parser.parse_args()


def load_chunk_frames(dataset_dir: Path, sensor_name: str, chunk_id: int, time_base: str) -> list[dict]:
    frames_path = dataset_dir / "sensors" / sensor_name / "frames.csv"
    time_field = get_time_field(time_base)
    rows = []
    with frames_path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if int(row["chunk_id"]) != chunk_id:
                continue
            row["frame_index_in_chunk"] = int(row["frame_index_in_chunk"])
            row["timestamp_monotonic_ns"] = int(row["timestamp_monotonic_ns"])
            row["timestamp_utc_ns"] = int(row["timestamp_utc_ns"])
            rows.append(row)
    if not rows:
        raise RuntimeError(f"No rows found for {sensor_name} chunk {chunk_id}")
    rows.sort(key=lambda row: row[time_field])
    return rows


def pick_common_t0(
    chunk_frames: dict[str, list[dict]],
    t0_margin_ns: int,
    t0_seconds_from_chunk_start: float | None,
    time_base: str,
) -> tuple[int, dict]:
    time_field = get_time_field(time_base)
    common_start = max(rows[0][time_field] for rows in chunk_frames.values())
    common_end = min(rows[-1][time_field] for rows in chunk_frames.values())
    latest_safe_t0 = common_end - FUTURE_DURATION_NS - t0_margin_ns
    if t0_seconds_from_chunk_start is None:
        chosen_t0 = latest_safe_t0
    else:
        chosen_t0 = common_start + int(round(t0_seconds_from_chunk_start * 1e9))
    history_start = chosen_t0 - (HISTORY_LEN - 1) * HISTORY_DT_NS
    future_end = chosen_t0 + FUTURE_DURATION_NS
    if history_start < common_start:
        raise RuntimeError("Not enough overlap to build past history and 6.4s future GT")
    if future_end > common_end:
        raise RuntimeError("Chosen t0 does not leave enough room for 6.4s future GT")
    info = {
        f"common_start_{time_base}_ns": int(common_start),
        f"common_end_{time_base}_ns": int(common_end),
        f"latest_safe_t0_{time_base}_ns": int(latest_safe_t0),
        f"chosen_t0_{time_base}_ns": int(chosen_t0),
    }
    return chosen_t0, info


def nearest_row(rows: list[dict], target_time_ns: int, time_base: str) -> dict:
    time_field = get_time_field(time_base)
    timestamps = np.asarray([row[time_field] for row in rows], dtype=np.int64)
    index = int(np.argmin(np.abs(timestamps - target_time_ns)))
    return rows[index]


def extract_frame_png(video_path: Path, frame_index: int, output_path: Path) -> None:
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video: {video_path}")
    try:
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
        ok, frame_bgr = cap.read()
        if not ok or frame_bgr is None:
            raise RuntimeError(f"Failed to read frame {frame_index} from {video_path}")
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        Image.fromarray(frame_rgb).save(output_path)
    finally:
        cap.release()


def save_camera_frame_variants(
    video_path: Path,
    frame_index: int,
    images_dir: Path,
    camera_id: int,
    semantic_name: str,
    frame_slot: int,
) -> tuple[Path, Path]:
    semantic_path = images_dir / f"{semantic_name}_f{frame_slot}.png"
    cam_id_path = images_dir / f"cam{camera_id}_f{frame_slot}.png"
    extract_frame_png(video_path, frame_index, semantic_path)
    shutil.copyfile(semantic_path, cam_id_path)
    return semantic_path, cam_id_path


def load_gnss_valid(dataset_dir: Path, time_base: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    path = dataset_dir / "sensors" / "gnss_ins" / "gnss_ins.csv"
    time_field = get_time_field(time_base)
    times = []
    lla = []
    velocities = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if not row["lat"] or not row["lon"] or not row["alt"]:
                continue
            times.append(int(row[time_field]))
            lla.append([float(row["lat"]), float(row["lon"]), float(row["alt"])])
            velocities.append([float(row["vx"]), float(row["vy"]), float(row["vz"])])
    order = np.argsort(times)
    times_arr = np.asarray(times, dtype=np.int64)[order]
    lla_arr = np.asarray(lla, dtype=np.float64)[order]
    vel_arr = np.asarray(velocities, dtype=np.float64)[order]
    unique_times, unique_indices = np.unique(times_arr, return_index=True)
    return unique_times, lla_arr[unique_indices], vel_arr[unique_indices]


def load_imu_valid(dataset_dir: Path, time_base: str) -> tuple[np.ndarray, np.ndarray]:
    path = dataset_dir / "sensors" / "imu" / "imu.csv"
    time_field = get_time_field(time_base)
    times = []
    accel = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if not row["ax"] or not row["ay"] or not row["az"]:
                continue
            times.append(int(row[time_field]))
            accel.append([float(row["ax"]), float(row["ay"]), float(row["az"])])
    order = np.argsort(times)
    times_arr = np.asarray(times, dtype=np.int64)[order]
    accel_arr = np.asarray(accel, dtype=np.float64)[order]
    unique_times, unique_indices = np.unique(times_arr, return_index=True)
    accel_arr = accel_arr[unique_indices]
    kernel = np.ones(25, dtype=np.float64) / 25.0
    accel_smooth = np.vstack(
        [np.convolve(accel_arr[:, axis], kernel, mode="same") for axis in range(3)]
    ).T
    return unique_times, accel_smooth


def interpolate_series(query_times: np.ndarray, sample_times: np.ndarray, sample_values: np.ndarray) -> np.ndarray:
    if query_times[0] < sample_times[0] or query_times[-1] > sample_times[-1]:
        raise RuntimeError("Interpolation query is outside available sample range")
    result = np.empty((len(query_times), sample_values.shape[1]), dtype=np.float64)
    for axis in range(sample_values.shape[1]):
        result[:, axis] = np.interp(query_times, sample_times, sample_values[:, axis])
    return result


def lla_to_ecef(lat_deg: np.ndarray, lon_deg: np.ndarray, alt_m: np.ndarray) -> np.ndarray:
    a = 6378137.0
    e2 = 6.69437999014e-3
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)
    N = a / np.sqrt(1.0 - e2 * sin_lat * sin_lat)
    x = (N + alt_m) * cos_lat * cos_lon
    y = (N + alt_m) * cos_lat * sin_lon
    z = (N * (1.0 - e2) + alt_m) * sin_lat
    return np.column_stack([x, y, z])


def ecef_to_enu(ecef_xyz: np.ndarray, ref_lla: np.ndarray) -> np.ndarray:
    ref_lat = math.radians(ref_lla[0])
    ref_lon = math.radians(ref_lla[1])
    ref_ecef = lla_to_ecef(
        np.array([ref_lla[0]], dtype=np.float64),
        np.array([ref_lla[1]], dtype=np.float64),
        np.array([ref_lla[2]], dtype=np.float64),
    )[0]
    dx = ecef_xyz - ref_ecef
    sin_lat = math.sin(ref_lat)
    cos_lat = math.cos(ref_lat)
    sin_lon = math.sin(ref_lon)
    cos_lon = math.cos(ref_lon)
    rot = np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=np.float64,
    )
    return dx @ rot.T


def rotation_matrix_from_ypr(yaw: float, pitch: float, roll: float) -> np.ndarray:
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    return rz @ ry @ rx


def build_pose_and_future(
    dataset_dir: Path,
    t0_ns: int,
    time_base: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    history_times = np.array(
        [t0_ns - (HISTORY_LEN - 1 - i) * HISTORY_DT_NS for i in range(HISTORY_LEN)],
        dtype=np.int64,
    )
    future_times = np.array(
        [t0_ns + (i + 1) * FUTURE_DT_NS for i in range(FUTURE_DURATION_NS // FUTURE_DT_NS)],
        dtype=np.int64,
    )
    full_times = np.concatenate([history_times, future_times])

    gnss_times, gnss_lla, gnss_vel = load_gnss_valid(dataset_dir, time_base)
    imu_times, imu_accel = load_imu_valid(dataset_dir, time_base)

    gnss_positions_ecef = lla_to_ecef(gnss_lla[:, 0], gnss_lla[:, 1], gnss_lla[:, 2])
    ref_index = int(np.argmin(np.abs(gnss_times - t0_ns)))
    ref_lla = gnss_lla[ref_index]
    gnss_positions_enu = ecef_to_enu(gnss_positions_ecef, ref_lla)

    positions_world = interpolate_series(full_times, gnss_times, gnss_positions_enu)
    velocity_world = interpolate_series(full_times, gnss_times, gnss_vel)
    accel_body = interpolate_series(history_times, imu_times, imu_accel)

    speed_xy = np.linalg.norm(velocity_world[:, :2], axis=1)
    position_grad = np.gradient(positions_world[:, :2], FUTURE_DT_NS / 1e9, axis=0)
    # ENU uses x=east, y=north, while the decoder expects x=forward in the
    # ego-local frame. Yaw therefore needs the standard atan2(y, x) convention
    # so that the t0 heading rotates onto +x.
    heading_from_pos = np.unwrap(np.arctan2(position_grad[:, 1], position_grad[:, 0]))
    heading_from_vel = np.unwrap(np.arctan2(velocity_world[:, 1], velocity_world[:, 0]))
    yaw_series = np.where(speed_xy > 0.5, heading_from_vel, heading_from_pos)

    ax = accel_body[:, 0]
    ay = accel_body[:, 1]
    az = accel_body[:, 2]
    roll_history = np.arctan2(ay, az)
    pitch_history = np.arctan2(-ax, np.sqrt(ay * ay + az * az))
    if len(roll_history) > 1:
        roll_future = np.full(len(future_times), roll_history[-1], dtype=np.float64)
        pitch_future = np.full(len(future_times), pitch_history[-1], dtype=np.float64)
        roll_series = np.concatenate([roll_history, roll_future])
        pitch_series = np.concatenate([pitch_history, pitch_future])
    else:
        roll_series = np.zeros(len(full_times), dtype=np.float64)
        pitch_series = np.zeros(len(full_times), dtype=np.float64)

    world_rotations = np.stack(
        [
            rotation_matrix_from_ypr(float(yaw), float(pitch), float(roll))
            for yaw, pitch, roll in zip(yaw_series, pitch_series, roll_series)
        ],
        axis=0,
    )

    p0 = positions_world[HISTORY_LEN - 1]
    r0 = world_rotations[HISTORY_LEN - 1]
    positions_local = np.stack([r0.T @ (p - p0) for p in positions_world], axis=0)
    rotations_local = np.stack([r0.T @ r for r in world_rotations], axis=0)

    history_xyz = positions_local[:HISTORY_LEN].astype(np.float32)[None, None, :, :]
    history_rot = rotations_local[:HISTORY_LEN].astype(np.float32)[None, None, :, :, :]
    future_gt_xyz = positions_local[HISTORY_LEN:].astype(np.float32)[None, None, :, :]

    meta = {
        f"t0_{time_base}_ns": int(t0_ns),
        f"history_times_{time_base}_ns": history_times.tolist(),
        f"future_times_{time_base}_ns": future_times.tolist(),
        "reference_lla_for_enu": ref_lla.tolist(),
        "future_horizon_seconds": FUTURE_DURATION_NS / 1e9,
        "time_base": time_base,
    }
    return history_xyz, history_rot, future_gt_xyz, meta


def build_request_json(
    image_picks: list[CameraPick],
    xyz_path: Path,
    rot_path: Path,
    future_gt_xyz_path: Path,
    t0_ns: int,
    chunk_id: int,
    dataset_dir: Path,
    meta_path: Path,
    time_base: str,
) -> dict:
    user_content = []
    by_camera: dict[int, list[CameraPick]] = {}
    for pick in image_picks:
        by_camera.setdefault(pick.camera_id, []).append(pick)
    for _, camera_id, semantic_name in CAMERA_ORDER:
        ordered = sorted(by_camera[camera_id], key=lambda pick: pick.target_time_ns)
        display_name = CAMERA_DISPLAY_NAMES[semantic_name]
        user_content.append({"type": "text", "text": f"{display_name} camera recent frames:"})
        for pick in ordered:
            user_content.append({"type": "image", "image": pick.output_path})

    return {
        "requests": [
            {
                "messages": [
                    {
                        "role": "system",
                        "content": [
                            {
                                "type": "text",
                                "text": "You are given synchronized multi-camera driving context and ego motion history.",
                            }
                        ],
                    },
                    {"role": "user", "content": user_content},
                ],
                "ego_history_xyz_npy": str(xyz_path),
                "ego_history_rot_npy": str(rot_path),
                "future_gt_xyz_npy": str(future_gt_xyz_path),
                "action_space_constants": {},
                "traj_token_offset": 0,
                "diffusion_seed": 42,
                "diffusion_num_steps": 10,
                "nav_text": "",
                "metadata_json": str(meta_path),
                "camera_roles": {
                    f"cam{camera_id}": semantic_name
                    for _, camera_id, semantic_name in CAMERA_ORDER
                },
                "source_session_id": dataset_dir.name,
                "source_chunk_id": chunk_id,
                "time_base": time_base,
                f"t0_{time_base}_ns": int(t0_ns),
            }
        ]
    }


def main() -> None:
    args = parse_args()
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    images_dir = output_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    chunk_frames = {
        sensor_name: load_chunk_frames(args.dataset_dir, sensor_name, args.chunk_id, args.time_base)
        for sensor_name, _, _ in CAMERA_ORDER
    }
    t0_ns, chunk_window = pick_common_t0(
        chunk_frames,
        args.t0_margin_ns,
        args.t0_seconds_from_chunk_start,
        args.time_base,
    )

    image_picks: list[CameraPick] = []
    for sensor_name, camera_id, semantic_name in CAMERA_ORDER:
        rows = chunk_frames[sensor_name]
        video_path = args.dataset_dir / "sensors" / sensor_name / "chunks" / f"chunk_{args.chunk_id:04d}.mkv"
        for frame_slot, offset_ns in enumerate(IMAGE_FRAME_OFFSETS_NS):
            target_time_ns = t0_ns + offset_ns
            row = nearest_row(rows, target_time_ns, args.time_base)
            semantic_path, cam_id_path = save_camera_frame_variants(
                video_path=video_path,
                frame_index=row["frame_index_in_chunk"],
                images_dir=images_dir,
                camera_id=camera_id,
                semantic_name=semantic_name,
                frame_slot=frame_slot,
            )
            image_picks.append(
                CameraPick(
                    sensor_name=sensor_name,
                    camera_id=camera_id,
                    semantic_name=semantic_name,
                    target_time_ns=int(target_time_ns),
                    selected_time_ns=int(row[get_time_field(args.time_base)]),
                    frame_index_in_chunk=int(row["frame_index_in_chunk"]),
                    output_path=str(semantic_path),
                )
            )

    ego_history_xyz, ego_history_rot, future_gt_xyz, pose_meta = build_pose_and_future(
        args.dataset_dir,
        t0_ns,
        args.time_base,
    )
    xyz_path = output_dir / "ego_history_xyz.npy"
    rot_path = output_dir / "ego_history_rot.npy"
    future_gt_xyz_path = output_dir / "future_gt_xyz.npy"
    np.save(xyz_path, ego_history_xyz)
    np.save(rot_path, ego_history_rot)
    np.save(future_gt_xyz_path, future_gt_xyz)

    metadata = {
        "source_dataset_dir": str(args.dataset_dir),
        "source_chunk_id": args.chunk_id,
        "time_base": args.time_base,
        f"t0_{args.time_base}_ns": int(t0_ns),
        "camera_mapping": [
            {
                "sensor_name": sensor_name,
                "camera_id": camera_id,
                "semantic_name": semantic_name,
                "display_name": CAMERA_DISPLAY_NAMES[semantic_name],
                "semantic_image_prefix": semantic_name,
                "legacy_cam_prefix": f"cam{camera_id}",
            }
            for sensor_name, camera_id, semantic_name in CAMERA_ORDER
        ],
        "image_frame_offsets_ns": IMAGE_FRAME_OFFSETS_NS,
        "history_len": HISTORY_LEN,
        "history_dt_ns": HISTORY_DT_NS,
        "future_len": int(FUTURE_DURATION_NS // FUTURE_DT_NS),
        "future_dt_ns": FUTURE_DT_NS,
        "image_picks": [pick.__dict__ for pick in image_picks],
        "chunk_window": chunk_window,
        "pose_meta": pose_meta,
    }
    metadata_path = output_dir / "conversion_metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2))

    request = build_request_json(
        image_picks=image_picks,
        xyz_path=xyz_path,
        rot_path=rot_path,
        future_gt_xyz_path=future_gt_xyz_path,
        t0_ns=t0_ns,
        chunk_id=args.chunk_id,
        dataset_dir=args.dataset_dir,
        meta_path=metadata_path,
        time_base=args.time_base,
    )
    request_path = output_dir / f"request_chunk{args.chunk_id:04d}.json"
    request_path.write_text(json.dumps(request, indent=2))

    print(f"Saved VLM input bundle to: {output_dir}")
    print(f"chunk_id: {args.chunk_id}")
    print(f"t0_{args.time_base}_ns: {t0_ns}")
    print(f"ego_history_xyz shape: {ego_history_xyz.shape}")
    print(f"ego_history_rot shape: {ego_history_rot.shape}")
    print(f"future_gt_xyz shape: {future_gt_xyz.shape}")


if __name__ == "__main__":
    main()
