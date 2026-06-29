#!/usr/bin/env python3
"""Render a portfolio video with a camera stream and ego-relative route overlay.

The raw recorder stores camera chunks plus timestamp tables and GNSS/INS CSVs.
This script decodes one camera stream with ffmpeg, aligns it to GNSS latitude /
longitude samples, draws past/future trajectory in the ego frame, and encodes a
single mp4 suitable for portfolio/demo use.
"""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps


NS_PER_SEC = 1_000_000_000


@dataclass(frozen=True)
class FrameRow:
    chunk_id: int
    frame_index: int
    timestamp_ns: int


@dataclass
class GnssTrack:
    times_ns: np.ndarray
    xy_m: np.ndarray

    def xy_at(self, query_ns: np.ndarray | int) -> np.ndarray:
        scalar = np.isscalar(query_ns)
        query = np.asarray([query_ns] if scalar else query_ns, dtype=np.float64)
        x = np.interp(query, self.times_ns.astype(np.float64), self.xy_m[:, 0])
        y = np.interp(query, self.times_ns.astype(np.float64), self.xy_m[:, 1])
        out = np.column_stack([x, y])
        return out[0] if scalar else out


@dataclass
class CaptionTrack:
    times_ns: np.ndarray
    captions: list[str]
    fallback: str

    def caption_at(self, timestamp_ns: int) -> str:
        if len(self.captions) == 0 or len(self.times_ns) == 0:
            return self.fallback
        index = int(np.argmin(np.abs(self.times_ns - np.int64(timestamp_ns))))
        caption = self.captions[index].strip()
        return caption if caption else self.fallback


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-dir", type=Path, required=True)
    parser.add_argument("--camera", default="camera_front")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--preview-image", type=Path, default=None)
    parser.add_argument("--preview-at-seconds", type=float, default=95.0)
    parser.add_argument(
        "--caption",
        default="Reasoning summary: side cameras verify curb clearance while GNSS/INS history projects a smooth leftward route.",
    )
    parser.add_argument("--caption-csv", type=Path, default=None)
    parser.add_argument("--caption-column", default="")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--start-seconds", type=float, default=0.0)
    parser.add_argument("--max-seconds", type=float, default=0.0)
    parser.add_argument("--past-seconds", type=float, default=5.0)
    parser.add_argument("--future-seconds", type=float, default=6.4)
    parser.add_argument("--forward-meters", type=float, default=36.0)
    parser.add_argument("--backward-meters", type=float, default=9.0)
    parser.add_argument("--side-meters", type=float, default=18.0)
    parser.add_argument("--bev-panel-size", type=int, default=300)
    parser.add_argument("--bev-grid-meters", type=float, default=5.0)
    parser.add_argument("--thumbnail-width", type=int, default=250)
    parser.add_argument("--thumbnail-height", type=int, default=141)
    parser.add_argument("--no-flip-bev-lateral", action="store_true")
    parser.add_argument("--crf", type=int, default=20)
    return parser.parse_args()


def normalize_camera_name(name: str) -> str:
    return name if name.startswith("camera_") else f"camera_{name}"


def read_camera_rows(dataset_dir: Path, camera: str) -> list[FrameRow]:
    path = dataset_dir / "sensors" / camera / "frames.csv"
    if not path.exists():
        raise FileNotFoundError(path)
    rows: list[FrameRow] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(
                FrameRow(
                    chunk_id=int(row["chunk_id"]),
                    frame_index=int(row["frame_index_in_chunk"]),
                    timestamp_ns=int(row["timestamp_utc_ns"]),
                )
            )
    rows.sort(key=lambda r: (r.chunk_id, r.frame_index))
    if not rows:
        raise RuntimeError(f"No camera rows found in {path}")
    return rows


def nearest_frame_row(rows: list[FrameRow], timestamp_ns: int) -> FrameRow:
    times = np.asarray([row.timestamp_ns for row in rows], dtype=np.int64)
    index = int(np.argmin(np.abs(times - timestamp_ns)))
    return rows[index]


def lla_to_ecef(lat_deg: np.ndarray, lon_deg: np.ndarray, alt_m: np.ndarray) -> np.ndarray:
    a = 6378137.0
    e2 = 6.69437999014e-3
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)
    n = a / np.sqrt(1.0 - e2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * cos_lon
    y = (n + alt_m) * cos_lat * sin_lon
    z = (n * (1.0 - e2) + alt_m) * sin_lat
    return np.column_stack([x, y, z])


def ecef_to_enu(ecef_xyz: np.ndarray, ref_lla: np.ndarray) -> np.ndarray:
    ref_lat = math.radians(float(ref_lla[0]))
    ref_lon = math.radians(float(ref_lla[1]))
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


def read_gnss_track(dataset_dir: Path) -> GnssTrack:
    path = dataset_dir / "sensors" / "gnss_ins" / "gnss_ins.csv"
    if not path.exists():
        raise FileNotFoundError(path)

    samples: list[tuple[int, float, float, float]] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if not row.get("timestamp_utc_ns") or not row.get("lat") or not row.get("lon"):
                continue
            try:
                alt = float(row["alt"]) if row.get("alt") else 0.0
                samples.append(
                    (
                        int(row["timestamp_utc_ns"]),
                        float(row["lat"]),
                        float(row["lon"]),
                        alt,
                    )
                )
            except ValueError:
                continue

    if len(samples) < 4:
        raise RuntimeError(f"Not enough GNSS samples in {path}")

    samples.sort(key=lambda x: x[0])
    deduped: list[tuple[int, float, float, float]] = []
    seen: set[int] = set()
    for sample in samples:
        if sample[0] in seen:
            continue
        seen.add(sample[0])
        deduped.append(sample)

    times_ns = np.asarray([s[0] for s in deduped], dtype=np.int64)
    lla = np.asarray([[s[1], s[2], s[3]] for s in deduped], dtype=np.float64)
    ecef = lla_to_ecef(lla[:, 0], lla[:, 1], lla[:, 2])
    enu = ecef_to_enu(ecef, lla[0])
    return GnssTrack(times_ns=times_ns, xy_m=enu[:, :2])


def load_caption_track(path: Path | None, fallback: str, caption_column: str = "") -> CaptionTrack:
    if path is None:
        return CaptionTrack(times_ns=np.asarray([], dtype=np.int64), captions=[], fallback=fallback)
    if not path.exists():
        raise FileNotFoundError(path)

    times: list[int] = []
    captions: list[str] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise RuntimeError(f"Caption CSV has no header: {path}")
        fields = set(reader.fieldnames)
        if "timestamp_utc_ns" in fields:
            time_field = "timestamp_utc_ns"
            scale = 1
        elif "t0_us" in fields:
            time_field = "t0_us"
            scale = 1000
        elif "timestamp_us" in fields:
            time_field = "timestamp_us"
            scale = 1000
        else:
            raise RuntimeError("Caption CSV needs timestamp_utc_ns, t0_us, or timestamp_us")

        if caption_column:
            text_field = caption_column
        else:
            candidates = ["caption", "reasoning_summary", "rationale", "cot", "text"]
            text_field = next((name for name in candidates if name in fields), "")
        if not text_field or text_field not in fields:
            raise RuntimeError("Caption CSV needs a caption/reasoning_summary/rationale/cot/text column")

        for row in reader:
            if not row.get(time_field):
                continue
            text = row.get(text_field, "").strip()
            if not text:
                continue
            try:
                times.append(int(row[time_field]) * scale)
                captions.append(text)
            except ValueError:
                continue

    if not captions:
        raise RuntimeError(f"No usable captions found in {path}")
    order = np.argsort(np.asarray(times, dtype=np.int64))
    sorted_times = np.asarray(times, dtype=np.int64)[order]
    sorted_captions = [captions[int(i)] for i in order]
    return CaptionTrack(times_ns=sorted_times, captions=sorted_captions, fallback=fallback)


def make_output_path(dataset_dir: Path) -> Path:
    out_dir = Path("/workspace/portfolio_outputs")
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir / f"{dataset_dir.name}_camera_path_overlay.mp4"


def launch_encoder(output: Path, width: int, height: int, fps: float, crf: int) -> subprocess.Popen:
    output.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{width}x{height}",
        "-r",
        f"{fps}",
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "medium",
        "-crf",
        str(crf),
        "-pix_fmt",
        "yuv420p",
        str(output),
    ]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE)


def launch_decoder(video_path: Path, width: int, height: int) -> subprocess.Popen:
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-i",
        str(video_path),
        "-vf",
        f"scale={width}:{height}",
        "-pix_fmt",
        "rgb24",
        "-f",
        "rawvideo",
        "-",
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE)


def decode_single_frame(video_path: Path, frame_index: int, width: int, height: int) -> Image.Image:
    bytes_per_frame = width * height * 3
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-i",
        str(video_path),
        "-vf",
        f"select=eq(n\\,{frame_index}),scale={width}:{height}",
        "-frames:v",
        "1",
        "-pix_fmt",
        "rgb24",
        "-f",
        "rawvideo",
        "-",
    ]
    result = subprocess.run(cmd, check=True, stdout=subprocess.PIPE)
    if len(result.stdout) != bytes_per_frame:
        raise RuntimeError(f"Failed to decode frame {frame_index} from {video_path}")
    return Image.frombytes("RGB", (width, height), result.stdout)


def decode_dataset_frame(dataset_dir: Path, camera: str, row: FrameRow, width: int, height: int) -> Image.Image:
    video_path = dataset_dir / "sensors" / camera / "chunks" / f"chunk_{row.chunk_id:04d}.mkv"
    if not video_path.exists():
        raise FileNotFoundError(video_path)
    return decode_single_frame(video_path, row.frame_index, width, height)


def draw_polyline(draw: ImageDraw.ImageDraw, points: list[tuple[float, float]], fill: tuple[int, int, int], width: int) -> None:
    if len(points) < 2:
        return
    draw.line(points, fill=fill, width=width, joint="curve")


def grid_values(min_value: float, max_value: float, step: float) -> list[float]:
    if step <= 0.0:
        return []
    start = math.floor(min_value / step) * step
    values: list[float] = []
    current = start
    while current <= max_value + 1e-6:
        if current >= min_value - 1e-6:
            values.append(current)
        current += step
    return values


def grid_style(value: float, step: float) -> tuple[tuple[int, int, int, int], int]:
    major_step = step * 2.0
    is_major = abs((value / major_step) - round(value / major_step)) < 1e-6
    return ((92, 108, 125, 118), 1) if is_major else ((82, 96, 112, 68), 1)


def load_font(size: int) -> ImageFont.ImageFont:
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ):
        try:
            return ImageFont.truetype(path, size=size)
        except OSError:
            continue
    return ImageFont.load_default()


def ego_transform(track: GnssTrack, t_ns: int, query_ns: np.ndarray) -> tuple[np.ndarray, float, float]:
    now_xy = track.xy_at(t_ns)
    before_ns = max(int(track.times_ns[0]), t_ns - int(0.75 * NS_PER_SEC))
    after_ns = min(int(track.times_ns[-1]), t_ns + int(0.75 * NS_PER_SEC))
    before_xy = track.xy_at(before_ns)
    after_xy = track.xy_at(after_ns)
    heading_vec = after_xy - before_xy
    if float(np.linalg.norm(heading_vec)) < 0.05:
        after_xy = track.xy_at(min(int(track.times_ns[-1]), t_ns + int(2.0 * NS_PER_SEC)))
        heading_vec = after_xy - now_xy
    heading = math.atan2(float(heading_vec[1]), float(heading_vec[0])) if float(np.linalg.norm(heading_vec)) > 1e-6 else 0.0

    points_xy = track.xy_at(query_ns)
    delta = points_xy - now_xy
    forward = np.array([math.cos(heading), math.sin(heading)], dtype=np.float64)
    left = np.array([-math.sin(heading), math.cos(heading)], dtype=np.float64)
    local = np.column_stack([delta @ forward, delta @ left])
    speed = float(np.linalg.norm(heading_vec) / max(1e-6, (after_ns - before_ns) / NS_PER_SEC))
    return local, heading, speed


def local_to_panel_px(
    local: np.ndarray,
    panel_box: tuple[int, int, int, int],
    forward_m: float,
    backward_m: float,
    side_m: float,
    flip_lateral: bool,
) -> list[tuple[float, float]]:
    x0, y0, x1, y1 = panel_box
    w = x1 - x0
    h = y1 - y0
    scale_x = w / (2.0 * side_m)
    scale_y = h / (forward_m + backward_m)
    ego_x = x0 + w / 2.0
    ego_y = y1 - backward_m * scale_y
    lateral_sign = -1.0 if flip_lateral else 1.0
    px = ego_x + lateral_sign * local[:, 1] * scale_x
    py = ego_y - local[:, 0] * scale_y
    return list(zip(px.tolist(), py.tolist()))


def route_ranges(local_arrays: list[np.ndarray], args: argparse.Namespace) -> tuple[float, float, float]:
    valid = [arr for arr in local_arrays if arr.size]
    if not valid:
        return args.forward_meters, args.backward_meters, args.side_meters
    points = np.concatenate(valid, axis=0)
    max_forward = float(np.max(points[:, 0]))
    min_forward = float(np.min(points[:, 0]))
    max_side = float(np.max(np.abs(points[:, 1])))
    forward_m = max(20.0, min(args.forward_meters, max_forward + 5.0))
    backward_m = max(6.0, min(args.backward_meters, -min_forward + 3.0))
    side_m = max(10.0, min(args.side_meters, max_side + 4.0))
    return forward_m, backward_m, side_m


def draw_camera_insets(
    overlay: Image.Image,
    side_frames: dict[str, Image.Image] | None,
    args: argparse.Namespace,
) -> None:
    if not side_frames:
        return
    draw = ImageDraw.Draw(overlay)
    font = ImageFont.load_default()
    margin = 24
    gap = 12
    thumb_w = args.thumbnail_width
    thumb_h = args.thumbnail_height
    labels = [("camera_left", "LEFT"), ("camera_right", "RIGHT")]
    for idx, (camera_name, label) in enumerate(labels):
        if camera_name not in side_frames:
            continue
        x = margin + idx * (thumb_w + gap)
        y = margin
        thumb = ImageOps.fit(side_frames[camera_name], (thumb_w, thumb_h), method=Image.Resampling.LANCZOS)
        shadow = Image.new("RGBA", (thumb_w + 8, thumb_h + 8), (0, 0, 0, 0))
        shadow_draw = ImageDraw.Draw(shadow)
        shadow_draw.rounded_rectangle((4, 4, thumb_w + 4, thumb_h + 4), radius=8, fill=(0, 0, 0, 120))
        overlay.alpha_composite(shadow, (x - 4, y - 4))
        overlay.alpha_composite(thumb.convert("RGBA"), (x, y))
        draw.rounded_rectangle((x, y, x + thumb_w, y + thumb_h), radius=8, outline=(235, 241, 247, 205), width=2)
        draw.rounded_rectangle((x + 8, y + 8, x + 72, y + 28), radius=5, fill=(8, 12, 18, 180))
        draw.text((x + 16, y + 14), label, fill=(255, 255, 255, 235), font=font)


def render_overlay(
    frame: Image.Image,
    track: GnssTrack,
    t_ns: int,
    dataset_name: str,
    camera_name: str,
    frame_count: int,
    total_frames: int,
    args: argparse.Namespace,
    caption: str,
    side_frames: dict[str, Image.Image] | None = None,
) -> Image.Image:
    canvas = frame.convert("RGBA")
    overlay = Image.new("RGBA", canvas.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    caption_font = load_font(15)
    meta_font = load_font(12)
    panel_font = load_font(12)

    margin = 24
    panel_size = min(args.bev_panel_size, args.height - 2 * margin)
    panel_box = (args.width - panel_size - margin, margin, args.width - margin, margin + panel_size)
    x0, y0, x1, y1 = panel_box

    draw.rounded_rectangle(panel_box, radius=8, fill=(8, 12, 18, 205), outline=(220, 230, 240, 180), width=2)

    past_times = np.arange(
        max(int(track.times_ns[0]), t_ns - int(args.past_seconds * NS_PER_SEC)),
        t_ns + 1,
        int(0.1 * NS_PER_SEC),
        dtype=np.int64,
    )
    future_times = np.arange(
        t_ns,
        min(int(track.times_ns[-1]), t_ns + int(args.future_seconds * NS_PER_SEC)) + 1,
        int(0.1 * NS_PER_SEC),
        dtype=np.int64,
    )
    past_local, _, speed = ego_transform(track, t_ns, past_times)
    future_local, heading, _ = ego_transform(track, t_ns, future_times)
    forward_m, backward_m, side_m = route_ranges([past_local, future_local], args)
    flip_lateral = not args.no_flip_bev_lateral

    # Grid in ego-local meters.
    grid_step = max(1.0, float(args.bev_grid_meters))
    for meters in grid_values(-side_m, side_m, grid_step):
        pts = np.array([[-backward_m, meters], [forward_m, meters]], dtype=np.float64)
        fill, line_width = grid_style(meters, grid_step)
        draw_polyline(
            draw,
            local_to_panel_px(pts, panel_box, forward_m, backward_m, side_m, flip_lateral),
            fill=fill,
            width=line_width,
        )
    for meters in grid_values(-backward_m, forward_m, grid_step):
        pts = np.array([[meters, -side_m], [meters, side_m]], dtype=np.float64)
        fill, line_width = grid_style(meters, grid_step)
        draw_polyline(
            draw,
            local_to_panel_px(pts, panel_box, forward_m, backward_m, side_m, flip_lateral),
            fill=fill,
            width=line_width,
        )

    draw_polyline(
        draw,
        local_to_panel_px(past_local, panel_box, forward_m, backward_m, side_m, flip_lateral),
        fill=(245, 190, 75, 230),
        width=4,
    )
    draw_polyline(
        draw,
        local_to_panel_px(future_local, panel_box, forward_m, backward_m, side_m, flip_lateral),
        fill=(70, 225, 185, 245),
        width=5,
    )

    ego_px = local_to_panel_px(
        np.array([[0.0, 0.0]], dtype=np.float64),
        panel_box,
        forward_m,
        backward_m,
        side_m,
        flip_lateral,
    )[0]
    ex, ey = ego_px
    draw.polygon([(ex, ey - 13), (ex - 9, ey + 11), (ex + 9, ey + 11)], fill=(255, 255, 255, 245))
    draw.text((x0 + 14, y0 + 12), "Ego Route", fill=(255, 255, 255, 245), font=panel_font)
    draw.text((x0 + 14, y0 + 32), "orange=past  green=future", fill=(200, 214, 225, 230), font=panel_font)
    draw.text((x0 + 14, y1 - 30), f"speed {speed * 3.6:4.1f} km/h  yaw {math.degrees(heading):5.1f}", fill=(230, 238, 245, 230), font=panel_font)

    elapsed = (t_ns - int(track.times_ns[0])) / NS_PER_SEC
    progress = frame_count / max(1, total_frames)
    bar_w = args.width - 2 * margin
    bar_y = args.height - 42
    draw.rounded_rectangle((margin, bar_y, margin + bar_w, bar_y + 10), radius=5, fill=(12, 18, 24, 190))
    draw.rounded_rectangle((margin, bar_y, margin + int(bar_w * progress), bar_y + 10), radius=5, fill=(70, 225, 185, 230))
    caption = caption.strip()
    caption_box_right = min(args.width - margin, margin + 930)
    draw.rounded_rectangle((margin, args.height - 102, caption_box_right, args.height - 56), radius=8, fill=(8, 12, 18, 178))
    draw.text((margin + 14, args.height - 92), caption, fill=(255, 255, 255, 242), font=caption_font)
    draw.text((margin + 14, args.height - 72), f"{dataset_name}  |  {camera_name}  |  t={elapsed:6.1f}s", fill=(205, 218, 230, 220), font=meta_font)
    draw_camera_insets(overlay, side_frames, args)

    return Image.alpha_composite(canvas, overlay).convert("RGB")


def render_video(args: argparse.Namespace) -> Path:
    dataset_dir = args.dataset_dir.resolve()
    camera = normalize_camera_name(args.camera)
    output = args.output.resolve() if args.output else make_output_path(dataset_dir)

    frame_rows = read_camera_rows(dataset_dir, camera)
    track = read_gnss_track(dataset_dir)
    caption_track = load_caption_track(args.caption_csv, args.caption, args.caption_column)

    start_ns = max(frame_rows[0].timestamp_ns, int(track.times_ns[0]) + int(args.past_seconds * NS_PER_SEC))
    start_ns += int(args.start_seconds * NS_PER_SEC)
    end_ns = min(frame_rows[-1].timestamp_ns, int(track.times_ns[-1]) - int(args.future_seconds * NS_PER_SEC))
    if args.max_seconds > 0:
        end_ns = min(end_ns, start_ns + int(args.max_seconds * NS_PER_SEC))
    if end_ns <= start_ns:
        raise RuntimeError("No overlapping camera/GNSS time range to render")

    target_dt_ns = int(round(NS_PER_SEC / args.fps))
    next_target_ns = start_ns
    total_frames = max(1, int(math.floor((end_ns - start_ns) / target_dt_ns)) + 1)
    rendered = 0
    bytes_per_frame = args.width * args.height * 3

    rows_by_chunk: dict[int, list[FrameRow]] = defaultdict(list)
    for row in frame_rows:
        rows_by_chunk[row.chunk_id].append(row)

    encoder = launch_encoder(output, args.width, args.height, args.fps, args.crf)
    assert encoder.stdin is not None
    try:
        for chunk_id in sorted(rows_by_chunk):
            if next_target_ns > end_ns:
                break
            chunk_path = dataset_dir / "sensors" / camera / "chunks" / f"chunk_{chunk_id:04d}.mkv"
            if not chunk_path.exists():
                raise FileNotFoundError(chunk_path)
            decoder = launch_decoder(chunk_path, args.width, args.height)
            assert decoder.stdout is not None
            try:
                for row in sorted(rows_by_chunk[chunk_id], key=lambda r: r.frame_index):
                    raw = decoder.stdout.read(bytes_per_frame)
                    if len(raw) != bytes_per_frame:
                        break
                    if row.timestamp_ns < next_target_ns:
                        continue
                    image = Image.frombytes("RGB", (args.width, args.height), raw)
                    while next_target_ns <= row.timestamp_ns and next_target_ns <= end_ns:
                        composed = render_overlay(
                            frame=image,
                            track=track,
                            t_ns=next_target_ns,
                            dataset_name=dataset_dir.name,
                            camera_name=camera,
                            frame_count=rendered,
                            total_frames=total_frames,
                            args=args,
                            caption=caption_track.caption_at(next_target_ns),
                        )
                        encoder.stdin.write(composed.tobytes())
                        rendered += 1
                        next_target_ns += target_dt_ns
                decoder.stdout.close()
                decoder.wait(timeout=10)
            finally:
                if decoder.poll() is None:
                    decoder.kill()

        encoder.stdin.close()
        ret = encoder.wait(timeout=60)
        if ret != 0:
            raise RuntimeError(f"ffmpeg encoder failed with exit code {ret}")
    finally:
        if encoder.poll() is None:
            encoder.kill()

    if rendered == 0:
        raise RuntimeError("Rendered zero frames")
    print(f"rendered_frames={rendered}")
    print(f"output={output}")
    return output


def render_preview_image(args: argparse.Namespace) -> Path:
    dataset_dir = args.dataset_dir.resolve()
    camera = normalize_camera_name(args.camera)
    output = args.preview_image.resolve()

    frame_rows = read_camera_rows(dataset_dir, camera)
    track = read_gnss_track(dataset_dir)
    caption_track = load_caption_track(args.caption_csv, args.caption, args.caption_column)

    start_ns = max(frame_rows[0].timestamp_ns, int(track.times_ns[0]) + int(args.past_seconds * NS_PER_SEC))
    end_ns = min(frame_rows[-1].timestamp_ns, int(track.times_ns[-1]) - int(args.future_seconds * NS_PER_SEC))
    if end_ns <= start_ns:
        raise RuntimeError("No overlapping camera/GNSS time range to preview")

    requested_ns = int(track.times_ns[0]) + int(args.preview_at_seconds * NS_PER_SEC)
    t_ns = min(max(requested_ns, start_ns), end_ns)
    main_row = nearest_frame_row(frame_rows, t_ns)
    base_frame = decode_dataset_frame(dataset_dir, camera, main_row, args.width, args.height)

    side_frames: dict[str, Image.Image] = {}
    thumb_decode_w = max(args.thumbnail_width * 2, args.thumbnail_width)
    thumb_decode_h = max(args.thumbnail_height * 2, args.thumbnail_height)
    for side_camera in ("camera_left", "camera_right"):
        try:
            rows = read_camera_rows(dataset_dir, side_camera)
            row = nearest_frame_row(rows, t_ns)
            side_frames[side_camera] = decode_dataset_frame(dataset_dir, side_camera, row, thumb_decode_w, thumb_decode_h)
        except Exception as exc:
            print(f"warn: failed to add {side_camera} inset: {exc}", file=sys.stderr)

    target_dt_ns = int(round(NS_PER_SEC / args.fps))
    total_frames = max(1, int(math.floor((end_ns - start_ns) / target_dt_ns)) + 1)
    frame_count = max(0, min(total_frames - 1, int((t_ns - start_ns) / target_dt_ns)))
    composed = render_overlay(
        frame=base_frame,
        track=track,
        t_ns=t_ns,
        dataset_name=dataset_dir.name,
        camera_name=camera,
        frame_count=frame_count,
        total_frames=total_frames,
        args=args,
        caption=caption_track.caption_at(t_ns),
        side_frames=side_frames,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    composed.save(output)
    print(f"preview_time_seconds={(t_ns - int(track.times_ns[0])) / NS_PER_SEC:.3f}")
    print(f"output={output}")
    return output


def main() -> None:
    try:
        args = parse_args()
        if args.preview_image:
            render_preview_image(args)
        else:
            render_video(args)
    except BrokenPipeError:
        raise
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
