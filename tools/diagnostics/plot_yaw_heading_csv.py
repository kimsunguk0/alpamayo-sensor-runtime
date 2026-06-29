#!/usr/bin/env python3
"""Plot yaw/heading CSV logs emitted by the live sensor container."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass
class YawHeadingRows:
    label: str
    path: Path
    time_us: list[int]
    yaw_valid: list[bool]
    yaw_deg: list[float]
    heading_valid: list[bool]
    heading_yaw_deg: list[float]
    heading_compass_deg: list[float]
    yaw_minus_heading_deg: list[float]
    speed_2d_mps: list[float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", type=Path, default=Path("/workspace/yaw_heading_logs"))
    parser.add_argument("--clip-id", default="bringup_run_live_4k_2cam")
    parser.add_argument("--sensor-csv", type=Path)
    parser.add_argument("--planner-csv", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--min-speed-mps",
        type=float,
        default=0.0,
        help="Only treat GNSS velocity heading as valid at or above this speed.",
    )
    parser.add_argument("--show", action="store_true", help="Display the plot window after saving.")
    return parser.parse_args()


def finite_float(text: str | None) -> float:
    if text is None or text == "":
        return math.nan
    try:
        value = float(text)
    except ValueError:
        return math.nan
    return value if math.isfinite(value) else math.nan


def bool_field(text: str | None) -> bool:
    return text in {"1", "true", "True", "yes", "YES"}


def read_csv(path: Path, label: str, min_speed_mps: float) -> YawHeadingRows:
    with path.open("r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        rows = list(reader)

    if not rows:
        raise ValueError(f"{path} has no data rows")

    timestamp_key = "t0_us" if "t0_us" in rows[0] else "utc_us"
    out = YawHeadingRows(
        label=label,
        path=path,
        time_us=[],
        yaw_valid=[],
        yaw_deg=[],
        heading_valid=[],
        heading_yaw_deg=[],
        heading_compass_deg=[],
        yaw_minus_heading_deg=[],
        speed_2d_mps=[],
    )

    for row in rows:
        try:
            time_us = int(row.get(timestamp_key, "0") or "0")
        except ValueError:
            continue
        speed = finite_float(row.get("speed_2d_mps"))
        heading_valid = bool_field(row.get("heading_valid"))
        if min_speed_mps > 0.0 and (not math.isfinite(speed) or speed < min_speed_mps):
            heading_valid = False
        yaw_valid = bool_field(row.get("yaw_valid"))

        out.time_us.append(time_us)
        out.yaw_valid.append(yaw_valid)
        out.yaw_deg.append(finite_float(row.get("yaw_deg")) if yaw_valid else math.nan)
        out.heading_valid.append(heading_valid)
        out.heading_yaw_deg.append(finite_float(row.get("heading_yaw_deg")) if heading_valid else math.nan)
        out.heading_compass_deg.append(finite_float(row.get("heading_compass_deg")) if heading_valid else math.nan)
        out.yaw_minus_heading_deg.append(
            finite_float(row.get("yaw_minus_heading_deg")) if yaw_valid and heading_valid else math.nan
        )
        out.speed_2d_mps.append(speed)

    return out


def finite_values(values: Iterable[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    idx = (len(ordered) - 1) * pct
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - idx) + ordered[hi] * (idx - lo)


def summarize(rows: YawHeadingRows) -> str:
    speed = finite_values(rows.speed_2d_mps)
    yaw_error_abs = [abs(value) for value in finite_values(rows.yaw_minus_heading_deg)]
    both_valid = sum(
        1 for yaw_ok, heading_ok in zip(rows.yaw_valid, rows.heading_valid) if yaw_ok and heading_ok
    )
    return (
        f"{rows.label}: rows={len(rows.time_us)} "
        f"yaw_valid={sum(rows.yaw_valid)} heading_valid={sum(rows.heading_valid)} both_valid={both_valid} "
        f"speed_mean={mean(speed):.3f}m/s speed_max={max(speed) if speed else math.nan:.3f}m/s "
        f"abs_err_mean={mean(yaw_error_abs):.3f}deg abs_err_p95={percentile(yaw_error_abs, 0.95):.3f}deg"
    )


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else math.nan


def seconds_from_origin(rows: YawHeadingRows, origin_us: int) -> list[float]:
    return [(time_us - origin_us) / 1e6 for time_us in rows.time_us]


def masked(values: list[float], valid: list[bool]) -> list[float]:
    return [value if ok and math.isfinite(value) else math.nan for value, ok in zip(values, valid)]


def make_plot(datasets: list[YawHeadingRows], output_path: Path, show: bool) -> None:
    import matplotlib.pyplot as plt

    origin_us = min(min(rows.time_us) for rows in datasets if rows.time_us)
    fig, axes = plt.subplots(4, 1, figsize=(14, 11))
    fig.suptitle("Yaw vs GNSS Velocity Heading")

    for rows in datasets:
        t = seconds_from_origin(rows, origin_us)
        both_valid = [yaw_ok and heading_ok for yaw_ok, heading_ok in zip(rows.yaw_valid, rows.heading_valid)]
        axes[0].plot(t, masked(rows.yaw_deg, rows.yaw_valid), ".", markersize=2, label=f"{rows.label} yaw")
        axes[0].plot(
            t,
            masked(rows.heading_yaw_deg, rows.heading_valid),
            ".",
            markersize=2,
            label=f"{rows.label} velocity heading",
        )
        axes[1].plot(t, masked(rows.yaw_minus_heading_deg, both_valid), ".", markersize=2, label=rows.label)
        axes[2].plot(t, rows.speed_2d_mps, ".", markersize=2, label=rows.label)
        axes[3].hist(
            finite_values(masked(rows.yaw_minus_heading_deg, both_valid)),
            bins=72,
            alpha=0.45,
            label=rows.label,
        )

    axes[0].set_ylabel("deg")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].set_ylabel("yaw-heading deg")
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")
    axes[2].set_ylabel("speed m/s")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="best")
    axes[3].set_xlabel("yaw-heading deg")
    axes[3].set_ylabel("count")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc="best")
    axes[2].set_xlabel("seconds since first row")

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def svg_escape(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def padded_range(values: list[float], default: tuple[float, float]) -> tuple[float, float]:
    finite = finite_values(values)
    if not finite:
        return default
    lo = min(finite)
    hi = max(finite)
    if math.isclose(lo, hi):
        pad = max(1.0, abs(lo) * 0.1)
    else:
        pad = (hi - lo) * 0.08
    return lo - pad, hi + pad


def svg_polyline(
    xs: list[float],
    ys: list[float],
    valid: list[bool],
    rect: tuple[float, float, float, float],
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    color: str,
) -> str:
    x0, y0, w, h = rect
    xmin, xmax = x_range
    ymin, ymax = y_range
    if xmax <= xmin or ymax <= ymin:
        return ""

    segments: list[str] = []
    current: list[str] = []
    for x, y, ok in zip(xs, ys, valid):
        if not ok or not math.isfinite(x) or not math.isfinite(y):
            if len(current) >= 2:
                segments.append(
                    f'<polyline points="{" ".join(current)}" fill="none" stroke="{color}" '
                    'stroke-width="1.4" stroke-linejoin="round" stroke-linecap="round" />'
                )
            current = []
            continue
        sx = x0 + (x - xmin) / (xmax - xmin) * w
        sy = y0 + h - (y - ymin) / (ymax - ymin) * h
        current.append(f"{sx:.1f},{sy:.1f}")
    if len(current) >= 2:
        segments.append(
            f'<polyline points="{" ".join(current)}" fill="none" stroke="{color}" '
            'stroke-width="1.4" stroke-linejoin="round" stroke-linecap="round" />'
        )
    return "\n".join(segments)


def svg_panel(
    title: str,
    rect: tuple[float, float, float, float],
    x_label: str,
    y_label: str,
    y_range: tuple[float, float],
) -> str:
    x, y, w, h = rect
    ymin, ymax = y_range
    parts = [
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="white" stroke="#b8c0cc" />',
        f'<text x="{x}" y="{y - 8}" font-size="15" font-weight="600">{svg_escape(title)}</text>',
        f'<text x="{x + w / 2}" y="{y + h + 34}" font-size="12" text-anchor="middle">{svg_escape(x_label)}</text>',
        (
            f'<text x="{x - 44}" y="{y + h / 2}" font-size="12" text-anchor="middle" '
            f'transform="rotate(-90 {x - 44} {y + h / 2})">{svg_escape(y_label)}</text>'
        ),
    ]
    for i in range(5):
        gy = y + h * i / 4.0
        value = ymax - (ymax - ymin) * i / 4.0
        parts.append(f'<line x1="{x}" y1="{gy:.1f}" x2="{x + w}" y2="{gy:.1f}" stroke="#edf0f5" />')
        parts.append(f'<text x="{x - 8}" y="{gy + 4:.1f}" font-size="10" text-anchor="end">{value:.1f}</text>')
    return "\n".join(parts)


def make_svg_plot(datasets: list[YawHeadingRows], output_path: Path) -> None:
    width = 1240
    height = 980
    left = 92
    panel_w = 1060
    panel_h = 170
    gap = 62
    top = 72
    colors = {
        "sensor": ("#2563eb", "#dc2626"),
        "planner_10hz": ("#0891b2", "#f97316"),
    }

    origin_us = min(min(rows.time_us) for rows in datasets if rows.time_us)
    time_max = max(max(seconds_from_origin(rows, origin_us)) for rows in datasets if rows.time_us)
    x_range = (0.0, max(1.0, time_max))

    all_errors: list[float] = []
    all_speeds: list[float] = []
    for rows in datasets:
        all_errors.extend(finite_values(rows.yaw_minus_heading_deg))
        all_speeds.extend(finite_values(rows.speed_2d_mps))
    error_range = padded_range(all_errors, (-10.0, 10.0))
    speed_range = (0.0, max(1.0, max(all_speeds) * 1.1 if all_speeds else 1.0))

    rects = [
        (left, top + (panel_h + gap) * i, panel_w, panel_h)
        for i in range(4)
    ]
    parts = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc" />',
        '<text x="620" y="34" font-size="24" font-weight="700" text-anchor="middle">Yaw vs GNSS Velocity Heading</text>',
        svg_panel("Yaw and velocity heading", rects[0], "seconds since first row", "deg", (-180.0, 180.0)),
        svg_panel("Yaw minus velocity heading", rects[1], "seconds since first row", "deg", error_range),
        svg_panel("GNSS horizontal speed", rects[2], "seconds since first row", "m/s", speed_range),
        svg_panel("Yaw-heading error histogram", rects[3], "yaw-heading deg", "count", (0.0, 1.0)),
    ]

    legend_x = left + panel_w - 260
    legend_y = top - 34
    legend_items: list[tuple[str, str]] = []
    for rows in datasets:
        yaw_color, heading_color = colors.get(rows.label, ("#334155", "#ef4444"))
        t = seconds_from_origin(rows, origin_us)
        both_valid = [yaw_ok and heading_ok for yaw_ok, heading_ok in zip(rows.yaw_valid, rows.heading_valid)]
        parts.append(svg_polyline(t, rows.yaw_deg, rows.yaw_valid, rects[0], x_range, (-180.0, 180.0), yaw_color))
        parts.append(svg_polyline(t, rows.heading_yaw_deg, rows.heading_valid, rects[0], x_range, (-180.0, 180.0), heading_color))
        parts.append(svg_polyline(t, rows.yaw_minus_heading_deg, both_valid, rects[1], x_range, error_range, heading_color))
        parts.append(
            svg_polyline(
                t,
                rows.speed_2d_mps,
                [math.isfinite(v) for v in rows.speed_2d_mps],
                rects[2],
                x_range,
                speed_range,
                yaw_color,
            )
        )
        legend_items.append((f"{rows.label} yaw", yaw_color))
        legend_items.append((f"{rows.label} velocity heading/error", heading_color))

    for idx, (label, color) in enumerate(legend_items):
        y = legend_y + idx * 18
        parts.append(f'<line x1="{legend_x}" y1="{y}" x2="{legend_x + 22}" y2="{y}" stroke="{color}" stroke-width="3" />')
        parts.append(f'<text x="{legend_x + 30}" y="{y + 4}" font-size="12">{svg_escape(label)}</text>')

    hist_values_by_label = [
        (rows.label, finite_values(masked(rows.yaw_minus_heading_deg, [a and b for a, b in zip(rows.yaw_valid, rows.heading_valid)])))
        for rows in datasets
    ]
    all_hist_values = [value for _, values in hist_values_by_label for value in values]
    hist_min, hist_max = padded_range(all_hist_values, (-10.0, 10.0))
    bin_count = 72
    bin_width = (hist_max - hist_min) / bin_count
    hist_counts: list[tuple[str, str, list[int]]] = []
    max_count = 1
    for rows, (_, values) in zip(datasets, hist_values_by_label):
        _, color = colors.get(rows.label, ("#334155", "#ef4444"))
        counts = [0] * bin_count
        for value in values:
            idx = int((value - hist_min) / bin_width)
            idx = max(0, min(bin_count - 1, idx))
            counts[idx] += 1
        max_count = max(max_count, max(counts) if counts else 0)
        hist_counts.append((rows.label, color, counts))

    x, y, w, h = rects[3]
    parts.append(f'<text x="{x - 8}" y="{y + 4}" font-size="10" text-anchor="end">{max_count}</text>')
    bar_w = w / bin_count / max(1, len(hist_counts))
    for dataset_idx, (_label, color, counts) in enumerate(hist_counts):
        for bin_idx, count in enumerate(counts):
            if count <= 0:
                continue
            bx = x + bin_idx * (w / bin_count) + dataset_idx * bar_w
            bh = h * count / max_count
            by = y + h - bh
            parts.append(
                f'<rect x="{bx:.1f}" y="{by:.1f}" width="{max(1.0, bar_w - 1):.1f}" height="{bh:.1f}" '
                f'fill="{color}" opacity="0.55" />'
            )
    parts.append(f'<text x="{x}" y="{y + h + 18}" font-size="10">{hist_min:.1f}</text>')
    parts.append(f'<text x="{x + w}" y="{y + h + 18}" font-size="10" text-anchor="end">{hist_max:.1f}</text>')
    parts.append("</svg>")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def resolve_inputs(args: argparse.Namespace) -> tuple[Path | None, Path | None]:
    sensor_csv = args.sensor_csv
    planner_csv = args.planner_csv
    if sensor_csv is None:
        candidate = args.log_dir / f"{args.clip_id}_sensor_yaw_heading.csv"
        sensor_csv = candidate if candidate.exists() else None
    if planner_csv is None:
        candidate = args.log_dir / f"{args.clip_id}_planner_yaw_heading_10hz.csv"
        planner_csv = candidate if candidate.exists() else None
    return sensor_csv, planner_csv


def main() -> int:
    args = parse_args()
    sensor_csv, planner_csv = resolve_inputs(args)
    datasets: list[YawHeadingRows] = []

    if sensor_csv and sensor_csv.exists():
        datasets.append(read_csv(sensor_csv, "sensor", args.min_speed_mps))
    if planner_csv and planner_csv.exists():
        datasets.append(read_csv(planner_csv, "planner_10hz", args.min_speed_mps))

    if not datasets:
        print("No yaw/heading CSV files found. Pass --sensor-csv/--planner-csv or check --log-dir/--clip-id.")
        return 2

    output_dir = args.output_dir or (sensor_csv or planner_csv).parent
    output_path = output_dir / f"{args.clip_id}_yaw_heading_plot.png"
    try:
        make_plot(datasets, output_path, args.show)
    except ModuleNotFoundError as exc:
        if exc.name != "matplotlib":
            raise
        output_path = output_dir / f"{args.clip_id}_yaw_heading_plot.svg"
        make_svg_plot(datasets, output_path)
        print("matplotlib is not installed; wrote SVG fallback instead.")

    summary_path = output_dir / f"{args.clip_id}_yaw_heading_summary.txt"
    summary_lines = [summarize(rows) for rows in datasets]
    summary_path.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")

    print(f"wrote {output_path}")
    print(f"wrote {summary_path}")
    for line in summary_lines:
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
