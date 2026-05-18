#!/usr/bin/env python3
"""Generate a simple replayable IMU/GNSS dump for alpamayo_raw_recorder."""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path


HEADER = [
    "offset_ns",
    "type",
    "timestamp_utc_ns",
    "ax",
    "ay",
    "az",
    "gx",
    "gy",
    "gz",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "lat",
    "lon",
    "alt",
    "fix_type",
    "num_sats",
    "hdop",
    "vdop",
    "vx",
    "vy",
    "vz",
    "seq",
    "pc",
    "utc_valid",
]


def build_rows(duration_sec: int, imu_hz: int, gnss_hz: int, start_utc_ns: int) -> list[list[object]]:
    rows: list[list[object]] = []
    base_lat = 37.4275
    base_lon = -122.1697
    base_alt = 32.0

    for i in range(duration_sec * imu_hz):
        offset_ns = round(i * 1_000_000_000 / imu_hz)
        t = i / imu_hz
        utc_ns = start_utc_ns + offset_ns
        ax = 0.2 * math.sin(t * 0.8)
        ay = 0.15 * math.cos(t * 0.5)
        az = 9.81 + 0.05 * math.sin(t * 0.3)
        gx = 0.01 * math.sin(t * 1.3)
        gy = 0.02 * math.cos(t * 0.7)
        gz = 0.015 * math.sin(t * 0.9)
        roll_deg = 1.5 * math.sin(t * 0.4)
        pitch_deg = 0.8 * math.cos(t * 0.3)
        yaw_deg = 15.0 * math.sin(t * 0.1)
        rows.append([
            offset_ns, "imu", utc_ns,
            ax, ay, az, gx, gy, gz,
            roll_deg, pitch_deg, yaw_deg,
            "", "", "",
            "", "", "", "",
            "", "", "",
            i, i % 65536, 3,
        ])

    for i in range(duration_sec * gnss_hz):
        offset_ns = round(i * 1_000_000_000 / gnss_hz)
        t = i / gnss_hz
        utc_ns = start_utc_ns + offset_ns
        lat = base_lat + 0.00001 * t
        lon = base_lon + 0.000015 * t
        alt = base_alt + 0.2 * math.sin(t * 0.2)
        vx = 2.5 + 0.2 * math.sin(t * 0.4)
        vy = 0.1 * math.cos(t * 0.6)
        vz = 0.0
        rows.append([
            offset_ns, "gnss", utc_ns,
            "", "", "", "", "", "",
            "", "", "",
            lat, lon, alt,
            4, 18, 0.7, 1.1,
            vx, vy, vz,
            i, i % 65536, 3,
        ])

    rows.sort(key=lambda row: (int(row[0]), 0 if row[1] == "gnss" else 1))
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration-sec", type=int, default=12)
    parser.add_argument("--imu-hz", type=int, default=100)
    parser.add_argument("--gnss-hz", type=int, default=10)
    parser.add_argument("--start-utc-ns", type=int, default=time.time_ns())
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    rows = build_rows(args.duration_sec, args.imu_hz, args.gnss_hz, args.start_utc_ns)

    with args.output.open("w", encoding="utf-8", newline="") as fh:
      writer = csv.writer(fh)
      writer.writerow(HEADER)
      writer.writerows(rows)

    print(f"wrote {len(rows)} rows to {args.output}")


if __name__ == "__main__":
    main()
