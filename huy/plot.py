#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt

EARTH_RADIUS_M = 6378137.0


def to_float(value):
    if value is None:
        return None
    try:
        text = str(value).strip()
        if not text:
            return None
        return float(text)
    except ValueError:
        return None


def to_int(value):
    if value is None:
        return None
    try:
        text = str(value).strip()
        if not text:
            return None
        return int(float(text))
    except ValueError:
        return None


def normalize_fieldnames(fieldnames):
    if fieldnames is None:
        return {}
    return {name.strip().lstrip("\ufeff"): name for name in fieldnames}


def read_points_from_csv(csv_path: Path, only_fix=5, only_status=None):
    points = []

    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)

        if reader.fieldnames is None:
            raise ValueError("CSV khong co header hop le.")

        field_map = normalize_fieldnames(reader.fieldnames)

        required = ["latitude", "longitude"]
        for key in required:
            if key not in field_map:
                raise ValueError(
                    f"Khong tim thay cot '{key}'. Cac cot hien co: {reader.fieldnames}"
                )

        lat_key = field_map["latitude"]
        lon_key = field_map["longitude"]
        fix_key = field_map.get("fix_quality", None)
        status_key = field_map.get("fix_status", None)

        for row in reader:
            lat = to_float(row.get(lat_key))
            lon = to_float(row.get(lon_key))

            if lat is None or lon is None:
                continue

            if only_fix is not None and fix_key is not None:
                fix_val = to_int(row.get(fix_key))
                if fix_val != only_fix:
                    continue

            if only_status is not None and status_key is not None:
                status_val = str(row.get(status_key, "")).strip().upper()
                if status_val != only_status.strip().upper():
                    continue

            points.append((lat, lon))

    return points


def lat_lon_to_local_xy_m(points):
    if not points:
        return []

    lat0_deg, lon0_deg = points[0]
    lat0_rad = math.radians(lat0_deg)

    xy = []
    for lat_deg, lon_deg in points:
        dlat = math.radians(lat_deg - lat0_deg)
        dlon = math.radians(lon_deg - lon0_deg)

        x = EARTH_RADIUS_M * dlon * math.cos(lat0_rad)
        y = EARTH_RADIUS_M * dlat
        xy.append((x, y))

    return xy


def print_stats(points, xy_points):
    x_values = [p[0] for p in xy_points]
    y_values = [p[1] for p in xy_points]

    print(f"So diem hop le: {len(points)}")
    print(f"Diem dau : lat={points[0][0]:.10f}, lon={points[0][1]:.10f}")
    print(f"Diem cuoi: lat={points[-1][0]:.10f}, lon={points[-1][1]:.10f}")
    print(f"X min/max: {min(x_values):.3f} / {max(x_values):.3f} m")
    print(f"Y min/max: {min(y_values):.3f} / {max(y_values):.3f} m")
    print(f"Do rong X: {max(x_values) - min(x_values):.3f} m")
    print(f"Do rong Y: {max(y_values) - min(y_values):.3f} m")


def plot_xy_path(xy_points, title: str):
    if not xy_points:
        raise ValueError("No valid GPS points to plot.")

    x_values = [p[0] for p in xy_points]
    y_values = [p[1] for p in xy_points]

    plt.figure(figsize=(8, 6))
    plt.plot(x_values, y_values, "b-", linewidth=1.5, label="RTK FLOAT path")
    plt.scatter(x_values[0], y_values[0], c="green", s=70, label="Start", zorder=3)
    plt.scatter(x_values[-1], y_values[-1], c="red", s=70, label="End", zorder=3)

    plt.title(title)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.axis("equal")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()


def main():
    parser = argparse.ArgumentParser(
        description="Plot GPS path from csv, default only RTK FLOAT points."
    )
    parser.add_argument(
        "csv_file",
        nargs="?",
        default="gps_log.csv",
        help="Path to CSV file (default: gps_log.csv)",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Ve tat ca diem, khong chi RTK FLOAT",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    only_fix = None if args.all else 5

    points = read_points_from_csv(csv_path, only_fix=only_fix, only_status=None)

    if len(points) < 2:
        raise ValueError(
            f"Need at least 2 valid latitude/longitude points to plot. Found {len(points)} points."
        )

    xy_points = lat_lon_to_local_xy_m(points)
    print_stats(points, xy_points)

    title = f"GPS Path (local XY) - {csv_path.name}"
    if not args.all:
        title += " | only RTK FLOAT"

    plot_xy_path(xy_points, title=title)
    plt.show()


if __name__ == "__main__":
    main()
