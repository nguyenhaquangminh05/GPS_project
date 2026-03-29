#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt

EARTH_RADIUS_M = 6378137.0


def _to_float(value):
    if value is None:
        return None
    try:
        text = str(value).strip()
        if not text:
            return None
        return float(text)
    except ValueError:
        return None


def read_lat_lon_from_csv(csv_path: Path):
    points = []

    with csv_path.open("r", newline="") as f:
        sample = f.read(2048)
        f.seek(0)

        has_header = csv.Sniffer().has_header(sample)
        if has_header:
            reader = csv.DictReader(f)
            for row in reader:
                lat = _to_float(row.get("latitude"))
                lon = _to_float(row.get("longitude"))
                if lat is not None and lon is not None:
                    points.append((lat, lon))
        else:
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 2:
                    continue
                lat = _to_float(row[0])
                lon = _to_float(row[1])
                if lat is not None and lon is not None:
                    points.append((lat, lon))

    return points


def lat_lon_to_local_xy_m(points):
    if not points:
        return []

    lat0_deg, lon0_deg = points[0]
    lat0_rad = math.radians(lat0_deg)
    lon0_rad = math.radians(lon0_deg)

    xy = []
    for lat_deg, lon_deg in points:
        lat_rad = math.radians(lat_deg)
        lon_rad = math.radians(lon_deg)

        dlat = lat_rad - lat0_rad
        dlon = lon_rad - lon0_rad

        x = EARTH_RADIUS_M * dlon * math.cos(lat0_rad)
        y = EARTH_RADIUS_M * dlat
        xy.append((x, y))

    return xy


def plot_xy_path(xy_points, title: str):
    if not xy_points:
        raise ValueError("No valid GPS points to plot.")

    x_values = [p[0] for p in xy_points]
    y_values = [p[1] for p in xy_points]

    plt.figure(figsize=(8, 6))
    plt.plot(x_values, y_values, "b-", linewidth=1.5, label="Recorded path")
    plt.scatter(x_values[0], y_values[0], c="green", s=60, label="Start", zorder=3)
    plt.scatter(x_values[-1], y_values[-1], c="red", s=60, label="End", zorder=3)

    plt.title(title)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.axis("equal")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()


def main():
    parser = argparse.ArgumentParser(
        description="Plot recorded GPS path from CSV after converting lat/lon to local XY (meters)."
    )
    parser.add_argument(
        "csv_file",
        nargs="?",
        default="gps_data.csv",
        help="Path to CSV file (default: gps_data.csv)",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    points = read_lat_lon_from_csv(csv_path)
    if len(points) < 2:
        raise ValueError("Need at least 2 valid latitude/longitude points to plot.")

    xy_points = lat_lon_to_local_xy_m(points)
    plot_xy_path(xy_points, title=f"GPS Path (local XY) - {csv_path.name}")

    plt.show()


if __name__ == "__main__":
    main()
