#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

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


def read_lat_lon_from_csv(csv_path: Path, only_fix=4):
    points = []

    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)

        if reader.fieldnames is None:
            raise ValueError("CSV khong co header hop le.")

        # chong BOM / khoang trang
        field_map = {name.strip().lstrip("\ufeff"): name for name in reader.fieldnames}

        if "latitude" not in field_map or "longitude" not in field_map:
            raise ValueError(
                f"Khong tim thay cot latitude/longitude. Cac cot hien co: {reader.fieldnames}"
            )

        lat_key = field_map["latitude"]
        lon_key = field_map["longitude"]
        fix_key = field_map.get("fix_quality", None)

        for row in reader:
            lat = to_float(row.get(lat_key))
            lon = to_float(row.get(lon_key))

            if lat is None or lon is None:
                continue

            if only_fix is not None and fix_key is not None:
                fix_val = to_int(row.get(fix_key))
                if fix_val != only_fix:
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
    print(f"Diem dau : lat={points[0][0]:.8f}, lon={points[0][1]:.8f}")
    print(f"Diem cuoi: lat={points[-1][0]:.8f}, lon={points[-1][1]:.8f}")
    print(f"X min/max: {min(x_values):.3f} / {max(x_values):.3f} m")
    print(f"Y min/max: {min(y_values):.3f} / {max(y_values):.3f} m")
    print(f"Do rong X: {max(x_values) - min(x_values):.3f} m")
    print(f"Do rong Y: {max(y_values) - min(y_values):.3f} m")


def ascii_plot(xy_points, width=80, height=25):
    x_values = [p[0] for p in xy_points]
    y_values = [p[1] for p in xy_points]

    min_x, max_x = min(x_values), max(x_values)
    min_y, max_y = min(y_values), max(y_values)

    if abs(max_x - min_x) < 1e-12:
        max_x += 1.0
        min_x -= 1.0
    if abs(max_y - min_y) < 1e-12:
        max_y += 1.0
        min_y -= 1.0

    canvas = [[" " for _ in range(width)] for _ in range(height)]

    for i, (x, y) in enumerate(xy_points):
        cx = int((x - min_x) / (max_x - min_x) * (width - 1))
        cy = int((y - min_y) / (max_y - min_y) * (height - 1))
        cy = height - 1 - cy

        ch = "."
        if i == 0:
            ch = "S"
        elif i == len(xy_points) - 1:
            ch = "E"

        canvas[cy][cx] = ch

    print("\nQuy dao tuong doi tren terminal:")
    print("S = Start, E = End\n")
    for row in canvas:
        print("".join(row))


def main():
    parser = argparse.ArgumentParser(
        description="Read GPS CSV and show local XY stats + ASCII plot."
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
        help="Lay tat ca diem, khong chi RTK FIX",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    only_fix = None if args.all else 4

    points = read_lat_lon_from_csv(csv_path, only_fix=only_fix)
    if len(points) < 2:
        raise ValueError(
            f"Need at least 2 valid latitude/longitude points. Found {len(points)} points."
        )

    xy_points = lat_lon_to_local_xy_m(points)

    print_stats(points, xy_points)
    ascii_plot(xy_points)


if __name__ == "__main__":
    main()