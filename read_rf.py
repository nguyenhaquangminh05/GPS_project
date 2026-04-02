#!/usr/bin/env python3
import serial
import threading
import pynmea2
import time
import csv
import os
from queue import Queue, Empty

# =========================
# CONFIG
# =========================
COM_RTCM_IN = "COM10"   # RF module connected to laptop
COM_ROVER = "COM9"      # USB-UART connected to RTK module

BAUD_RTCM_IN = 57600
BAUD_ROVER = 57600

CSV_FILE = "gps_log.csv"
PRINT_ALL_NMEA = False  # True nếu muốn in mọi câu NMEA ra terminal

# Chỉ lưu khi rover ở RTK FIX
SAVE_ONLY_RTK_FIX = True

# =========================
# GLOBALS
# =========================
rtcm_queue = Queue()
stop_flag = False

rtcm_in_bytes = 0
rtcm_out_bytes = 0
last_stat_ts = time.time()


def init_csv():
    if not os.path.exists(CSV_FILE):
        with open(CSV_FILE, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "pc_timestamp",
                "nmea_type",
                "nmea_utc_time",
                "latitude",
                "longitude",
                "altitude_m",
                "num_sats",
                "hdop",
                "fix_quality",
                "fix_status",
                "raw_nmea"
            ])


def save_to_csv(pc_ts, nmea_type, nmea_time, lat, lon, alt, sats, hdop, fix, status, raw_nmea):
    with open(CSV_FILE, mode="a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            pc_ts,
            nmea_type,
            nmea_time,
            lat,
            lon,
            alt,
            sats,
            hdop,
            fix,
            status,
            raw_nmea
        ])


def fix_quality_to_text(fix):
    fix_map = {
        0: "NO FIX",
        1: "GPS",
        2: "DGPS",
        3: "PPS",
        4: "RTK FIX",
        5: "RTK FLOAT",
        6: "ESTIMATED",
        7: "MANUAL",
        8: "SIMULATION",
    }
    return fix_map.get(fix, f"UNKNOWN({fix})")


def rtcm_input_thread():
    global rtcm_in_bytes, stop_flag

    ser_in = None
    while not stop_flag:
        try:
            if ser_in is None:
                ser_in = serial.Serial(COM_RTCM_IN, BAUD_RTCM_IN, timeout=0.1)
                print(f"[OK] Opened RTCM input {COM_RTCM_IN} @ {BAUD_RTCM_IN}")

            data = ser_in.read(4096)
            if data:
                rtcm_in_bytes += len(data)
                rtcm_queue.put(data)

        except serial.SerialException as e:
            print(f"[RTCM INPUT ERROR] {e}")
            if ser_in is not None:
                try:
                    ser_in.close()
                except Exception:
                    pass
                ser_in = None
            time.sleep(2)

        except Exception as e:
            print(f"[RTCM INPUT UNEXPECTED ERROR] {e}")
            time.sleep(1)

    if ser_in is not None:
        try:
            ser_in.close()
        except Exception:
            pass


def handle_gga(line):
    try:
        msg = pynmea2.parse(line)

        fix = int(msg.gps_qual) if msg.gps_qual not in [None, ""] else 0
        fix_status = fix_quality_to_text(fix)

        nmea_time = str(msg.timestamp) if msg.timestamp else ""
        lat = float(msg.latitude) if msg.latitude not in [None, ""] else 0.0
        lon = float(msg.longitude) if msg.longitude not in [None, ""] else 0.0
        alt = float(msg.altitude) if msg.altitude not in [None, ""] else 0.0
        sats = int(msg.num_sats) if msg.num_sats not in [None, ""] else 0
        hdop = float(msg.horizontal_dil) if msg.horizontal_dil not in [None, ""] else 0.0

        print(
            f"[GGA] utc={nmea_time} | lat={lat:.8f} | lon={lon:.8f} | "
            f"alt={alt:.3f} m | sats={sats} | hdop={hdop:.2f} | "
            f"fix={fix} ({fix_status})"
        )

        should_save = True
        if SAVE_ONLY_RTK_FIX:
            should_save = (fix == 4)

        if should_save:
            save_to_csv(
                pc_ts=time.time(),
                nmea_type="GGA",
                nmea_time=nmea_time,
                lat=lat,
                lon=lon,
                alt=alt,
                sats=sats,
                hdop=hdop,
                fix=fix,
                status=fix_status,
                raw_nmea=line
            )

    except pynmea2.ParseError as e:
        print(f"[WARN] GGA parse error: {e}")
    except Exception as e:
        print(f"[WARN] GGA unexpected parse error: {e}")


def handle_rmc(line):
    try:
        msg = pynmea2.parse(line)

        nmea_time = str(msg.timestamp) if getattr(msg, "timestamp", None) else ""
        lat = float(msg.latitude) if msg.latitude not in [None, ""] else 0.0
        lon = float(msg.longitude) if msg.longitude not in [None, ""] else 0.0
        status_char = getattr(msg, "status", "")
        rmc_status = "VALID" if status_char == "A" else "INVALID"

        print(
            f"[RMC] utc={nmea_time} | lat={lat:.8f} | lon={lon:.8f} | status={rmc_status}"
        )

        # Khong luu RMC vao CSV de file chi con cac diem RTK FIX tu GGA

    except pynmea2.ParseError as e:
        print(f"[WARN] RMC parse error: {e}")
    except Exception as e:
        print(f"[WARN] RMC unexpected parse error: {e}")


def rover_serial_thread():
    global rtcm_out_bytes, last_stat_ts, stop_flag

    ser = None

    while not stop_flag:
        try:
            if ser is None:
                ser = serial.Serial(COM_ROVER, BAUD_ROVER, timeout=0.1, write_timeout=0.5)
                print(f"[OK] Opened rover port {COM_ROVER} @ {BAUD_ROVER}")

            # forward all RTCM bytes from queue -> rover
            while True:
                try:
                    data = rtcm_queue.get_nowait()
                except Empty:
                    break

                try:
                    written = ser.write(data)
                    rtcm_out_bytes += int(written)
                except serial.SerialTimeoutException:
                    print("[WARN] Serial write timeout while sending RTCM to rover.")
                    break

            # print stats every 5s
            now = time.time()
            if now - last_stat_ts > 5.0:
                print(f"[STATS] RTCM in={rtcm_in_bytes} bytes | RTCM out={rtcm_out_bytes} bytes")
                last_stat_ts = now

            # read NMEA back from rover
            if ser.in_waiting:
                line = ser.readline().decode("ascii", errors="ignore").strip()

                if not line:
                    continue

                if PRINT_ALL_NMEA and line.startswith("$"):
                    print(f"[NMEA] {line}")

                if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                    handle_gga(line)

                elif line.startswith("$GNRMC") or line.startswith("$GPRMC"):
                    handle_rmc(line)

            else:
                time.sleep(0.01)

        except serial.SerialException as e:
            print(f"[ROVER SERIAL ERROR] {e}")
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
            time.sleep(2)

        except Exception as e:
            print(f"[ROVER UNEXPECTED ERROR] {e}")
            time.sleep(1)

    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass


def main():
    global stop_flag

    init_csv()

    t1 = threading.Thread(target=rtcm_input_thread, daemon=True)
    t2 = threading.Thread(target=rover_serial_thread, daemon=True)

    t1.start()
    t2.start()

    print("[INFO] Running...")
    print(f"[INFO] RTCM: {COM_RTCM_IN} -> {COM_ROVER}")
    print(f"[INFO] NMEA readback from: {COM_ROVER}")
    print(f"[INFO] CSV log: {CSV_FILE}")
    print("[INFO] Saving only GGA points with RTK FIX (fix_quality = 4)")
    print("[INFO] Press Ctrl+C to stop")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")
        stop_flag = True
        time.sleep(1)


if __name__ == "__main__":
    main()