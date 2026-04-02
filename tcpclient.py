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
COM_RTCM_IN = "COM7"      # Base output RTCM
COM_ROVER = "COM8"        # Rover port
BAUD_RTCM_IN = 115200
BAUD_ROVER = 115200

CSV_FILE = "gps_log.csv"
LOG_ONLY_RTK_FIX = False   # True: chỉ lưu fix=4, False: lưu mọi fix > 0

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
                "nmea_utc_time",
                "latitude",
                "longitude",
                "altitude",
                "num_sats",
                "hdop",
                "fix_quality",
                "fix_status",
                "dgps_age",
                "base_station_id",
            ])


def save_to_csv(pc_ts, nmea_time, lat, lon, alt, sats, hdop, fix, status, dgps_age, base_id):
    with open(CSV_FILE, mode="a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            pc_ts,
            nmea_time,
            lat,
            lon,
            alt,
            sats,
            hdop,
            fix,
            status,
            dgps_age,
            base_id,
        ])


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


def rover_serial_thread():
    global rtcm_out_bytes, last_stat_ts, stop_flag

    ser = None

    while not stop_flag:
        try:
            if ser is None:
                ser = serial.Serial(COM_ROVER, BAUD_ROVER, timeout=0.1, write_timeout=0.5)
                print(f"[OK] Opened rover port {COM_ROVER} @ {BAUD_ROVER}")

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

            now = time.time()
            if now - last_stat_ts > 5.0:
                print(f"[STATS] RTCM in: {rtcm_in_bytes} bytes | RTCM out: {rtcm_out_bytes} bytes")
                last_stat_ts = now

            if ser.in_waiting:
                line = ser.readline().decode("ascii", errors="ignore").strip()

                if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                    try:
                        msg = pynmea2.parse(line)

                        fix = int(msg.gps_qual)
                        fix_types = {
                            0: "NO FIX",
                            1: "SINGLE",
                            2: "DGPS",
                            4: "RTK FIX",
                            5: "RTK FLOAT",
                        }
                        status = fix_types.get(fix, f"UNKNOWN({fix})")

                        nmea_time = str(msg.timestamp) if msg.timestamp else ""
                        lat = msg.latitude
                        lon = msg.longitude
                        alt = float(msg.altitude or 0.0)
                        sats = int(msg.num_sats or 0)
                        hdop = float(msg.horizontal_dil or 0.0)
                        dgps_age = msg.age_gps_data if hasattr(msg, "age_gps_data") else ""
                        base_id = msg.ref_station_id if hasattr(msg, "ref_station_id") else ""

                        print(
                            f"[GGA] time={nmea_time} | lat={lat:.8f} | lon={lon:.8f} | "
                            f"alt={alt:.3f} | sats={sats} | hdop={hdop} | fix={status}"
                        )

                        should_save = False
                        if LOG_ONLY_RTK_FIX:
                            if fix == 4:
                                should_save = True
                        else:
                            if fix > 0:
                                should_save = True

                        if should_save:
                            save_to_csv(
                                pc_ts=time.time(),
                                nmea_time=nmea_time,
                                lat=lat,
                                lon=lon,
                                alt=alt,
                                sats=sats,
                                hdop=hdop,
                                fix=fix,
                                status=status,
                                dgps_age=dgps_age,
                                base_id=base_id,
                            )

                    except pynmea2.ParseError as e:
                        print(f"[WARN] NMEA parse error: {e}")
                    except (ValueError, TypeError) as e:
                        print(f"[WARN] NMEA conversion error: {e}")

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
    print(f"[INFO] Read RTCM from {COM_RTCM_IN}, send to rover on {COM_ROVER}, read NMEA back from {COM_ROVER}")
    print(f"[INFO] Logging CSV to: {CSV_FILE}")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")
        stop_flag = True
        time.sleep(1)


if __name__ == "__main__":
    main()