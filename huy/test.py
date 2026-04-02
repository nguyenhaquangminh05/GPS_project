#!/usr/bin/env python3
import csv
import os
import serial
import threading
import time
from queue import Queue, Empty

import pynmea2

# =========================
# CONFIG
# =========================
BASE_RTCM_PORT = "/dev/ttyACM0"   # Base: output RTCM
ROVER_PORT = "/dev/ttyACM1"       # Rover: input RTCM + output NMEA

BAUD_BASE = 115200
BAUD_ROVER = 115200

CSV_FILE = "gps_log.csv"
PRINT_ALL_RTCM = True

# Chi log diem RTK FLOAT
SAVE_ONLY_RTK_FLOAT = True

# =========================
# GLOBALS
# =========================
rtcm_frame_queue = Queue()
stop_flag = False

rtcm_in_frames = 0
rtcm_out_frames = 0
rtcm_in_bytes = 0
rtcm_out_bytes = 0
partial_write_count = 0
write_timeout_count = 0
last_stat_ts = time.time()

csv_lock = threading.Lock()
stat_lock = threading.Lock()


# =========================
# CSV
# =========================
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
                "raw_gga",
            ])


def save_to_csv(pc_ts, nmea_time, lat, lon, alt, sats, hdop, fix, status, dgps_age, base_id, raw_gga):
    with csv_lock:
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
                raw_gga,
            ])


# =========================
# RTCM HELPERS
# =========================
def get_rtcm_payload_length(buf: bytearray):
    if len(buf) < 3:
        return None
    return ((buf[1] & 0x03) << 8) | buf[2]


def get_rtcm_message_type(frame: bytes):
    if len(frame) < 8:
        return None
    payload = frame[3:-3]
    if len(payload) < 2:
        return None
    return ((payload[0] << 4) | (payload[1] >> 4)) & 0x0FFF


def extract_rtcm_frames(buffer: bytearray):
    frames = []

    while True:
        start = buffer.find(b"\xD3")
        if start < 0:
            buffer.clear()
            break

        if start > 0:
            del buffer[:start]

        if len(buffer) < 3:
            break

        payload_len = get_rtcm_payload_length(buffer)
        if payload_len is None:
            break

        total_len = 3 + payload_len + 3

        if len(buffer) < total_len:
            break

        frame = bytes(buffer[:total_len])
        frames.append(frame)
        del buffer[:total_len]

    return frames


def write_all(ser, data: bytes):
    global partial_write_count, write_timeout_count

    total_sent = 0
    total_len = len(data)

    while total_sent < total_len and not stop_flag:
        try:
            n = ser.write(data[total_sent:])
            if n is None:
                n = 0

            if n <= 0:
                raise serial.SerialTimeoutException("write returned 0 bytes")

            if n < (total_len - total_sent):
                with stat_lock:
                    partial_write_count += 1

            total_sent += n

        except serial.SerialTimeoutException:
            with stat_lock:
                write_timeout_count += 1
            time.sleep(0.01)

    return total_sent


# =========================
# STATS
# =========================
def print_stats():
    with stat_lock:
        ratio = 0.0
        if rtcm_in_bytes > 0:
            ratio = 100.0 * rtcm_out_bytes / rtcm_in_bytes

        print(
            f"[STATS] "
            f"IN: {rtcm_in_frames} frames / {rtcm_in_bytes} bytes | "
            f"OUT: {rtcm_out_frames} frames / {rtcm_out_bytes} bytes | "
            f"OUT/IN: {ratio:.2f}% | "
            f"partial_write={partial_write_count} | "
            f"write_timeout={write_timeout_count} | "
            f"queue={rtcm_frame_queue.qsize()}"
        )


# =========================
# THREAD 1: READ COMPLETE RTCM FRAMES FROM BASE
# =========================
def rtcm_input_thread():
    global rtcm_in_frames, rtcm_in_bytes

    ser_in = None
    rx_buffer = bytearray()

    while not stop_flag:
        try:
            if ser_in is None:
                ser_in = serial.Serial(BASE_RTCM_PORT, BAUD_BASE, timeout=0.1)
                print(f"[OK] Opened base RTCM input {BASE_RTCM_PORT} @ {BAUD_BASE}")

            data = ser_in.read(4096)
            if not data:
                continue

            rx_buffer.extend(data)

            frames = extract_rtcm_frames(rx_buffer)
            for frame in frames:
                with stat_lock:
                    rtcm_in_frames += 1
                    rtcm_in_bytes += len(frame)

                rtcm_frame_queue.put(frame)

                if PRINT_ALL_RTCM:
                    msg_type = get_rtcm_message_type(frame)
                    payload_len = len(frame) - 6
                    print(f"[IN ] RTCM {msg_type} | payload={payload_len} | total={len(frame)}")

        except serial.SerialException as e:
            print(f"[BASE RTCM ERROR] {e}")
            if ser_in is not None:
                try:
                    ser_in.close()
                except Exception:
                    pass
                ser_in = None
            time.sleep(1)

        except Exception as e:
            print(f"[BASE RTCM UNEXPECTED ERROR] {e}")
            time.sleep(1)

    if ser_in is not None:
        try:
            ser_in.close()
        except Exception:
            pass


# =========================
# THREAD 2: SEND RTCM TO ROVER + READ NMEA FROM ROVER
# =========================
def rover_io_thread():
    global rtcm_out_frames, rtcm_out_bytes, last_stat_ts

    ser = None

    while not stop_flag:
        try:
            if ser is None:
                ser = serial.Serial(ROVER_PORT, BAUD_ROVER, timeout=0.1, write_timeout=0.5)
                print(f"[OK] Opened rover port {ROVER_PORT} @ {BAUD_ROVER}")

            # 1) Gui RTCM xuong rover
            while True:
                try:
                    frame = rtcm_frame_queue.get_nowait()
                except Empty:
                    break

                written = write_all(ser, frame)
                if written == len(frame):
                    with stat_lock:
                        rtcm_out_frames += 1
                        rtcm_out_bytes += written

                    if PRINT_ALL_RTCM:
                        msg_type = get_rtcm_message_type(frame)
                        payload_len = len(frame) - 6
                        print(f"[OUT] RTCM {msg_type} | payload={payload_len} | total={written}")
                else:
                    print(f"[WARN] Incomplete frame write: {written}/{len(frame)} bytes")

            # 2) Doc GGA tu rover
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
                        dgps_age = getattr(msg, "age_gps_data", "")
                        base_id = getattr(msg, "ref_station_id", "")

                        print(
                            f"[GGA] time={nmea_time} | lat={lat:.8f} | lon={lon:.8f} | "
                            f"alt={alt:.3f} | sats={sats} | hdop={hdop:.2f} | "
                            f"fix={status} | dgps_age={dgps_age} | base_id={base_id}"
                        )

                        should_save = False
                        if SAVE_ONLY_RTK_FLOAT:
                            should_save = (fix == 5)
                        else:
                            should_save = (fix > 0)

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
                                raw_gga=line,
                            )

                    except pynmea2.ParseError as e:
                        print(f"[WARN] NMEA parse error: {e}")
                    except (ValueError, TypeError) as e:
                        print(f"[WARN] NMEA conversion error: {e}")

            else:
                time.sleep(0.01)

            # 3) In thong ke
            now = time.time()
            if now - last_stat_ts > 5.0:
                print_stats()
                last_stat_ts = now

        except serial.SerialException as e:
            print(f"[ROVER IO ERROR] {e}")
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
            time.sleep(1)

        except Exception as e:
            print(f"[ROVER IO UNEXPECTED ERROR] {e}")
            time.sleep(1)

    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass


# =========================
# MAIN
# =========================
def main():
    global stop_flag

    init_csv()

    t1 = threading.Thread(target=rtcm_input_thread, daemon=True)
    t2 = threading.Thread(target=rover_io_thread, daemon=True)

    t1.start()
    t2.start()

    print("[INFO] Running...")
    print(f"[INFO] Read complete RTCM frames from base: {BASE_RTCM_PORT}")
    print(f"[INFO] Send RTCM to rover and read NMEA back on: {ROVER_PORT}")
    print(f"[INFO] Logging only RTK FLOAT points to: {CSV_FILE}")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")
        stop_flag = True
        time.sleep(1)


if __name__ == "__main__":
    main()
