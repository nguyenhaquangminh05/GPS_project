#!/usr/bin/env python3
import serial
import time

COM_PORT = "COM6"
BAUD_RATE = 115200
OUT_FILE = "rtcm_frames.txt"


def get_rtcm_message_type(frame: bytes):
    # frame = D3 + 2-byte header + payload + 3-byte CRC
    # message type nằm ở 12 bit đầu của payload
    if len(frame) < 8:
        return None
    payload = frame[3:-3]
    if len(payload) < 2:
        return None
    msg_type = ((payload[0] << 4) | (payload[1] >> 4)) & 0xFFF
    return msg_type


def extract_rtcm_frames(buffer: bytearray):
    frames = []

    while True:
        # tìm preamble 0xD3
        start = buffer.find(b'\xD3')
        if start < 0:
            buffer.clear()
            break

        # bỏ rác trước preamble
        if start > 0:
            del buffer[:start]

        # cần ít nhất 3 byte để đọc header
        if len(buffer) < 3:
            break

        # RTCM3 length = 10 bit thấp của 2 byte header
        length = ((buffer[1] & 0x03) << 8) | buffer[2]
        total_len = 3 + length + 3

        if len(buffer) < total_len:
            break

        frame = bytes(buffer[:total_len])
        frames.append(frame)
        del buffer[:total_len]

    return frames


def main():
    buf = bytearray()

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
        print(f"[OK] Opened {COM_PORT} @ {BAUD_RATE}")
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open {COM_PORT}: {e}")
        return

    with open(OUT_FILE, "w", encoding="utf-8") as f:
        f.write("timestamp | msg_type | payload_len | total_len | hex\n")

        try:
            while True:
                data = ser.read(4096)
                if not data:
                    continue

                buf.extend(data)
                frames = extract_rtcm_frames(buf)

                for frame in frames:
                    payload_len = ((frame[1] & 0x03) << 8) | frame[2]
                    total_len = len(frame)
                    msg_type = get_rtcm_message_type(frame)
                    ts = time.strftime("%H:%M:%S")

                    line = f"{ts} | {msg_type} | {payload_len} | {total_len} | {frame.hex().upper()}\n"
                    f.write(line)
                    f.flush()

                    print(f"[{ts}] RTCM {msg_type} | payload={payload_len} | total={total_len}")

        except KeyboardInterrupt:
            print("\n[INFO] Stopped.")
        finally:
            ser.close()
            print(f"[INFO] Saved to {OUT_FILE}")


if __name__ == "__main__":
    main()