#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import threading
import pynmea2
import time
import socket
import queue
import base64
import csv
import os

class GpsRtkBridgeNode(Node):
    def __init__(self):
        super().__init__('gps_rtk_bridge')

        # Parameters uBlox
        self.declare_parameter('device', '/dev/ttyACM0')
        self.declare_parameter('baud', 38400)

        # Parameters RTCM
        self.declare_parameter('rtcm_server', '192.168.4.1')
        self.declare_parameter('rtcm_port', 2101)
        self.declare_parameter('mount_point', 'KINDHELM')
        self.declare_parameter('username', '')
        self.declare_parameter('password', '')

        self.device = self.get_parameter('device').value
        self.baud = self.get_parameter('baud').value
        self.rtcm_server = self.get_parameter('rtcm_server').value
        self.rtcm_port = self.get_parameter('rtcm_port').value
        self.mount_point = self.get_parameter('mount_point').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value

        self.rtcm_in_bytes = 0
        self.rtcm_out_bytes = 0
        self.last_stat_ts = time.time()

        # ROS publisher
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)

        # Record data - longitude, latitude, altitude, timestamp
        self.csv_file = 'gps_data.csv'
        self.init_csv()

        self.rtcm_queue = queue.Queue()

        # Theading
        threading.Thread(target=self.rtcm_socket_thread, daemon=True).start()
        threading.Thread(target=self.serial_manager_thread, daemon=True).start()

    def init_csv(self):
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude'])

    def save_to_csv(self, timestamp, lat, lon, alt):
        with open(self.csv_file, "a", newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, lat, lon, alt])

    # NTRIP request builder
    def ntrip_request(self):
        mount = self.mount_point.strip()
        request = f"GET /{mount} HTTP/1.0\r\n"
        request += f"Host: {self.rtcm_server}:{self.rtcm_port}\r\n"
        if self.username and self.password:
            auth = base64.b64encode(f"{self.username}:{self.password}".encode()).decode()
            request += f"Authorization: Basic {auth}\r\n"
        request += "\r\n"
        return request
    
    # RTCM socket thread 
    def rtcm_socket_thread(self):
        while rclpy.ok():
            sock = None
            try:
                self.get_logger().info(f"Connecting to RTCM server {self.rtcm_server}:{self.rtcm_port}...")
                sock = socket.create_connection((self.rtcm_server, self.rtcm_port), timeout=10)
                sock.sendall(self.ntrip_request().encode())

                response = sock.recv(4096)
                self.get_logger().info(f"NTRIP responde: {response[:80]!r}")
                if b"200" not in response:
                    raise Exception("NTRIP server not response =))")
                
                sock.settimeout(None)  # Set to blocking mode
                
                while rclpy.ok():
                    data = sock.recv(4096)
                    if not data:
                        self.get_logger().warning("RTCM server closed the connection.")
                        break
                    self.rtcm_in_bytes += len(data)
                    self.rtcm_queue.put(data)
            except Exception as e:
                self.get_logger().error(f"RTCM socket error: {e}")
                time.sleep(5)  # Wait 5s before retrying
            finally:
                if sock:
                    sock.close()
                    self.get_logger().info("RTCM socket closed.")
    

    # Serial thread
    def serial_manager_thread(self):
        while rclpy.ok():
            try:
                ser = serial.Serial(self.device, self.baud, timeout=0.1, write_timeout=0.5)
                self.get_logger().info(f"Connected to GPS device {self.device} at {self.baud} baud.")

                while rclpy.ok():
                    # Send RTCM data to GPS
                    while not self.rtcm_queue.empty():
                        data = self.rtcm_queue.get()
                        try:
                            written = ser.write(data)
                            self.rtcm_out_bytes += int(written)
                        except serial.SerialTimeoutException:
                            self.get_logger().warning("Serial write timeout.")
                            break
                    
                    now = time.time()
                    if now - self.last_stat_ts > 5.0:
                        self.get_logger().info(f"RTCM in: {self.rtcm_in_bytes} bytes, out: {self.rtcm_out_bytes} bytes")
                        self.last_stat_ts = now
                    
                    # Read GPS data
                    if ser.in_waiting:
                        line = ser.readline().decode(encoding='ascii', errors='ignore').strip()

                        if line.startswith('$GNGGA'):
                            try:
                                msg = pynmea2.parse(line)
                                fix = int(msg.gps_qual)
                                fix_types = {
                                    0: "NO FIX",
                                    1: "SINGLE",
                                    2: "DGPS",
                                    4: "RTK FIX",
                                    5: "RTK FLOAT"
                                }
                                status = fix_types.get(fix, f"UNKNOWN({fix})")

                                # NMEA data
                                time_utc = msg.timestamp
                                lat = msg.latitude
                                lon = msg.longitude
                                alt = float(msg.altitude or 0)
                                sats = int(msg.num_sats or 0)
                                hdop = float(msg.horizontal_dil or 0)
                                geoid = float(msg.geo_sep or 0)
                                dgps_age = msg.age_gps_data
                                base_id = msg.ref_station_id

                                # Print debug info
                                print("\n--- GGA DEBUG ---")
                                print(f"Time (UTC): {time_utc}")
                                print(f"Latitude: {lat}")
                                print(f"Longitude: {lon}")
                                print(f"Altitude: {alt}")
                                print(f"Number of Satellites: {sats}")
                                print(f"HDOP: {hdop}")
                                print(f"Geoid Separation: {geoid}")
                                print(f"DGPS Age: {dgps_age}")
                                print(f"Base Station ID: {base_id}")
                                print(f"Fix Status: {status}")

                                # Quality check
                                if fix == 0:
                                    print("No GPS fix")
                                elif fix == 1:
                                    print("SINGLE")
                                elif fix == 2:
                                    print("DGPS")
                                elif fix == 4:
                                    print("RTK FIX")
                                elif fix == 5:
                                    print("RTK FLOAT")
                                
                                if dgps_age:
                                    print(f"DGPS data age: {dgps_age} seconds, RTCM OK")
                                else:
                                    print("RTCM data not received or too old")
                                
                                # Save to CSV
                                if fix == 4:
                                    self.save_to_csv(time.time(), lat, lon, alt)
                                
                                # Publish ROS message
                                if fix > 0:
                                    msg = NavSatFix()
                                    msg.header.stamp = self.get_clock().now().to_msg()
                                    msg.latitude = lat
                                    msg.longitude = lon
                                    msg.altitude = alt
                                    self.publisher_.publish(msg)
                            except pynmea2.ParseError as e:
                                self.get_logger().warning(f"NMEA parse error: {e}")
                            except (ValueError, TypeError) as e:
                                self.get_logger().warning(f"NMEA data conversion error: {e}")
                    else:
                        time.sleep(0.01)
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}")
                time.sleep(5)  # Wait before retrying

def main(args=None):
    rclpy.init(args=args)
    node = GpsRtkBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()