#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2
import threading
import asyncio
import json
from aiohttp import web

# --- GIAO DIỆN (Dùng Leaflet - Miễn phí) ---
HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8" />
    <title>ROS2 Robot Tracker</title>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        html, body { margin: 0; padding: 0; height: 100%; font-family: sans-serif; }
        #map { width: 100%; height: calc(100% - 40px); }
        #status { padding: 10px; background: #34495e; color: white; font-size: 14px; }
    </style>
</head>
<body>
    <div id="status">Status: <span id="ws-status">Offline</span> | Lat: <span id="lat">--</span> | Lon: <span id="lon">--</span></div>
    <div id="map"></div>
    <script>
        const map = L.map('map').setView([21.0285, 105.8542], 16);
        L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
        const marker = L.marker([21.0285, 105.8542]).addTo(map);
        const path = L.polyline([], {color: 'blue'}).addTo(map);
        
        const ws = new WebSocket(`ws://${location.host}/ws`);
        ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (!data.lat) return;
            document.getElementById('ws-status').textContent = "Online";
            document.getElementById('lat').textContent = data.lat.toFixed(6);
            document.getElementById('lon').textContent = data.lon.toFixed(6);
            
            const pos = [data.lat, data.lon];
            marker.setLatLng(pos);
            path.addLatLng(pos);
            map.panTo(pos);
        };
    </script>
</body>
</html>
"""

class GpsBridgeNode(Node):
    def __init__(self):
        super().__init__('gps_bridge_node')
        
        # 1. Khai báo Parameters
        self.declare_parameter('device', '/dev/ttyACM0')
        self.declare_parameter('baud', 38400)
        
        self.device = self.get_parameter('device').value
        self.baud = self.get_parameter('baud').value
        
        # 2. ROS 2 Publisher (Để các node khác dùng)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # 3. Quản lý Web
        self.ws_clients = set()
        self.latest_data = {}

        # 4. Chạy luồng đọc Serial
        threading.Thread(target=self.serial_thread, daemon=True).start()
        
        # 5. Chạy Web Server
        threading.Thread(target=self.start_web_server, daemon=True).start()
        self.get_logger().info("GPS Bridge Node has started!")

    def serial_thread(self):
        try:
            ser = serial.Serial(self.device, self.baud, timeout=1)
            while rclpy.ok():
                if ser.in_waiting:
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith('$GNGGA'):
                        try:
                            msg_nmea = pynmea2.parse(line)
                            if msg_nmea.latitude != 0:
                                # Publish lên ROS 2
                                ros_msg = NavSatFix()
                                ros_msg.header.stamp = self.get_clock().now().to_msg()
                                ros_msg.header.frame_id = "gps_link"
                                ros_msg.latitude = msg_nmea.latitude
                                ros_msg.longitude = msg_nmea.longitude
                                ros_msg.altitude = float(msg_nmea.altitude)
                                self.gps_pub.publish(ros_msg)
                                
                                # Lưu data cho Web
                                self.latest_data = {'lat': msg_nmea.latitude, 'lon': msg_nmea.longitude}
                        except Exception as e:
                            pass
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")

    # --- Phần xử lý WebServer ---
    def start_web_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        app = web.Application()
        app.add_routes([web.get('/', self.handle_index), web.get('/ws', self.websocket_handler)])
        
        # Chạy task gửi data qua WebSocket định kỳ
        async def broadcast_loop():
            while True:
                if self.latest_data and self.ws_clients:
                    data = json.dumps(self.latest_data)
                    for ws in list(self.ws_clients):
                        try: await ws.send_str(data)
                        except: self.ws_clients.discard(ws)
                await asyncio.sleep(0.2)

        loop.create_task(broadcast_loop())
        runner = web.AppRunner(app)
        loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, '0.0.0.0', 8080)
        loop.run_until_complete(site.start())
        loop.run_forever()

    async def handle_index(self, request): return web.Response(text=HTML_PAGE, content_type='text/html')

    async def websocket_handler(self, request):
        ws = web.WebSocketResponse(); await ws.prepare(request)
        self.ws_clients.add(ws)
        try:
            async for _ in ws: pass
        finally: self.ws_clients.discard(ws)
        return ws

def main():
    rclpy.init()
    node = GpsBridgeNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__':
    main()

