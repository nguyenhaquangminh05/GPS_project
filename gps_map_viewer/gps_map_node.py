#!/usr/bin/env python3
import asyncio
import json
import threading
import webbrowser

from aiohttp import web
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <title>ROS2 GPS Realtime Map</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">

  <link
    rel="stylesheet"
    href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    crossorigin=""
  />
  <script
    src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
    crossorigin=""
  ></script>

  <style>
    html, body {
      margin: 0;
      padding: 0;
      height: 100%;
      font-family: Arial, sans-serif;
    }

    #topbar {
      padding: 10px 14px;
      background: #f5f5f5;
      border-bottom: 1px solid #ddd;
      font-size: 14px;
    }

    #map {
      width: 100%;
      height: calc(100% - 48px);
    }

    .status {
      font-weight: bold;
    }
  </style>
</head>
<body>
  <div id="topbar">
    <span class="status" id="ws-status">Connecting...</span>
    <span style="margin-left: 16px;">Lat: <span id="lat-val">--</span></span>
    <span style="margin-left: 16px;">Lon: <span id="lon-val">--</span></span>
    <span style="margin-left: 16px;">Alt: <span id="alt-val">--</span></span>
  </div>

  <div id="map"></div>

  <script>
    const defaultLat = 21.028511;
    const defaultLon = 105.852020;

    const map = L.map('map').setView([defaultLat, defaultLon], 16);

    L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
      attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    const marker = L.marker([defaultLat, defaultLon]).addTo(map);
    const pathLine = L.polyline([[defaultLat, defaultLon]], {weight: 4}).addTo(map);

    let firstFix = true;
    let trail = [];

    const wsStatus = document.getElementById('ws-status');
    const latVal = document.getElementById('lat-val');
    const lonVal = document.getElementById('lon-val');
    const altVal = document.getElementById('alt-val');

    function connectWS() {
      const ws = new WebSocket(`ws://${location.host}/ws`);

      ws.onopen = () => {
        wsStatus.textContent = 'WebSocket connected';
      };

      ws.onclose = () => {
        wsStatus.textContent = 'WebSocket disconnected - retrying...';
        setTimeout(connectWS, 1000);
      };

      ws.onerror = () => {
        wsStatus.textContent = 'WebSocket error';
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);

          if (data.lat === null || data.lon === null) {
            return;
          }

          const lat = data.lat;
          const lon = data.lon;
          const alt = data.alt;

          latVal.textContent = lat.toFixed(7);
          lonVal.textContent = lon.toFixed(7);
          altVal.textContent = Number.isFinite(alt) ? alt.toFixed(2) + ' m' : 'NaN';

          marker.setLatLng([lat, lon]);
          marker.bindPopup(
            `lat=${lat.toFixed(7)}<br>lon=${lon.toFixed(7)}<br>alt=${Number.isFinite(alt) ? alt.toFixed(2) : 'NaN'} m`
          );

          trail.push([lat, lon]);
          pathLine.setLatLngs(trail);

          if (firstFix) {
            map.setView([lat, lon], 18);
            firstFix = false;
          } else {
            map.panTo([lat, lon], {animate: true, duration: 0.5});
          }
        } catch (err) {
          console.error('Bad WS message:', err);
        }
      };
    }

    connectWS();
  </script>
</body>
</html>
"""


class GpsMapNode(Node):
    def __init__(self):
        super().__init__('gps_map_node')

        self.declare_parameter('topic_name', '/gps/fix')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8000)
        self.declare_parameter('open_browser', True)

        self.topic_name = self.get_parameter('topic_name').value
        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.open_browser = self.get_parameter('open_browser').value

        self.latest_data = {
            'lat': None,
            'lon': None,
            'alt': None
        }
        self.data_lock = threading.Lock()

        self.ws_clients = set()
        self.ws_loop = None
        self.server_thread = None

        self.subscription = self.create_subscription(
            NavSatFix,
            self.topic_name,
            self.gps_callback,
            10
        )

        self.get_logger().info(f'Subscribing to topic: {self.topic_name}')
        self.start_web_server()

    def gps_callback(self, msg: NavSatFix):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            self.get_logger().warn(f'Invalid GPS coordinate: lat={lat}, lon={lon}')
            return

        with self.data_lock:
            self.latest_data = {
                'lat': float(lat),
                'lon': float(lon),
                'alt': float(alt)
            }

        self.get_logger().info(
            f'GPS update -> lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}'
        )

        if self.ws_loop is not None:
            asyncio.run_coroutine_threadsafe(
                self.broadcast_position(),
                self.ws_loop
            )

    async def handle_index(self, request):
        return web.Response(text=HTML_PAGE, content_type='text/html')

    async def websocket_handler(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        self.ws_clients.add(ws)

        with self.data_lock:
            payload = dict(self.latest_data)
        await ws.send_str(json.dumps(payload))

        try:
            async for _ in ws:
                pass
        finally:
            self.ws_clients.discard(ws)

        return ws

    async def broadcast_position(self):
        if not self.ws_clients:
            return

        with self.data_lock:
            payload = json.dumps(self.latest_data)

        dead_clients = []
        for ws in list(self.ws_clients):
            try:
                await ws.send_str(payload)
            except Exception:
                dead_clients.append(ws)

        for ws in dead_clients:
            self.ws_clients.discard(ws)

    async def _start_aiohttp_app(self):
        app = web.Application()
        app.add_routes([
            web.get('/', self.handle_index),
            web.get('/ws', self.websocket_handler),
        ])

        runner = web.AppRunner(app)
        await runner.setup()

        site = web.TCPSite(runner, self.host, self.port)
        await site.start()

        self.get_logger().info(f'Web server running at http://{self.host}:{self.port}')

        if self.open_browser:
            webbrowser.open(f'http://{self.host}:{self.port}')

        while True:
            await asyncio.sleep(3600)

    def _run_server_thread(self):
        self.ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.ws_loop)
        self.ws_loop.run_until_complete(self._start_aiohttp_app())

    def start_web_server(self):
        self.server_thread = threading.Thread(target=self._run_server_thread, daemon=True)
        self.server_thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = GpsMapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
