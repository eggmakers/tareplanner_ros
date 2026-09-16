#!/usr/bin/env python3
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import airsim
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


HTML_PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>TARE UAV Live</title>
<style>
:root { color-scheme: dark; font-family: Inter, "Segoe UI", sans-serif; }
* { box-sizing: border-box; }
html, body { margin: 0; width: 100%; height: 100%; overflow: hidden; background: #101316; color: #eef2f4; }
body { display: grid; grid-template-rows: auto minmax(0, 1fr) 32px; }
header { display: flex; align-items: center; gap: 18px; padding: 0 18px; border-bottom: 1px solid #30363b; background: #171b1f; }
.brand { min-width: 150px; font-size: 15px; font-weight: 700; letter-spacing: 0; }
.stats { display: flex; align-items: center; gap: 18px; flex: 1; min-width: 0; }
.metric { min-width: 72px; }
.label { color: #8d989f; font-size: 10px; text-transform: uppercase; }
.value { margin-top: 2px; color: #f3f5f6; font-size: 13px; font-variant-numeric: tabular-nums; white-space: nowrap; }
.mode { color: #55d6a8; }
.controls { display: flex; gap: 6px; }
button { height: 32px; border: 1px solid #3a4248; border-radius: 4px; padding: 0 11px; background: #22282d; color: #e5eaed; cursor: pointer; }
button:hover { background: #2a3238; }
button.active { border-color: #39bfa0; color: #65e0bc; }
main { position: relative; min-height: 0; }
canvas { display: block; width: 100%; height: 100%; cursor: grab; }
canvas.dragging { cursor: grabbing; }
.camera-panel { position: absolute; top: 12px; right: 12px; width: min(390px, calc(100% - 24px)); border: 1px solid #3a4248; background: #171b1f; box-shadow: 0 10px 30px rgba(0,0,0,.35); z-index: 4; }
.camera-panel[hidden] { display: none; }
.camera-preview { display: block; width: 100%; aspect-ratio: 16 / 9; object-fit: cover; background: #0b0d0f; border-bottom: 1px solid #30363b; }
.camera-status { position: absolute; top: 8px; left: 8px; padding: 3px 6px; background: rgba(12,15,17,.8); color: #d9dee1; font-size: 10px; }
.camera-controls { display: grid; grid-template-columns: 76px minmax(0,1fr) 54px; gap: 7px 9px; align-items: center; padding: 10px 12px 12px; }
.camera-controls label, .camera-controls output { font-size: 11px; color: #aeb7bc; }
.camera-controls output { text-align: right; font-variant-numeric: tabular-nums; }
.camera-controls input { width: 100%; accent-color: #39bfa0; }
.side-title { position: absolute; right: 22px; bottom: 190px; color: #9ca7ad; font-size: 11px; pointer-events: none; }
footer { display: flex; align-items: center; gap: 18px; padding: 0 18px; border-top: 1px solid #30363b; background: #171b1f; color: #9ca7ad; font-size: 11px; }
.legend { display: inline-flex; align-items: center; gap: 6px; }
.swatch { width: 12px; height: 3px; }
.cloud { background: #57c9e8; }.trail { background: #65e0bc; }.global { background: #f0c04a; }.local { background: #ff875f; }.obstacle { background: #d85d66; }.fence { background: #d9dee1; }
@media (min-width: 701px) { header { height: 58px; } }
@media (max-width: 900px) { .metric:nth-child(n+5) { display: none; } .brand { min-width: auto; } header { gap: 10px; padding: 0 10px; } .stats { gap: 10px; } }
@media (max-width: 700px) {
  header { min-height: 82px; display: grid; grid-template-columns: minmax(100px, 1fr) auto; grid-template-rows: 40px 34px; gap: 0 8px; padding: 4px 10px; }
  .brand { grid-column: 1; grid-row: 1; white-space: nowrap; }
  .controls { grid-column: 2; grid-row: 1; }
  button { height: 28px; padding: 0 7px; font-size: 11px; }
  .stats { grid-column: 1 / -1; grid-row: 2; gap: 20px; overflow: hidden; }
  .metric { min-width: 0; }
  .metric:nth-child(n+3) { display: none; }
  .value { font-size: 12px; }
  footer { gap: 12px; padding: 0 10px; overflow: hidden; white-space: nowrap; }
  footer .legend:nth-child(3), footer .legend:nth-child(4) { display: none; }
}
@media (max-width: 430px) {
  .brand { font-size: 13px; }
  button { padding: 0 5px; }
  footer { gap: 9px; font-size: 10px; }
  #age { display: none; }
  .camera-panel { top: 8px; right: 8px; width: min(340px, calc(100% - 16px)); }
}
</style>
</head>
<body>
<header>
  <div class="brand">TARE UAV LIVE</div>
  <div class="stats">
    <div class="metric"><div class="label">Mode</div><div id="mode" class="value mode">WAIT</div></div>
    <div class="metric"><div class="label">Mission</div><div id="mission" class="value">WAIT</div></div>
    <div class="metric"><div class="label">Position ENU</div><div id="position" class="value">--</div></div>
    <div class="metric"><div class="label">Speed</div><div id="speed" class="value">--</div></div>
    <div class="metric"><div class="label">Tilt</div><div id="tilt" class="value">--</div></div>
    <div class="metric"><div class="label">Cloud</div><div id="cloudCount" class="value">--</div></div>
    <div class="metric"><div class="label">Setpoint</div><div id="setpoint" class="value">--</div></div>
  </div>
  <div class="controls">
    <button id="follow" class="active" title="Keep the vehicle centered">Follow</button>
    <button id="fit" title="Fit all visible data">Fit</button>
    <button id="camera" title="Adjust the AirSim observation camera">Camera</button>
    <button id="pause" title="Pause live updates">Pause</button>
    <button id="clear" title="Clear flown trajectory">Clear</button>
  </div>
</header>
<main>
  <canvas id="map"></canvas><div class="side-title">SIDE PROFILE</div>
  <aside id="cameraPanel" class="camera-panel" hidden>
    <img id="cameraPreview" class="camera-preview" alt="AirSim observation camera">
    <span id="cameraStatus" class="camera-status">connecting</span>
    <div class="camera-controls">
      <label for="camOrbit">Orbit</label><input id="camOrbit" type="range" min="-180" max="180" step="1"><output id="camOrbitOut"></output>
      <label for="camDistance">Distance</label><input id="camDistance" type="range" min="1" max="12" step="0.1"><output id="camDistanceOut"></output>
      <label for="camHeight">Height</label><input id="camHeight" type="range" min="0.5" max="8" step="0.1"><output id="camHeightOut"></output>
      <label for="camPitch">Pitch</label><input id="camPitch" type="range" min="-45" max="10" step="0.5"><output id="camPitchOut"></output>
      <label for="camFov">FOV</label><input id="camFov" type="range" min="60" max="130" step="1"><output id="camFovOut"></output>
    </div>
  </aside>
</main>
<footer>
  <span class="legend"><span class="swatch cloud"></span>LiDAR</span>
  <span class="legend"><span class="swatch trail"></span>Flown</span>
  <span class="legend"><span class="swatch global"></span>Global path</span>
  <span class="legend"><span class="swatch local"></span>Local path</span>
  <span class="legend"><span class="swatch obstacle"></span>Obstacle</span>
  <span class="legend"><span class="swatch fence"></span>Safety fence</span>
  <span id="age" style="margin-left:auto">offline</span>
</footer>
<script>
const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');
let view = {x: 0, y: 10, scale: 24};
let live = null, paused = false, follow = true, dragging = false, lastMouse = null;
let cameraVisible = false, cameraTimer = null, cameraPostTimer = null;
const cameraControls = {
  orbit_degrees: ['camOrbit', 'camOrbitOut', ' deg'],
  distance: ['camDistance', 'camDistanceOut', ' m'],
  height: ['camHeight', 'camHeightOut', ' m'],
  pitch_degrees: ['camPitch', 'camPitchOut', ' deg'],
  fov_degrees: ['camFov', 'camFovOut', ' deg']
};

function resize() {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  draw();
}
function sx(x) { return canvas.clientWidth / 2 + (x - view.x) * view.scale; }
function sy(y) { return canvas.clientHeight / 2 - (y - view.y) * view.scale; }
function grid() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  ctx.fillStyle = '#101316'; ctx.fillRect(0, 0, w, h);
  const step = view.scale < 12 ? 5 : 1;
  ctx.lineWidth = 1; ctx.strokeStyle = '#242a2f'; ctx.beginPath();
  const minX = view.x - w / (2 * view.scale), maxX = view.x + w / (2 * view.scale);
  const minY = view.y - h / (2 * view.scale), maxY = view.y + h / (2 * view.scale);
  for (let x = Math.floor(minX / step) * step; x <= maxX; x += step) { ctx.moveTo(sx(x), 0); ctx.lineTo(sx(x), h); }
  for (let y = Math.floor(minY / step) * step; y <= maxY; y += step) { ctx.moveTo(0, sy(y)); ctx.lineTo(w, sy(y)); }
  ctx.stroke();
  ctx.strokeStyle = '#465058'; ctx.beginPath(); ctx.moveTo(sx(0), 0); ctx.lineTo(sx(0), h); ctx.moveTo(0, sy(0)); ctx.lineTo(w, sy(0)); ctx.stroke();
}
function drawPolyline(points, color, width) {
  if (!points || points.length < 2) return;
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath();
  ctx.moveTo(sx(points[0][0]), sy(points[0][1]));
  for (let i = 1; i < points.length; i++) ctx.lineTo(sx(points[i][0]), sy(points[i][1]));
  ctx.stroke();
}
function drawObstacles(items) {
  if (!items) return;
  for (const item of items) {
    if (item.role === 'floor' || item.role === 'ceiling') continue;
    const p = item.center_enu, size = item.size_enu;
    ctx.fillStyle = item.role === 'obstacle' ? 'rgba(216,93,102,.48)' : 'rgba(112,122,129,.42)';
    ctx.strokeStyle = item.role === 'obstacle' ? '#d85d66' : '#7b858b'; ctx.lineWidth = 1;
    const x = sx(p[0] - size[0] / 2), y = sy(p[1] + size[1] / 2);
    const width = size[0] * view.scale, height = size[1] * view.scale;
    ctx.fillRect(x, y, width, height); ctx.strokeRect(x, y, width, height);
  }
}
function drawGeofence(bounds) {
  if (!bounds) return;
  const x = sx(bounds.min_x), y = sy(bounds.max_y);
  const width = (bounds.max_x - bounds.min_x) * view.scale;
  const height = (bounds.max_y - bounds.min_y) * view.scale;
  ctx.save(); ctx.setLineDash([7,5]); ctx.strokeStyle = '#d9dee1'; ctx.lineWidth = 1.5;
  ctx.strokeRect(x, y, width, height); ctx.restore();
}
function drawCloud(points) {
  if (!points) return;
  for (const p of points) {
    const hue = 190 - Math.max(0, Math.min(1, p[2] / 4)) * 125;
    ctx.fillStyle = `hsla(${hue},78%,64%,.72)`;
    ctx.fillRect(sx(p[0]), sy(p[1]), 2, 2);
  }
}
function cross(point, color, radius) {
  if (!point) return;
  const x = sx(point[0]), y = sy(point[1]); ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
  ctx.moveTo(x-radius,y); ctx.lineTo(x+radius,y); ctx.moveTo(x,y-radius); ctx.lineTo(x,y+radius); ctx.stroke();
}
function vehicle(odom) {
  if (!odom) return;
  const x = sx(odom.position[0]), y = sy(odom.position[1]), yaw = odom.yaw;
  ctx.save(); ctx.translate(x,y); ctx.rotate(-yaw); ctx.fillStyle = '#f3f5f6'; ctx.strokeStyle = '#101316'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(12,0); ctx.lineTo(-8,-7); ctx.lineTo(-5,0); ctx.lineTo(-8,7); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
}
function sideProfile(data) {
  const width = Math.min(390, canvas.clientWidth * .42), height = 160;
  const left = canvas.clientWidth - width - 18, top = canvas.clientHeight - height - 18;
  ctx.fillStyle = 'rgba(19,23,26,.94)'; ctx.fillRect(left, top, width, height);
  ctx.strokeStyle = '#3b444a'; ctx.strokeRect(left, top, width, height);
  const all = (data.cloud || []).concat(data.trail || []);
  let minY = 0, maxY = 32; if (all.length) { minY = Math.min(...all.map(p=>p[1]), 0); maxY = Math.max(...all.map(p=>p[1]), 10); }
  const px = y => left + 12 + (y-minY) / Math.max(1,maxY-minY) * (width-24);
  const py = z => top + height - 15 - z / 5 * (height-30);
  ctx.strokeStyle = '#4a545b'; ctx.beginPath(); ctx.moveTo(left+8,py(0)); ctx.lineTo(left+width-8,py(0)); ctx.stroke();
  ctx.fillStyle = 'rgba(87,201,232,.55)'; for (const p of data.cloud || []) ctx.fillRect(px(p[1]),py(p[2]),1.5,1.5);
  drawSideLine(data.trail, '#65e0bc', px, py);
  if (data.odom) { ctx.fillStyle = '#fff'; ctx.beginPath(); ctx.arc(px(data.odom.position[1]),py(data.odom.position[2]),4,0,Math.PI*2); ctx.fill(); }
}
function drawSideLine(points, color, px, py) { if (!points || points.length < 2) return; ctx.strokeStyle=color; ctx.lineWidth=1.5; ctx.beginPath(); ctx.moveTo(px(points[0][1]),py(points[0][2])); for(let i=1;i<points.length;i++)ctx.lineTo(px(points[i][1]),py(points[i][2])); ctx.stroke(); }
function draw() {
  grid(); if (!live) return;
  if (follow && live.odom) { view.x = live.odom.position[0]; view.y = live.odom.position[1]; }
  drawObstacles(live.scene); drawGeofence(live.geofence); drawCloud(live.cloud);
  drawPolyline(live.global_path, '#f0c04a', 2); drawPolyline(live.local_path, '#ff875f', 2.5); drawPolyline(live.trail, '#65e0bc', 2);
  cross(live.waypoint, '#ff875f', 7); cross(live.setpoint, '#f0c04a', 5); vehicle(live.odom); sideProfile(live);
}
function fmt(value, digits=2) { return Number.isFinite(value) ? value.toFixed(digits) : '--'; }
function metrics(data) {
  document.getElementById('mode').textContent = `${data.state.mode || 'WAIT'} ${data.state.armed ? 'ARMED' : 'SAFE'}`;
  document.getElementById('mission').textContent = data.exploration_finished ? 'DONE / HOME' : 'EXPLORING';
  document.getElementById('position').textContent = data.odom ? data.odom.position.map(v=>fmt(v)).join('  ') : '--';
  document.getElementById('speed').textContent = `${fmt(data.command_speed)} m/s`;
  document.getElementById('tilt').textContent = data.odom ? `${fmt(data.odom.tilt_deg,1)} deg` : '--';
  document.getElementById('cloudCount').textContent = `${data.cloud_source_points || 0} @ ${fmt(data.cloud_rate_hz,1)} Hz`;
  document.getElementById('setpoint').textContent = data.setpoint ? data.setpoint.map(v=>fmt(v)).join('  ') : '--';
  document.getElementById('age').textContent = `${fmt(data.age_seconds,1)} s`;
}
async function poll() {
  if (paused) return;
  try { const response = await fetch('/state', {cache:'no-store'}); if (!response.ok) throw new Error('offline'); live = await response.json(); metrics(live); draw(); }
  catch (_) { document.getElementById('age').textContent = 'offline'; }
}
function cameraValues() {
  const result = {};
  for (const [key, ids] of Object.entries(cameraControls)) result[key] = Number(document.getElementById(ids[0]).value);
  return result;
}
function renderCameraValues(values) {
  for (const [key, ids] of Object.entries(cameraControls)) {
    const input = document.getElementById(ids[0]), output = document.getElementById(ids[1]);
    if (values && Number.isFinite(values[key])) input.value = values[key];
    output.textContent = `${input.value}${ids[2]}`;
  }
}
async function loadCameraSettings() {
  try {
    const response = await fetch('/camera', {cache:'no-store'});
    if (!response.ok) throw new Error('offline');
    renderCameraValues(await response.json());
  } catch (_) { document.getElementById('cameraStatus').textContent = 'camera offline'; }
}
async function postCamera() {
  try {
    const response = await fetch('/camera', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(cameraValues())});
    if (!response.ok) throw new Error('offline');
    renderCameraValues(await response.json());
    document.getElementById('cameraStatus').textContent = 'live';
    refreshCameraImage();
  } catch (_) { document.getElementById('cameraStatus').textContent = 'camera offline'; }
}
function refreshCameraImage() {
  if (!cameraVisible) return;
  const preview = document.getElementById('cameraPreview');
  preview.onload = () => { document.getElementById('cameraStatus').textContent = 'live'; };
  preview.onerror = () => { document.getElementById('cameraStatus').textContent = 'camera offline'; };
  preview.src = `/camera.png?t=${Date.now()}`;
}
document.getElementById('follow').onclick = e => { follow=!follow; e.target.classList.toggle('active',follow); };
document.getElementById('camera').onclick = async e => {
  cameraVisible = !cameraVisible; e.target.classList.toggle('active', cameraVisible);
  document.getElementById('cameraPanel').hidden = !cameraVisible;
  if (cameraVisible) { await loadCameraSettings(); refreshCameraImage(); cameraTimer = setInterval(refreshCameraImage, 1000); }
  else { clearInterval(cameraTimer); cameraTimer = null; }
};
for (const ids of Object.values(cameraControls)) document.getElementById(ids[0]).oninput = () => {
  renderCameraValues(); clearTimeout(cameraPostTimer); cameraPostTimer = setTimeout(postCamera, 120);
};
document.getElementById('pause').onclick = e => { paused=!paused; e.target.textContent=paused?'Resume':'Pause'; e.target.classList.toggle('active',paused); };
document.getElementById('clear').onclick = async () => { await fetch('/trail/clear',{method:'POST'}); };
document.getElementById('fit').onclick = () => { if (!live) return; const pts=(live.cloud||[]).concat(live.trail||[]); for(const o of live.scene||[])pts.push(o.center_enu); if(live.geofence){pts.push([live.geofence.min_x,live.geofence.min_y,0],[live.geofence.max_x,live.geofence.max_y,0]);} if(!pts.length)return; const xs=pts.map(p=>p[0]),ys=pts.map(p=>p[1]); const minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys); view.x=(minX+maxX)/2;view.y=(minY+maxY)/2;view.scale=Math.max(4,Math.min(canvas.clientWidth/Math.max(10,maxX-minX+4),canvas.clientHeight/Math.max(10,maxY-minY+4)));follow=false;document.getElementById('follow').classList.remove('active');draw(); };
canvas.onmousedown = e => { dragging=true;lastMouse=[e.clientX,e.clientY];canvas.classList.add('dragging');follow=false;document.getElementById('follow').classList.remove('active'); };
window.onmouseup = () => { dragging=false;canvas.classList.remove('dragging'); };
window.onmousemove = e => { if(!dragging)return;view.x-=(e.clientX-lastMouse[0])/view.scale;view.y+=(e.clientY-lastMouse[1])/view.scale;lastMouse=[e.clientX,e.clientY];draw(); };
canvas.onwheel = e => { e.preventDefault(); view.scale=Math.max(3,Math.min(120,view.scale*Math.exp(-e.deltaY*.001)));draw(); };
window.onresize=resize; resize(); setInterval(poll,200); poll();
</script>
</body>
</html>"""


class AirSimViewCamera:
    LIMITS = {
        "orbit_degrees": (-180.0, 180.0),
        "distance": (1.0, 12.0),
        "height": (0.5, 8.0),
        "pitch_degrees": (-45.0, 10.0),
        "fov_degrees": (60.0, 130.0),
    }

    def __init__(self, host, port, vehicle_name, camera_name):
        self.host = host
        self.port = port
        self.vehicle_name = vehicle_name
        self.camera_name = camera_name
        self.lock = threading.Lock()
        self.client = None
        self.values = {
            "orbit_degrees": 0.0,
            "distance": 3.0,
            "height": 2.0,
            "pitch_degrees": -11.5,
            "fov_degrees": 110.0,
        }

    def snapshot(self):
        with self.lock:
            return dict(self.values)

    def _client(self):
        if self.client is None:
            self.client = airsim.MultirotorClient(ip=self.host, port=self.port)
        return self.client

    @classmethod
    def clamp_values(cls, values, current):
        result = dict(current)
        for key, (minimum, maximum) in cls.LIMITS.items():
            if key not in values:
                continue
            value = float(values[key])
            if not math.isfinite(value):
                raise ValueError("camera setting must be finite: " + key)
            result[key] = max(minimum, min(maximum, value))
        return result

    def update(self, values):
        with self.lock:
            updated = self.clamp_values(values, self.values)
            orbit = math.radians(updated["orbit_degrees"])
            pose = airsim.Pose(
                airsim.Vector3r(
                    -updated["distance"] * math.cos(orbit),
                    -updated["distance"] * math.sin(orbit),
                    -updated["height"],
                ),
                airsim.to_quaternion(
                    math.radians(updated["pitch_degrees"]), 0.0, orbit
                ),
            )
            try:
                client = self._client()
                client.simSetCameraFov(
                    self.camera_name,
                    updated["fov_degrees"],
                    vehicle_name=self.vehicle_name,
                )
                client.simSetCameraPose(
                    self.camera_name, pose, vehicle_name=self.vehicle_name
                )
            except Exception:
                self.client = None
                raise
            self.values = updated
            return dict(self.values)

    def image_png(self):
        with self.lock:
            try:
                response = self._client().simGetImages(
                    [
                        airsim.ImageRequest(
                            self.camera_name,
                            airsim.ImageType.Scene,
                            pixels_as_float=False,
                            compress=True,
                        )
                    ],
                    vehicle_name=self.vehicle_name,
                )[0]
                payload = bytes(response.image_data_uint8)
                if not payload:
                    raise RuntimeError("AirSim returned an empty camera image")
                return payload
            except Exception:
                self.client = None
                raise


def quaternion_to_euler(quaternion):
    sin_roll = 2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
    cos_roll = 1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return roll, pitch, yaw


def rounded_point(point):
    return [round(point.x, 3), round(point.y, 3), round(point.z, 3)]


class ViewerState:
    def __init__(
        self,
        max_cloud_points,
        max_trail_points,
        scene_file,
        cloud_display_rate,
        view_camera,
    ):
        self.lock = threading.Lock()
        self.max_cloud_points = max_cloud_points
        self.max_trail_points = max_trail_points
        self.min_cloud_interval = 1.0 / max(0.5, cloud_display_rate)
        self.scene_file = scene_file
        self.scene_mtime = None
        self.scene = []
        self.geofence = None
        self.cloud = []
        self.cloud_source_points = 0
        self.cloud_rate_hz = 0.0
        self.last_cloud_time = None
        self.last_cloud_attempt = 0.0
        self.last_update_time = None
        self.odom = None
        self.trail = []
        self.waypoint = None
        self.setpoint = None
        self.command_speed = 0.0
        self.global_path = []
        self.local_path = []
        self.vehicle_state = {"connected": False, "armed": False, "mode": ""}
        self.exploration_finished = False
        self.view_camera = view_camera

    def cloud_callback(self, msg):
        attempt_time = time.monotonic()
        with self.lock:
            if attempt_time - self.last_cloud_attempt < self.min_cloud_interval:
                return
            self.last_cloud_attempt = attempt_time
        source_count = max(0, msg.width * msg.height)
        stride = max(1, int(math.ceil(source_count / float(self.max_cloud_points))))
        points = []
        for index, values in enumerate(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if index % stride == 0:
                points.append([round(values[0], 3), round(values[1], 3), round(values[2], 3)])
                if len(points) >= self.max_cloud_points:
                    break
        now = time.time()
        with self.lock:
            if self.last_cloud_time is not None:
                instant_rate = 1.0 / max(1e-3, now - self.last_cloud_time)
                self.cloud_rate_hz = 0.8 * self.cloud_rate_hz + 0.2 * instant_rate
            self.last_cloud_time = now
            self.last_update_time = now
            self.cloud_source_points = source_count
            self.cloud = points

    def odom_callback(self, msg):
        roll, pitch, yaw = quaternion_to_euler(msg.pose.pose.orientation)
        position = rounded_point(msg.pose.pose.position)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch)))))
        odom = {
            "position": position,
            "roll_deg": round(math.degrees(roll), 2),
            "pitch_deg": round(math.degrees(pitch), 2),
            "yaw": round(yaw, 4),
            "tilt_deg": round(tilt, 2),
        }
        with self.lock:
            self.odom = odom
            self.last_update_time = time.time()
            if not self.trail or math.dist(position, self.trail[-1]) >= 0.08:
                self.trail.append(position)
                if len(self.trail) > self.max_trail_points:
                    self.trail = self.trail[-self.max_trail_points :]

    def waypoint_callback(self, msg):
        with self.lock:
            self.waypoint = rounded_point(msg.point)

    def setpoint_callback(self, msg):
        with self.lock:
            self.setpoint = rounded_point(msg.pose.position)

    def velocity_callback(self, msg):
        velocity = msg.twist.linear
        with self.lock:
            self.command_speed = round(
                math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z), 3
            )

    def global_path_callback(self, msg):
        with self.lock:
            self.global_path = [rounded_point(pose.pose.position) for pose in msg.poses[::2]][:800]

    def local_path_callback(self, msg):
        with self.lock:
            self.local_path = [rounded_point(pose.pose.position) for pose in msg.poses][:500]

    def state_callback(self, msg):
        with self.lock:
            self.vehicle_state = {
                "connected": bool(msg.connected),
                "armed": bool(msg.armed),
                "mode": msg.mode,
            }

    def exploration_finish_callback(self, msg):
        with self.lock:
            self.exploration_finished = bool(msg.data)

    def clear_trail(self):
        with self.lock:
            self.trail = []

    def load_scene(self):
        try:
            mtime = os.path.getmtime(self.scene_file)
            if mtime == self.scene_mtime:
                return
            with open(self.scene_file, "r", encoding="utf-8") as stream:
                report = json.load(stream)
            self.scene = report.get("objects", [])
            self.geofence = report.get("safety_geofence_enu")
            self.scene_mtime = mtime
        except (OSError, ValueError):
            self.scene = []
            self.geofence = None

    def snapshot(self):
        self.load_scene()
        with self.lock:
            age = None if self.last_update_time is None else time.time() - self.last_update_time
            return {
                "cloud": self.cloud,
                "cloud_source_points": self.cloud_source_points,
                "cloud_rate_hz": round(self.cloud_rate_hz, 2),
                "odom": self.odom,
                "trail": self.trail,
                "waypoint": self.waypoint,
                "setpoint": self.setpoint,
                "command_speed": self.command_speed,
                "global_path": self.global_path,
                "local_path": self.local_path,
                "state": self.vehicle_state,
                "exploration_finished": self.exploration_finished,
                "scene": self.scene,
                "geofence": self.geofence,
                "camera": self.view_camera.snapshot(),
                "age_seconds": None if age is None else round(age, 2),
            }


class ViewerHandler(BaseHTTPRequestHandler):
    state = None

    def send_bytes(self, status, content_type, payload):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        route = urlparse(self.path).path
        if route == "/state":
            payload = json.dumps(self.state.snapshot(), separators=(",", ":")).encode("utf-8")
            self.send_bytes(200, "application/json", payload)
        elif route == "/health":
            self.send_bytes(200, "application/json", b'{"status":"ok"}')
        elif route == "/camera":
            payload = json.dumps(
                self.state.view_camera.snapshot(), separators=(",", ":")
            ).encode("utf-8")
            self.send_bytes(200, "application/json", payload)
        elif route == "/camera.png":
            try:
                self.send_bytes(200, "image/png", self.state.view_camera.image_png())
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "viewer camera image failed: %s", exc)
                self.send_bytes(503, "text/plain; charset=utf-8", b"camera offline")
        elif route == "/" or route == "/index.html":
            self.send_bytes(200, "text/html; charset=utf-8", HTML_PAGE.encode("utf-8"))
        else:
            self.send_bytes(404, "text/plain; charset=utf-8", b"not found")

    def do_POST(self):
        route = urlparse(self.path).path
        if route == "/trail/clear":
            self.state.clear_trail()
            self.send_bytes(200, "application/json", b'{"status":"ok"}')
        elif route == "/camera":
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 4096:
                    raise ValueError("invalid request length")
                values = json.loads(self.rfile.read(length).decode("utf-8"))
                if not isinstance(values, dict):
                    raise ValueError("camera request must be an object")
                payload = json.dumps(
                    self.state.view_camera.update(values), separators=(",", ":")
                ).encode("utf-8")
                self.send_bytes(200, "application/json", payload)
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                self.send_bytes(400, "text/plain; charset=utf-8", str(exc).encode("utf-8"))
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "viewer camera update failed: %s", exc)
                self.send_bytes(503, "text/plain; charset=utf-8", b"camera offline")
        else:
            self.send_bytes(404, "text/plain; charset=utf-8", b"not found")

    def log_message(self, _format, *_args):
        return


def main():
    rospy.init_node("uav_exploration_viewer")
    port = int(rospy.get_param("~port", 8765))
    max_cloud_points = int(rospy.get_param("~max_cloud_points", 2500))
    max_trail_points = int(rospy.get_param("~max_trail_points", 3000))
    cloud_display_rate = float(rospy.get_param("~cloud_display_rate", 5.0))
    scene_file = rospy.get_param("~scene_file", "/data/tunnel_scene.json")
    view_camera = AirSimViewCamera(
        rospy.get_param("~airsim_host", "host.docker.internal"),
        int(rospy.get_param("~airsim_port", 41452)),
        rospy.get_param("~vehicle_name", "PX4"),
        rospy.get_param("~camera_name", "0"),
    )
    state = ViewerState(
        max_cloud_points,
        max_trail_points,
        scene_file,
        cloud_display_rate,
        view_camera,
    )

    rospy.Subscriber("/airsim/registered_scan", PointCloud2, state.cloud_callback, queue_size=1)
    rospy.Subscriber("/mavros/local_position/odom", Odometry, state.odom_callback, queue_size=5)
    rospy.Subscriber("/mavros/state", State, state.state_callback, queue_size=5)
    rospy.Subscriber("/way_point", PointStamped, state.waypoint_callback, queue_size=2)
    rospy.Subscriber(
        "/sensor_coverage_planner/exploration_finish",
        Bool,
        state.exploration_finish_callback,
        queue_size=2,
    )
    rospy.Subscriber(
        "/mavros/setpoint_position/local", PoseStamped, state.setpoint_callback, queue_size=2
    )
    rospy.Subscriber(
        "/tare_uav/command_velocity", TwistStamped, state.velocity_callback, queue_size=2
    )
    rospy.Subscriber(
        "/sensor_coverage_planner/global_path", Path, state.global_path_callback, queue_size=1
    )
    rospy.Subscriber(
        "/sensor_coverage_planner/local_path", Path, state.local_path_callback, queue_size=1
    )

    ViewerHandler.state = state
    server = ThreadingHTTPServer(("0.0.0.0", port), ViewerHandler)
    server.daemon_threads = True
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    rospy.on_shutdown(server.shutdown)
    rospy.loginfo("uav_exploration_viewer: http://0.0.0.0:%d", port)
    rospy.spin()


if __name__ == "__main__":
    main()
