#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
import subprocess
import signal
import os

from flask import Flask, request, jsonify, render_template_string
from flask_cors import CORS

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid

HTML = """
<!DOCTYPE html><html lang="ru"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>Robot Mapper</title><style>
:root{--bg:#0b0d12;--card:#15181e;--acc:#3b82f6;--txt:#e2e8f0;--mut:#94a3b8;--rad:16px;--green:#10b981;--red:#ef4444}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{font-family:system-ui,sans-serif;background:var(--bg);color:var(--txt);margin:0;padding:16px;display:flex;flex-direction:column;align-items:center;min-height:100vh}
h1{font-size:1.6rem;margin:0 0 4px;background:linear-gradient(135deg,#60a5fa,#a78bfa);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.sub{font-size:.85rem;color:var(--mut);margin-bottom:16px;text-align:center}
.panel{background:var(--card);border-radius:var(--rad);padding:16px;width:100%;max-width:480px;box-shadow:0 4px 16px #0008;margin-bottom:14px}
.slider{display:flex;align-items:center;justify-content:space-between;margin:8px 0}
.slider label{min-width:100px;font-size:.9rem}.slider span{font-weight:600;color:var(--acc);min-width:40px;text-align:right}
input[type=range]{flex:1;margin:0 10px;accent-color:var(--acc)}
.dpad{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin:16px 0}
.btn{background:#252a33;color:#fff;border:none;border-radius:14px;padding:18px 0;font-size:1.4rem;cursor:pointer;transition:.12s;user-select:none;touch-action:manipulation}
.btn:active,.btn.on{background:var(--acc);transform:scale(.96)}
.btn.stop{background:var(--red)}.btn.stop:active{background:#b91c1c}
.ctrl-row{display:flex;gap:10px;margin-top:8px}
.ctrl-btn{flex:1;padding:14px;font-size:1rem;border-radius:12px;border:none;cursor:pointer;font-weight:600;transition:.15s}
.ctrl-btn.auto{background:#1e293b;color:#94a3b8;border:1px solid #334155}
.ctrl-btn.auto.on{background:var(--green);color:#fff;border-color:var(--green);box-shadow:0 0 12px #10b98140}
.ctrl-btn.kill{background:#3b0000;color:#ff8888;border:1px solid #600}
.ctrl-btn.kill:active{background:#500}
.status{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;text-align:center}
.s-item{background:#0f1115;padding:10px;border-radius:10px;font-size:.85rem}.s-val{font-weight:600;margin-top:4px}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px}
.g{background:var(--green);box-shadow:0 0 6px var(--green)}.y{background:#f59e0b}.r{background:var(--red)}
kbd{background:#1f2329;padding:3px 7px;border-radius:6px;font-size:.7rem;font-family:monospace}
</style></head><body>
<h1>🤖 Robot Mapper</h1><div class="sub">ROS 2 Web Control • Toggle Auto/Manual</div>
<div class="panel">
  <div class="slider"><label>Линейная</label><input id="lin" type="range" min="0.02" max="0.25" step="0.01" value="0.10"><span id="lv">0.10</span></div>
  <div class="slider"><label>Угловая</label><input id="ang" type="range" min="0.20" max="1.50" step="0.05" value="0.60"><span id="av">0.60</span></div>
</div>
<div class="panel">
  <div class="dpad">
    <div></div><button class="btn" data-d="f">↑</button><div></div>
    <button class="btn" data-d="l">←</button><button class="btn stop" data-d="s">⏹</button><button class="btn" data-d="r">→</button>
    <div></div><button class="btn" data-d="b">↓</button><div></div>
  </div>
  <div class="ctrl-row">
    <button class="ctrl-btn auto" id="btnAuto">▶ Авто</button>
    <button class="ctrl-btn kill" id="btnStopAuto">⏹ Стоп Авто</button>
  </div>
</div>
<div class="panel">
  <div class="status">
    <div class="s-item"><span class="dot g" id="d_s"></span>Scan<span class="s-val" id="st_s">—</span></div>
    <div class="s-item"><span class="dot y" id="d_m"></span>Map<span class="s-val" id="st_m">—</span></div>
    <div class="s-item"><span class="dot y" id="d_o"></span>Odom<span class="s-val" id="st_o">—</span></div>
  </div>
</div>
<div style="font-size:.75rem;color:var(--mut);text-align:center;margin-top:8px">
  <kbd>↑↓←→</kbd> движение • <kbd>Space</kbd> стоп
</div>
<script>
const $=id=>document.getElementById(id);let ht=null;
$('lin').oninput=e=>$('lv').textContent=parseFloat(e.target.value).toFixed(2);
$('ang').oninput=e=>$('av').textContent=parseFloat(e.target.value).toFixed(2);
function api(u,d={}){return fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)}).then(r=>r.json())}
function move(d){const l=parseFloat($('lin').value),a=parseFloat($('ang').value),p={linear_x:0,angular_z:0};
if(d==='f')p.linear_x=l;if(d==='b')p.linear_x=-l;if(d==='l')p.angular_z=a;if(d==='r')p.angular_z=-a;api('/api/move',p)}
function hold(d){if(ht)clearInterval(ht);ht=setInterval(()=>move(d),100);document.querySelectorAll('.btn').forEach(b=>b.classList.remove('on'));if(d!=='s')document.querySelector(`[data-d="${d}"]`)?.classList.add('on')}
function stop(){clearInterval(ht);ht=null;document.querySelectorAll('.btn').forEach(b=>b.classList.remove('on'));api('/api/stop')}
document.querySelectorAll('.btn').forEach(b=>{const d=b.dataset.d;b.addEventListener('mousedown',()=>d==='s'?stop():hold(d));b.addEventListener('mouseup',stop);b.addEventListener('mouseleave',stop);b.addEventListener('touchstart',e=>{e.preventDefault();d==='s'?stop():hold(d)});b.addEventListener('touchend',stop)});
document.addEventListener('keydown',e=>{if(e.repeat)return;const m={'ArrowUp':'f','ArrowDown':'b','ArrowLeft':'l','ArrowRight':'r',' ':'s'};if(m[e.key]){e.preventDefault();m[e.key]==='s'?stop():hold(m[e.key])}});
document.addEventListener('keyup',e=>{if(['ArrowUp','ArrowDown','ArrowLeft','ArrowRight',' '].includes(e.key))stop()});

$('btnAuto').onclick=()=>{api('/api/auto/start');$('btnAuto').classList.add('on');$('btnAuto').textContent='⏳ Запуск...'}
$('btnStopAuto').onclick=()=>{api('/api/auto/stop');$('btnAuto').classList.remove('on');$('btnAuto').textContent='▶ Авто'}

setInterval(async()=>{try{const d=await fetch('/api/status').then(r=>r.json());const now=Date.now()/1000;
const age=(t)=>t?((now-t).toFixed(1)+'s'):'OFF';
$('st_s').textContent=age(d.scan);$('st_m').textContent=age(d.map);$('st_o').textContent=age(d.odom);
$('d_s').className='dot '+(d.scan?'g':'r');$('d_m').className='dot '+(d.map?'g':'y');$('d_o').className='dot '+(d.odom?'g':'y');
if(!d.auto_run){$('btnAuto').classList.remove('on');$('btnAuto').textContent='▶ Авто'}
else{$('btnAuto').textContent='● Работает'}}catch{}},600)
</script></body></html>
"""

class WebControlNode(Node):
    def __init__(self):
        super().__init__('web_control')
        self.pub = self.create_publisher(Twist, '/cmd_vel/manual', 10)
        self.t_scan = self.t_map = self.t_odom = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self,'t_scan',time.time()), 10)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self,'t_map',time.time()), 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self,'t_odom',time.time()), 10)

    def move(self, lx=0.0, az=0.0):
        m = Twist(); m.linear.x = float(lx); m.angular.z = float(az); self.pub.publish(m)
    def stop(self): self.move()

app = Flask(__name__); CORS(app)
node = None
auto_proc = None
_lock = threading.Lock()

def ros_spin_thread():
    global node
    try:
        rclpy.init()
        node = WebControlNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"[web] ROS error: {e}", flush=True)
    finally:
        try: node.destroy_node()
        except: pass
        if rclpy.ok(): rclpy.shutdown()

def shutdown_cleanup():
    global auto_proc
    with _lock:
        if auto_proc and auto_proc.poll() is None:
            auto_proc.send_signal(signal.SIGINT)
            try: auto_proc.wait(timeout=3)
            except: auto_proc.kill(); auto_proc.wait(timeout=2)
        auto_proc = None
    if node: node.stop()

@app.route('/')
def idx():
    response = app.make_response(render_template_string(HTML))
    response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    return response
@app.route('/api/move', methods=['POST'])
def mv():
    d = request.get_json(silent=True) or {}
    if node: node.move(d.get('linear_x',0), d.get('angular_z',0))
    return jsonify({'ok':True})

@app.route('/api/stop', methods=['POST'])
def st():
    if node: node.stop()
    return jsonify({'ok':True})

@app.route('/api/auto/start', methods=['POST'])
def api_start():
    global auto_proc
    with _lock:
        if auto_proc and auto_proc.poll() is None:
            return jsonify({'status': 'running'})
        auto_proc = subprocess.Popen(['ros2', 'run', 'robot_mapper', 'auto_explorer'], start_new_session=True)
    return jsonify({'status': 'started'})

@app.route('/api/auto/stop', methods=['POST'])
def api_stop():
    global auto_proc
    with _lock:
        if auto_proc and auto_proc.poll() is None:
            auto_proc.send_signal(signal.SIGINT)
            try: auto_proc.wait(timeout=3)
            except: auto_proc.kill(); auto_proc.wait(timeout=2)
        auto_proc = None
    if node: node.stop()  # Мгновенно гасит /cmd_vel/manual, перебивая mux
    return jsonify({'status': 'stopped'})

@app.route('/api/status')
def status():
    global auto_proc
    is_auto = auto_proc is not None and auto_proc.poll() is None
    if not node: return jsonify({'scan':None,'map':None,'odom':None,'auto_run':False})
    return jsonify({'scan':node.t_scan,'map':node.t_map,'odom':node.t_odom,'auto_run':is_auto})

def main():
    global node
    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()
    while node is None: time.sleep(0.1)
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)
    except KeyboardInterrupt: pass
    except Exception as e: print(f"[web] Flask error: {e}", flush=True)
    finally: shutdown_cleanup()

if __name__ == '__main__':
    main()
