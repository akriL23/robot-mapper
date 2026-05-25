#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Web Control Node v3
Architecture:
  - Joystick is connected to RPi directly → joy_node → /joy → joystick_control.py → /cmd_vel/manual
  - This node handles: web D-pad, auto explorer process, status page
  - Subscribes /joy for live joystick display (axes, buttons)
  - Subscribes /ultrasonic/{front_left,front_right,back_left,back_right} for sonar radar
  - Publishes /cmd_vel/manual at 20 Hz keep-alive when web D-pad active
  - /api/joy/params  — GET/POST linear_scale & angular_scale for joystick_control node
  - Modes: WEB | JOY (display only, joy_node drives) | AUTO
"""

import threading
import time
import subprocess
import signal

from flask import Flask, request, jsonify, render_template_string
from flask_cors import CORS

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Joy, Range
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String, Bool

# ─────────────────────────────────────────────────────────────────────────────
HTML = r"""<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Robot Control</title>
<style>
*,*::before,*::after{box-sizing:border-box;-webkit-tap-highlight-color:transparent;margin:0;padding:0}
:root{
  --bg:#080a0e;--sf:#0f1217;--sf2:#161a22;--br:#1e2430;
  --acc:#3b82f6;--pur:#818cf8;--grn:#22c55e;--red:#ef4444;--amb:#f59e0b;
  --txt:#dde3ef;--mut:#4b5568;--r:10px;
  --font:'DM Mono',ui-monospace,monospace;
}
@import url('https://fonts.googleapis.com/css2?family=DM+Mono:wght@400;500&display=swap');

body{font-family:var(--font);background:var(--bg);color:var(--txt);
  display:flex;flex-direction:column;align-items:center;
  padding:10px 10px 32px;gap:8px;min-height:100dvh;overflow-x:hidden}

/* ── Header ── */
header{width:100%;max-width:500px;display:flex;align-items:center;justify-content:space-between;padding:2px 0 6px}
.logo{font-size:1rem;font-weight:500;letter-spacing:.05em;color:#60a5fa}
.logo span{color:#818cf8}
.pills{display:flex;gap:6px}
.pill{font-size:.6rem;padding:2px 7px;border-radius:20px;font-weight:500;letter-spacing:.05em;
  background:var(--sf2);color:var(--mut);border:1px solid var(--br);transition:.25s}
.pill.on{background:#0d2014;color:var(--grn);border-color:#1c4a2e}
.pill.warn{background:#1e1500;color:var(--amb);border-color:#4a3200}
.pill.joy-on{background:#1a1040;color:var(--pur);border-color:#3730a3}

/* ── Panel ── */
.panel{background:var(--sf);border:1px solid var(--br);border-radius:var(--r);
  width:100%;max-width:500px;padding:12px 14px}
.ptitle{font-size:.65rem;font-weight:500;color:var(--mut);letter-spacing:.1em;
  text-transform:uppercase;margin-bottom:10px;display:flex;align-items:center;gap:6px}
.ptitle .dot{width:6px;height:6px;border-radius:50%;flex-shrink:0}

/* ── Mode tabs ── */
.tabs{display:grid;grid-template-columns:repeat(3,1fr);gap:5px}
.tab{background:var(--sf2);color:var(--mut);border:1px solid var(--br);border-radius:8px;
  padding:9px 0 8px;font-size:.72rem;font-family:var(--font);cursor:pointer;
  display:flex;flex-direction:column;align-items:center;gap:4px;font-weight:500;transition:.15s;line-height:1}
.tab .ti{font-size:1.2rem}
.tab:hover{color:var(--txt);border-color:#2d3748}
.tab.active-web{background:#0f2040;color:#93c5fd;border-color:var(--acc)}
.tab.active-joy{background:#180f38;color:#c4b5fd;border-color:var(--pur)}
.tab.active-auto{background:#0a1f0e;color:#86efac;border-color:var(--grn)}

/* ── Status bar ── */
.sbar{display:flex;align-items:center;gap:7px;margin-top:8px;padding:7px 10px;
  background:var(--sf2);border-radius:7px;font-size:.72rem;border:1px solid var(--br)}
.sbar .led{width:7px;height:7px;border-radius:50%;flex-shrink:0;transition:.3s}
.led-g{background:var(--grn)} .led-y{background:var(--amb)} .led-r{background:var(--red)} .led-b{background:var(--acc)}
#modeLabel{flex:1;font-weight:500}
#fsmState{color:var(--mut);font-size:.68rem}

/* ── Sliders ── */
.sl-row{display:flex;align-items:center;gap:8px;margin:5px 0}
.sl-row label{font-size:.7rem;color:var(--mut);min-width:90px}
.sl-row input[type=range]{flex:1;accent-color:var(--acc);cursor:pointer}
.sl-row .sv{font-size:.72rem;font-weight:500;color:var(--acc);min-width:38px;text-align:right}

/* ── D-pad ── */
.dpad{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-top:8px}
.db{background:var(--sf2);color:var(--txt);border:1px solid var(--br);border-radius:9px;
  padding:15px 0;font-size:1.25rem;cursor:pointer;user-select:none;touch-action:manipulation;
  transition:background .08s,transform .08s,border-color .08s}
.db:active,.db.on{background:#102040;border-color:var(--acc);transform:scale(.94)}
.db.stp{border-color:#3a1010;color:var(--red)}
.db.stp:active,.db.stp.on{background:#2a0a0a;border-color:var(--red)}

/* ── Ultrasonic radar ── */
.radar-wrap{display:flex;flex-direction:column;align-items:center;gap:10px}
canvas#radar{display:block}
.sonar-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;width:100%}
.sonar-cell{background:var(--sf2);border-radius:7px;padding:7px 6px;text-align:center;border:1px solid var(--br)}
.sonar-cell .sname{font-size:.6rem;color:var(--mut);margin-bottom:3px}
.sonar-cell .sval{font-size:.9rem;font-weight:500}

/* ── Joystick panel ── */
.joy-info{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:4px}
.joy-box{background:var(--sf2);border-radius:8px;padding:10px;border:1px solid var(--br)}
.joy-box .jlabel{font-size:.65rem;color:var(--mut);margin-bottom:4px}
.joy-bar-wrap{height:6px;background:var(--br);border-radius:3px;overflow:hidden;margin-top:6px}
.joy-bar{height:100%;background:var(--pur);border-radius:3px;transition:width .08s;width:50%}
.joy-val{font-size:.85rem;font-weight:500;color:var(--pur)}
.joy-params{margin-top:10px;padding-top:10px;border-top:1px solid var(--br)}
.param-row{display:flex;align-items:center;gap:8px;margin:5px 0}
.param-row label{font-size:.7rem;color:var(--mut);min-width:100px}
.param-row input[type=range]{flex:1;accent-color:var(--pur)}
.param-row .pv{font-size:.72rem;font-weight:500;color:var(--pur);min-width:38px;text-align:right}
.apply-btn{margin-top:8px;width:100%;padding:8px;background:#1a1040;color:#c4b5fd;
  border:1px solid #3730a3;border-radius:7px;font-family:var(--font);font-size:.72rem;
  cursor:pointer;transition:.15s;letter-spacing:.04em}
.apply-btn:hover{background:#231550}
.apply-btn:active{transform:scale(.98)}

/* ── Auto panel ── */
.auto-fsm{display:grid;grid-template-columns:repeat(5,1fr);gap:4px;margin-top:8px}
.fsm-node{background:var(--sf2);border:1px solid var(--br);border-radius:6px;
  padding:6px 2px;text-align:center;font-size:.6rem;color:var(--mut);transition:.3s}
.fsm-node.active{background:#0a1f0e;color:#86efac;border-color:var(--grn)}

/* ── Sensor row ── */
.sens-row{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}
.s-cell{background:var(--sf2);border-radius:7px;padding:8px;text-align:center;border:1px solid var(--br)}
.s-cell .sn{font-size:.6rem;color:var(--mut);margin-bottom:3px}
.s-cell .sv{font-size:.8rem;font-weight:500}
</style>
</head>
<body>

<header>
  <div class="logo">ROBOT<span>OS</span></div>
  <div class="pills">
    <span class="pill" id="pOnline">OFFLINE</span>
    <span class="pill" id="pJoy">JOY</span>
    <span class="pill" id="pAuto">AUTO</span>
  </div>
</header>

<!-- ── MODE SELECTOR ─────────────────────────────────────── -->
<div class="panel">
  <div class="ptitle"><div class="dot led-b"></div>Режим управления</div>
  <div class="tabs">
    <button class="tab" id="tabWeb"  onclick="setMode('WEB')">
      <span class="ti">◈</span>Web
    </button>
    <button class="tab" id="tabJoy"  onclick="setMode('JOY')">
      <span class="ti">◎</span>Джойстик
    </button>
    <button class="tab" id="tabAuto" onclick="setMode('AUTO')">
      <span class="ti">◉</span>Авто
    </button>
  </div>
  <div class="sbar">
    <div class="led led-b" id="modeLed"></div>
    <span id="modeLabel">Инициализация...</span>
    <span id="fsmState"></span>
  </div>
</div>

<!-- ── WEB: SPEED + DPAD ─────────────────────────────────── -->
<div class="panel" id="panelWeb">
  <div class="ptitle"><div class="dot led-b"></div>Скорость</div>
  <div class="sl-row">
    <label>Линейная</label>
    <input id="sLin" type="range" min="0.02" max="0.25" step="0.01" value="0.10">
    <span class="sv" id="vLin">0.10</span>
  </div>
  <div class="sl-row">
    <label>Угловая</label>
    <input id="sAng" type="range" min="0.10" max="1.50" step="0.05" value="0.60">
    <span class="sv" id="vAng">0.60</span>
  </div>
  <div class="dpad">
    <div></div>
    <button class="db" data-d="f">↑</button>
    <div></div>
    <button class="db" data-d="l">←</button>
    <button class="db stp" data-d="s">■</button>
    <button class="db" data-d="r">→</button>
    <div></div>
    <button class="db" data-d="b">↓</button>
    <div></div>
  </div>
  <div style="text-align:center;margin-top:8px;font-size:.65rem;color:var(--mut)">
    ↑↓←→ клавиатура · Пробел = стоп
  </div>
</div>

<!-- ── JOY PANEL ─────────────────────────────────────────── -->
<div class="panel" id="panelJoy" style="display:none">
  <div class="ptitle"><div class="dot led-y" id="joyLed"></div>Физический джойстик (joy_node → /joy)</div>
  <div class="joy-info">
    <div class="joy-box">
      <div class="jlabel">Линейная (ось 1)</div>
      <div class="joy-val" id="jAxLin">0.000</div>
      <div class="joy-bar-wrap"><div class="joy-bar" id="jBarLin"></div></div>
    </div>
    <div class="joy-box">
      <div class="jlabel">Угловая (ось 0)</div>
      <div class="joy-val" id="jAxAng">0.000</div>
      <div class="joy-bar-wrap"><div class="joy-bar" id="jBarAng" style="background:var(--acc)"></div></div>
    </div>
  </div>
  <div class="joy-params">
    <div class="ptitle" style="margin-bottom:8px"><div class="dot led-y"></div>Масштабы (joystick_control)</div>
    <div class="param-row">
      <label>linear_scale</label>
      <input id="pLin" type="range" min="0.05" max="1.0" step="0.05" value="0.50">
      <span class="pv" id="pvLin">0.50</span>
    </div>
    <div class="param-row">
      <label>angular_scale</label>
      <input id="pAng" type="range" min="0.10" max="3.0" step="0.10" value="1.50">
      <span class="pv" id="pvAng">1.50</span>
    </div>
    <button class="apply-btn" onclick="applyJoyParams()">▶ Применить параметры</button>
  </div>
  <div style="margin-top:10px;font-size:.68rem;color:var(--mut);line-height:1.6">
    joy_node публикует /joy · joystick_control.py читает и пишет в /cmd_vel/manual<br>
    cmd_vel_mux выдаёт приоритет ручному управлению
  </div>
</div>

<!-- ── AUTO PANEL ─────────────────────────────────────────── -->
<div class="panel" id="panelAuto" style="display:none">
  <div class="ptitle"><div class="dot led-r" id="autoDotTitle"></div>Автоисследование (auto_explorer v4)</div>
  <div class="sbar" style="margin-top:0">
    <div class="led led-r" id="autoLed"></div>
    <span id="autoStatus" style="flex:1;font-size:.75rem">Не запущено</span>
    <button onclick="stopAuto()" style="background:#2a0808;color:#fca5a5;border:1px solid #5c1515;
      border-radius:6px;padding:3px 9px;font-size:.68rem;cursor:pointer;font-family:var(--font)">СТОП</button>
  </div>
  <div class="auto-fsm" style="margin-top:10px">
    <div class="fsm-node" id="fsm_FORWARD">FORWARD</div>
    <div class="fsm-node" id="fsm_AVOID">AVOID</div>
    <div class="fsm-node" id="fsm_ROTATE">ROTATE</div>
    <div class="fsm-node" id="fsm_WALL_FOLLOW">WALL</div>
    <div class="fsm-node" id="fsm_ESCAPE">ESCAPE</div>
  </div>
</div>

<!-- ── ULTRASONIC RADAR ──────────────────────────────────── -->
<div class="panel">
  <div class="ptitle"><div class="dot led-y" id="sonarDot"></div>Ультразвуковые датчики</div>
  <div class="radar-wrap">
    <canvas id="radar" width="220" height="220"></canvas>
    <div class="sonar-grid">
      <div class="sonar-cell"><div class="sname">FL</div><div class="sval" id="uFL">—</div></div>
      <div class="sonar-cell"><div class="sname">FR</div><div class="sval" id="uFR">—</div></div>
      <div class="sonar-cell"><div class="sname">BL</div><div class="sval" id="uBL">—</div></div>
      <div class="sonar-cell"><div class="sname">BR</div><div class="sval" id="uBR">—</div></div>
    </div>
  </div>
</div>

<!-- ── SENSORS ─────────────────────────────────────────────── -->
<div class="panel">
  <div class="ptitle"><div class="dot led-b"></div>Топики ROS 2</div>
  <div class="sens-row">
    <div class="s-cell"><div class="sn">LIDAR /scan</div><div class="sv" id="sScan">—</div></div>
    <div class="s-cell"><div class="sn">MAP /map</div><div class="sv"   id="sMap">—</div></div>
    <div class="s-cell"><div class="sn">ODOM /odom</div><div class="sv" id="sOdom">—</div></div>
  </div>
</div>

<script>
// ═══════════════════════════════════════════════════════════════
// State
// ═══════════════════════════════════════════════════════════════
let mode = 'WEB';
let holdTimer = null;
let lastFsm   = '';
let sonarData = {front_left:null,front_right:null,back_left:null,back_right:null};

// ── Slider bindings ──────────────────────────────────────────
function bindSlider(id, outId, fixed=2) {
  const el = document.getElementById(id);
  const ov = document.getElementById(outId);
  el.oninput = () => ov.textContent = parseFloat(el.value).toFixed(fixed);
}
bindSlider('sLin','vLin'); bindSlider('sAng','vAng');
bindSlider('pLin','pvLin'); bindSlider('pAng','pvAng');

// ── API helper ───────────────────────────────────────────────
function api(url, data={}) {
  return fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)})
    .then(r=>r.json()).catch(()=>null);
}

// ═══════════════════════════════════════════════════════════════
// Mode switching
// ═══════════════════════════════════════════════════════════════
function setMode(m) {
  stopHold();
  if (mode === 'AUTO' && m !== 'AUTO') stopAuto();
  mode = m;

  document.getElementById('panelWeb').style.display  = m === 'WEB'  ? '' : 'none';
  document.getElementById('panelJoy').style.display  = m === 'JOY'  ? '' : 'none';
  document.getElementById('panelAuto').style.display = m === 'AUTO' ? '' : 'none';

  ['tabWeb','tabJoy','tabAuto'].forEach(id => {
    document.getElementById(id).className = 'tab';
  });
  const cls = {WEB:'active-web', JOY:'active-joy', AUTO:'active-auto'};
  const ids = {WEB:'tabWeb', JOY:'tabJoy', AUTO:'tabAuto'};
  document.getElementById(ids[m]).classList.add(cls[m]);

  const leds  = {WEB:'led-b', JOY:'led-y', AUTO:'led-g'};
  const labs  = {WEB:'Ручной — Web', JOY:'Ручной — Джойстик', AUTO:'Автоисследование'};
  document.getElementById('modeLed').className   = 'led ' + leds[m];
  document.getElementById('modeLabel').textContent = labs[m];

  if (m === 'AUTO') startAuto();
  api('/api/mode', {mode: m});
}

// ═══════════════════════════════════════════════════════════════
// D-pad (Web mode)
// ═══════════════════════════════════════════════════════════════
function sendDir(d) {
  if (mode !== 'WEB') return;
  const lin = parseFloat(document.getElementById('sLin').value);
  const ang = parseFloat(document.getElementById('sAng').value);
  const map  = {f:{linear_x:lin,angular_z:0}, b:{linear_x:-lin,angular_z:0},
                l:{linear_x:0,angular_z:ang},  r:{linear_x:0,angular_z:-ang}};
  api('/api/move', map[d] || {linear_x:0,angular_z:0});
}

function stopHold() {
  if (holdTimer) { clearInterval(holdTimer); holdTimer = null; }
  document.querySelectorAll('.db').forEach(b => b.classList.remove('on'));
}

function startHold(d) {
  if (mode !== 'WEB') return;
  stopHold();
  if (d === 's') { api('/api/stop'); return; }
  sendDir(d);
  document.querySelector(`[data-d="${d}"]`)?.classList.add('on');
  holdTimer = setInterval(() => sendDir(d), 80);
}

document.querySelectorAll('.db').forEach(btn => {
  const d = btn.dataset.d;
  const doStop = () => { stopHold(); if (d !== 's') api('/api/stop'); };
  btn.addEventListener('mousedown',  () => d === 's' ? (stopHold(), api('/api/stop')) : startHold(d));
  btn.addEventListener('mouseup',    doStop);
  btn.addEventListener('mouseleave', doStop);
  btn.addEventListener('touchstart', e => { e.preventDefault(); d === 's' ? (stopHold(), api('/api/stop')) : startHold(d); }, {passive:false});
  btn.addEventListener('touchend',   doStop);
  btn.addEventListener('touchcancel',doStop);
});

document.addEventListener('keydown', e => {
  if (e.repeat || mode !== 'WEB') return;
  const map = {'ArrowUp':'f','ArrowDown':'b','ArrowLeft':'l','ArrowRight':'r',' ':'s'};
  if (map[e.key]) { e.preventDefault(); map[e.key] === 's' ? (stopHold(), api('/api/stop')) : startHold(map[e.key]); }
});
document.addEventListener('keyup', e => {
  const moves = ['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'];
  if (moves.includes(e.key)) { stopHold(); api('/api/stop'); }
});

// ═══════════════════════════════════════════════════════════════
// Joystick params (send to ROS node via ros2 param set)
// ═══════════════════════════════════════════════════════════════
function applyJoyParams() {
  const lin = parseFloat(document.getElementById('pLin').value);
  const ang = parseFloat(document.getElementById('pAng').value);
  api('/api/joy/params', {linear_scale: lin, angular_scale: ang}).then(r => {
    if (r && r.ok) {
      const btn = document.querySelector('.apply-btn');
      btn.textContent = '✓ Применено';
      setTimeout(() => btn.textContent = '▶ Применить параметры', 1500);
    }
  });
}

// ═══════════════════════════════════════════════════════════════
// Auto explorer
// ═══════════════════════════════════════════════════════════════
function startAuto() {
  api('/api/auto/start').then(() => {
    document.getElementById('autoLed').className = 'led led-g';
    document.getElementById('autoStatus').textContent = 'Запущено';
  });
}
function stopAuto() {
  api('/api/auto/stop').then(() => {
    document.getElementById('autoLed').className = 'led led-r';
    document.getElementById('autoStatus').textContent = 'Остановлено';
    document.getElementById('fsmState').textContent = '';
    document.querySelectorAll('.fsm-node').forEach(n => n.classList.remove('active'));
  });
}

// ═══════════════════════════════════════════════════════════════
// Ultrasonic radar drawing
// ═══════════════════════════════════════════════════════════════
const radarCanvas = document.getElementById('radar');
const rc = radarCanvas.getContext('2d');
const RC = 110; // center
const MAX_M = 1.5; // clip at 1.5m for display

function sonarColor(m) {
  if (m === null) return '#1e2430';
  if (m < 0.25) return '#7f1d1d';
  if (m < 0.50) return '#78350f';
  if (m < 1.00) return '#14532d';
  return '#1e3a5f';
}

function drawRadar() {
  const W = 220;
  rc.clearRect(0, 0, W, W);

  // Grid circles
  [0.25, 0.5, 1.0, 1.5].forEach(r => {
    const px = (r / MAX_M) * 80;
    rc.beginPath(); rc.arc(RC, RC, px, 0, 2*Math.PI);
    rc.strokeStyle = '#1e2430'; rc.lineWidth = 1; rc.stroke();
    rc.fillStyle = '#2d3748'; rc.font = '9px DM Mono,monospace';
    rc.fillText(r+'m', RC + px + 2, RC);
  });

  // Cross lines
  rc.strokeStyle = '#1a2030'; rc.lineWidth = 1;
  [[RC,10,RC,210],[10,RC,210,RC]].forEach(([x1,y1,x2,y2]) => {
    rc.beginPath(); rc.moveTo(x1,y1); rc.lineTo(x2,y2); rc.stroke();
  });

  // Robot body
  rc.fillStyle = '#1e2d40';
  rc.beginPath(); rc.roundRect(RC-14, RC-18, 28, 36, 5); rc.fill();
  rc.strokeStyle = '#3b82f6'; rc.lineWidth = 1.5; rc.stroke();
  // Front arrow
  rc.fillStyle = '#3b82f6';
  rc.beginPath(); rc.moveTo(RC, RC-22); rc.lineTo(RC-5, RC-16); rc.lineTo(RC+5, RC-16); rc.closePath(); rc.fill();

  // Sensor arcs
  // front_left: ~315° (-45°), front_right: ~45°, back_left: ~225°, back_right: ~135°
  // In canvas: 0=right, 90=down → robot up=270°(canvas)
  // front=270°, right=0°, back=90°, left=180°
  const sensors = [
    {key:'front_left',  start: 225, end: 270, id:'uFL'},
    {key:'front_right', start: 270, end: 315, id:'uFR'},
    {key:'back_right',  start:  45, end:  90, id:'uBR'},
    {key:'back_left',   start:  90, end: 135, id:'uBL'},
  ];

  sensors.forEach(({key, start, end, id}) => {
    const m = sonarData[key];
    const dist = (m !== null && m > 0) ? Math.min(m, MAX_M) : MAX_M;
    const px = (dist / MAX_M) * 78;
    const s  = (start - 90) * Math.PI / 180;
    const e  = (end   - 90) * Math.PI / 180;

    rc.beginPath();
    rc.moveTo(RC, RC);
    rc.arc(RC, RC, px, s, e);
    rc.closePath();
    rc.fillStyle = sonarColor(m); rc.fill();
    rc.strokeStyle = '#2d3748'; rc.lineWidth = 1; rc.stroke();

    // Value label
    const ma = (s + e) / 2;
    const lr = px * 0.6;
    const tx = RC + Math.cos(ma) * lr;
    const ty = RC + Math.sin(ma) * lr;
    rc.fillStyle = '#94a3b8'; rc.font = '9px DM Mono,monospace'; rc.textAlign = 'center';
    rc.fillText(m !== null ? m.toFixed(2) : '—', tx, ty);

    // Update cell
    const cell = document.getElementById(id);
    if (cell) {
      const color = m === null ? '#4b5568' : m < 0.25 ? '#f87171' : m < 0.5 ? '#fbbf24' : '#22c55e';
      cell.style.color = color;
      cell.textContent = m !== null ? m.toFixed(2)+'m' : '—';
    }
  });
  rc.textAlign = 'left';
}

drawRadar();

// ═══════════════════════════════════════════════════════════════
// Age helper for topic timestamps
// ═══════════════════════════════════════════════════════════════
function ageHtml(t) {
  if (!t) return '<span style="color:var(--red)">OFF</span>';
  const a = Date.now()/1000 - t;
  const c = a < 1 ? 'var(--grn)' : a < 3 ? 'var(--amb)' : 'var(--red)';
  return `<span style="color:${c}">${a.toFixed(1)}s</span>`;
}

// ═══════════════════════════════════════════════════════════════
// Status polling (500ms)
// ═══════════════════════════════════════════════════════════════
setInterval(async () => {
  try {
    const d = await fetch('/api/status').then(r => r.json());

    // Online pill
    const online = d.scan || d.odom;
    const po = document.getElementById('pOnline');
    po.textContent = online ? 'ONLINE' : 'OFFLINE';
    po.className = 'pill ' + (online ? 'on' : '');

    // Topic ages
    document.getElementById('sScan').innerHTML = ageHtml(d.scan);
    document.getElementById('sMap').innerHTML  = ageHtml(d.map);
    document.getElementById('sOdom').innerHTML = ageHtml(d.odom);

    // Joy pill
    const pj = document.getElementById('pJoy');
    pj.className = 'pill ' + (d.joy_active ? 'joy-on' : '');

    // Auto pill
    const pa = document.getElementById('pAuto');
    pa.className = 'pill ' + (d.auto_run ? 'on' : '');

    // Joystick axes (displayed in JOY panel)
    if (d.joy_axes && d.joy_axes.length >= 2) {
      const ax0 = -d.joy_axes[0]; // angular raw
      const ax1 = -d.joy_axes[1]; // linear raw
      document.getElementById('jAxLin').textContent = ax1.toFixed(3);
      document.getElementById('jAxAng').textContent = ax0.toFixed(3);
      // bars: map [-1,1] to [0,100]%
      document.getElementById('jBarLin').style.width = ((ax1+1)/2*100).toFixed(0)+'%';
      document.getElementById('jBarAng').style.width = ((ax0+1)/2*100).toFixed(0)+'%';
      const jled = document.getElementById('joyLed');
      jled.className = 'led ' + (d.joy_active ? 'led-g' : 'led-y');
    }

    // Ultrasonic
    if (d.sonar) {
      sonarData = d.sonar;
      const anyFresh = Object.values(d.sonar_times||{}).some(t => t && (Date.now()/1000 - t) < 2);
      document.getElementById('sonarDot').className = 'dot ' + (anyFresh ? 'led-g' : 'led-r');
      drawRadar();
    }

    // FSM state (auto explorer)
    if (d.explorer_state !== undefined) {
      const fsm = d.explorer_state;
      document.getElementById('fsmState').textContent = fsm ? '· '+fsm : '';
      document.querySelectorAll('.fsm-node').forEach(n => {
        n.classList.toggle('active', n.id === 'fsm_'+fsm);
      });
      if (d.auto_run) {
        document.getElementById('autoStatus').textContent = fsm ? 'Активно: '+fsm : 'Запущено';
        document.getElementById('autoLed').className = 'led led-g';
      } else if (mode === 'AUTO') {
        document.getElementById('autoLed').className = 'led led-r';
        document.getElementById('autoStatus').textContent = 'Процесс завершён';
      }
    }

    // Sync joy param sliders from server defaults (once)
    if (d.joy_params && !window._joyParamsSynced) {
      window._joyParamsSynced = true;
      const pl = document.getElementById('pLin');
      const pa2 = document.getElementById('pAng');
      pl.value = d.joy_params.linear_scale;
      pa2.value = d.joy_params.angular_scale;
      document.getElementById('pvLin').textContent = parseFloat(d.joy_params.linear_scale).toFixed(2);
      document.getElementById('pvAng').textContent = parseFloat(d.joy_params.angular_scale).toFixed(2);
    }
  } catch(e) {}
}, 500);

// Init
setMode('WEB');
</script>
</body>
</html>
"""

# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 Node
# ─────────────────────────────────────────────────────────────────────────────

class WebControlNode(Node):
    def __init__(self):
        super().__init__('web_control')

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_cmd         = self.create_publisher(Twist, '/cmd_vel/manual', 10)
        self.pub_auto_enable = self.create_publisher(Bool,  '/auto_enable',    10)

        # ── Topic timestamps ─────────────────────────────────────────────────
        self.t_scan = self.t_map = self.t_odom = None

        # ── Joystick state (from /joy — joy_node running on RPi) ─────────────
        self.joy_axes    = []
        self.joy_buttons = []
        self.joy_active  = False
        self.t_joy       = 0.0

        # ── Joystick control params (mirrored from joystick_control.py) ──────
        self.joy_linear_scale  = 0.5   # default matches joystick_control.py
        self.joy_angular_scale = 1.5

        # ── Ultrasonic ───────────────────────────────────────────────────────
        self.sonar = {
            'front_left':  None,
            'front_right': None,
            'back_left':   None,
            'back_right':  None,
        }
        self.sonar_times = {k: None for k in self.sonar}

        # ── Explorer / mux state ─────────────────────────────────────────────
        self.explorer_state = ''
        self.control_mode   = 'IDLE'

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(LaserScan,     '/scan',           lambda m: setattr(self,'t_scan',time.time()), 10)
        self.create_subscription(OccupancyGrid, '/map',            lambda m: setattr(self,'t_map', time.time()), 10)
        self.create_subscription(Odometry,      '/odom',           lambda m: setattr(self,'t_odom',time.time()), 10)
        self.create_subscription(String,        '/explorer_state', self._cb_explorer, 10)
        self.create_subscription(String,        '/control_mode',   self._cb_mode,     10)
        self.create_subscription(Joy,           '/joy',            self._cb_joy,       10)

        for name in self.sonar:
            self.create_subscription(Range, f'/ultrasonic/{name}', self._make_sonar_cb(name), 10)

        # ── Keep-alive for web D-pad ─────────────────────────────────────────
        # Publishes last web command at 20 Hz so cmd_vel_mux doesn't timeout
        self._last_twist    = Twist()
        self._manual_active = False
        self.create_timer(0.05, self._keepalive)

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _cb_explorer(self, msg: String): self.explorer_state = msg.data
    def _cb_mode(self,     msg: String): self.control_mode   = msg.data

    def _cb_joy(self, msg: Joy):
        self.joy_axes    = list(msg.axes)
        self.joy_buttons = list(msg.buttons)
        self.t_joy       = time.time()
        self.joy_active  = True

    def _make_sonar_cb(self, name: str):
        def cb(msg: Range):
            self.sonar[name]       = round(float(msg.range), 3)
            self.sonar_times[name] = time.time()
        return cb

    # ── Keep-alive ───────────────────────────────────────────────────────────
    def _keepalive(self):
        if self._manual_active:
            self.pub_cmd.publish(self._last_twist)
        # Detect joy timeout
        if self.joy_active and (time.time() - self.t_joy) > 2.0:
            self.joy_active = False

    # ── Control helpers ──────────────────────────────────────────────────────
    def move(self, lx: float, az: float):
        self._last_twist.linear.x  = float(lx)
        self._last_twist.angular.z = float(az)
        self._manual_active = True
        self.pub_cmd.publish(self._last_twist)

    def set_auto_enable(self, enabled: bool):
        """Публикуем /auto_enable несколько раз для надёжности."""
        msg = Bool()
        msg.data = enabled
        for _ in range(5):
            self.pub_auto_enable.publish(msg)

    def stop(self):
        self._last_twist    = Twist()
        self._manual_active = False
        self.pub_cmd.publish(Twist())


# ─────────────────────────────────────────────────────────────────────────────
# Flask
# ─────────────────────────────────────────────────────────────────────────────

app = Flask(__name__)
CORS(app)

node: WebControlNode | None = None
auto_proc = None
_lock = threading.Lock()


def ros_spin_thread():
    global node
    try:
        rclpy.init()
        node = WebControlNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'[web] ROS error: {e}', flush=True)
    finally:
        try:   node.destroy_node()
        except: pass
        if rclpy.ok(): rclpy.shutdown()


def shutdown_cleanup():
    global auto_proc
    with _lock:
        if auto_proc and auto_proc.poll() is None:
            auto_proc.send_signal(signal.SIGINT)
            try:    auto_proc.wait(timeout=3)
            except: auto_proc.kill(); auto_proc.wait(timeout=2)
        auto_proc = None
    if node: node.stop()


# ── Routes ────────────────────────────────────────────────────────────────────

@app.route('/')
def idx():
    r = app.make_response(render_template_string(HTML))
    r.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
    r.headers['Pragma']  = 'no-cache'
    r.headers['Expires'] = '0'
    return r


@app.route('/api/move', methods=['POST'])
def mv():
    d = request.get_json(silent=True) or {}
    if node: node.move(d.get('linear_x', 0), d.get('angular_z', 0))
    return jsonify({'ok': True})


@app.route('/api/stop', methods=['POST'])
def st():
    if node: node.stop()
    return jsonify({'ok': True})


@app.route('/api/mode', methods=['POST'])
def set_mode():
    d = request.get_json(silent=True) or {}
    m = d.get('mode', 'WEB')
    if m != 'WEB' and node:
        node.stop()
    return jsonify({'mode': m})


@app.route('/api/auto/start', methods=['POST'])
def api_start():
    global auto_proc
    with _lock:
        if auto_proc and auto_proc.poll() is None:
            return jsonify({'status': 'running'})
        # Сначала сигнализируем мультиплексору ДО запуска процесса
        if node:
            node.set_auto_enable(True)
        auto_proc = subprocess.Popen(
            ['ros2', 'run', 'robot_mapper', 'auto_explorer'],
            start_new_session=True
        )
    return jsonify({'status': 'started'})


@app.route('/api/auto/stop', methods=['POST'])
def api_stop():
    global auto_proc
    # ШАГ 1: НЕМЕДЛЕННО сообщаем мультиплексору — авто выключено.
    # Мультиплексор перестаёт пропускать /cmd_vel/auto до того,
    # как процесс успеет прислать ещё одну команду.
    if node:
        node.set_auto_enable(False)
        node.stop()   # явный ноль в /cmd_vel/manual тоже

    # ШАГ 2: убиваем процесс (уже не страшно если он пришлёт ещё команду —
    # мультиплексор её заблокировал)
    with _lock:
        if auto_proc and auto_proc.poll() is None:
            auto_proc.send_signal(signal.SIGINT)
            try:    auto_proc.wait(timeout=3)
            except: auto_proc.kill(); auto_proc.wait(timeout=2)
        auto_proc = None

    return jsonify({'status': 'stopped'})


@app.route('/api/joy/params', methods=['GET', 'POST'])
def joy_params():
    """GET: return current scales. POST: apply via ros2 param set."""
    if not node:
        return jsonify({'ok': False, 'error': 'node not ready'})

    if request.method == 'GET':
        return jsonify({
            'linear_scale':  node.joy_linear_scale,
            'angular_scale': node.joy_angular_scale,
        })

    d = request.get_json(silent=True) or {}
    lin = float(d.get('linear_scale',  node.joy_linear_scale))
    ang = float(d.get('angular_scale', node.joy_angular_scale))

    node.joy_linear_scale  = lin
    node.joy_angular_scale = ang

    # Apply to running joystick_control node via ros2 param set
    errors = []
    for param, val in [('linear_scale', lin), ('angular_scale', ang)]:
        try:
            result = subprocess.run(
                ['ros2', 'param', 'set', '/joystick_control', param, str(val)],
                capture_output=True, timeout=3
            )
            if result.returncode != 0:
                errors.append(f'{param}: {result.stderr.decode().strip()}')
        except Exception as e:
            errors.append(f'{param}: {e}')

    return jsonify({'ok': len(errors) == 0, 'errors': errors,
                    'linear_scale': lin, 'angular_scale': ang})


@app.route('/api/status')
def status():
    is_auto = auto_proc is not None and auto_proc.poll() is None
    if not node:
        return jsonify({
            'scan': None, 'map': None, 'odom': None,
            'auto_run': False, 'explorer_state': '', 'control_mode': 'IDLE',
            'joy_active': False, 'joy_axes': [], 'joy_buttons': [],
            'sonar': {}, 'sonar_times': {},
            'joy_params': {'linear_scale': 0.5, 'angular_scale': 1.5},
        })
    return jsonify({
        'scan':           node.t_scan,
        'map':            node.t_map,
        'odom':           node.t_odom,
        'auto_run':       is_auto,
        'explorer_state': node.explorer_state,
        'control_mode':   node.control_mode,
        'joy_active':     node.joy_active,
        'joy_axes':       node.joy_axes,
        'joy_buttons':    node.joy_buttons,
        'sonar':          node.sonar,
        'sonar_times':    node.sonar_times,
        'joy_params': {
            'linear_scale':  node.joy_linear_scale,
            'angular_scale': node.joy_angular_scale,
        },
    })


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()
    while node is None:
        time.sleep(0.05)
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[web] Flask error: {e}', flush=True)
    finally:
        shutdown_cleanup()


if __name__ == '__main__':
    main()
