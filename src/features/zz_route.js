// ============================================================================
// Mapping mission: an area route planner in the style of DJI Pilot 2 / DJI FlightHub, and an automatic flight
// that carries the EO, LWIR and LiDAR payload along the route
// ============================================================================
// Planning (top-down map of the live scene):
//   - click on the sea to add boundary points (3 or more); drag a point to move it, drag a "+" midpoint to add one,
//     Delete removes the selected point. The polygon is the survey area; the margin grows or shrinks it;
//   - the route is generated automatically: parallel flight lines inside the area, flown back and forth (a zigzag /
//     lawnmower pattern). Line direction = course angle (clockwise from north); line spacing comes from the camera
//     footprint at the flight altitude and the side overlap, the photo interval from the front overlap, as DJI does:
//       footprint across track W = 2 h tan(HFOV / 2), spacing = W (1 - side overlap),
//       footprint along track  F = 2 h tan(VFOV / 2), photo interval = F (1 - front overlap);
//   - speed 5-100 m/s (stepless), altitude, gimbal pitch, completion action (return to home / hover / repeat), start
//     corner, and optional recording of the three sensors during the flight.
// Flight: the aircraft flies the route at constant altitude and speed (heading follows the route with a limited yaw
// rate); the main view and the three sensor windows are the gimbal view. The mouse only tilts the gimbal (drag up /
// down); Space pauses; Esc ends the flight. The aircraft takes off from, and returns to, the start point S.
Object.assign(params, {
  rtAlt: 100, rtSpeed: 15, rtAngle: 0, rtSide: 70, rtFront: 80, rtMargin: 0, rtPitch: -90, rtFinish: 'rth', rtRecord: false, rtStart: 0,
});
const RT_KEY = 'oceanMob.route';
const RT = {
  mode: 'off', verts: [], sel: -1, drag: null, map: { cx: 0, cz: 0, alt: 500 }, route: null, saved: null, fly: null,
};
try {
  const s = JSON.parse(localStorage.getItem(RT_KEY) || 'null');
  if (s && Array.isArray(s.verts)) { RT.verts = s.verts.filter(v => Number.isFinite(v.x) && Number.isFinite(v.z)); Object.assign(params, s.params || {}); }
} catch (e) { /* no saved route */ }
function rtSave() { try { localStorage.setItem(RT_KEY, JSON.stringify({ verts: RT.verts, params: Object.fromEntries(Object.keys(params).filter(k => k.startsWith('rt')).map(k => [k, params[k]])) })); } catch (e) { /* storage blocked */ } }

// ---------------------------------------------------------------------------- geometry
const RT_MAP_FOV = 40;
function rtFlightFov() { return RT.saved ? RT.saved.fov : camera.fov; }                 // the gimbal camera's vertical FOV
function rtArea(V) { let a = 0; for (let i = 0; i < V.length; i++) { const p = V[i], q = V[(i + 1) % V.length]; a += p.x * q.z - q.x * p.z; } return a / 2; }
// offset a polygon by m metres (outwards for m > 0), mitred corners with a limited miter
function rtOffset(V, m) {
  if (!m || V.length < 3) return V.map(v => ({ ...v }));
  const s = Math.sign(rtArea(V)) || 1, n = V.length, N = [];
  for (let i = 0; i < n; i++) {
    const a = V[i], b = V[(i + 1) % n], ex = b.x - a.x, ez = b.z - a.z, l = Math.hypot(ex, ez) || 1;
    N.push({ x: s * ez / l, z: -s * ex / l });
  }
  return V.map((v, i) => {
    const n1 = N[(i + n - 1) % n], n2 = N[i], d = 1 + n1.x * n2.x + n1.z * n2.z, k = m / Math.max(d, 0.25);
    return { x: v.x + (n1.x + n2.x) * k, z: v.z + (n1.z + n2.z) * k };
  });
}
// footprint and spacing of the gimbal camera pointing straight down (5:4 sensor)
function rtFootprint() {
  const vf = rtFlightFov() * D2R, hf = 2 * Math.atan(Math.tan(vf / 2) * SENSOR_W / SENSOR_H), h = params.rtAlt;
  const across = 2 * h * Math.tan(hf / 2), along = 2 * h * Math.tan(vf / 2);
  return { across, along, spacing: Math.max(1, across * (1 - params.rtSide / 100)), interval: Math.max(0.5, along * (1 - params.rtFront / 100)) };
}
// the zigzag: lines parallel to the course angle across the (margin-adjusted) polygon, flown alternately
function rtBuild() {
  const V = RT.verts;
  if (V.length < 3) { RT.route = null; return; }
  const poly = rtOffset(V, params.rtMargin), psi = params.rtAngle * D2R, fp = rtFootprint();
  const dir = { x: Math.sin(psi), z: -Math.cos(psi) }, perp = { x: Math.cos(psi), z: Math.sin(psi) };   // along line, across lines
  const UV = poly.map(p => ({ u: p.x * dir.x + p.z * dir.z, v: p.x * perp.x + p.z * perp.z }));
  let vmin = Infinity, vmax = -Infinity;
  for (const p of UV) { vmin = Math.min(vmin, p.v); vmax = Math.max(vmax, p.v); }
  const range = vmax - vmin, nL = Math.max(1, Math.floor(range / fp.spacing) + 1), v0 = vmin + (range - (nL - 1) * fp.spacing) / 2;
  const lines = [];
  for (let k = 0; k < nL; k++) {
    const v = v0 + k * fp.spacing, us = [];
    for (let i = 0; i < UV.length; i++) {
      const a = UV[i], b = UV[(i + 1) % UV.length];
      if ((a.v <= v && b.v > v) || (b.v <= v && a.v > v)) us.push(a.u + (v - a.v) / (b.v - a.v) * (b.u - a.u));
    }
    if (us.length >= 2) lines.push({ v, u0: Math.min(...us), u1: Math.max(...us) });
  }
  if (!lines.length) { RT.route = null; return; }
  if (params.rtStart & 1) lines.reverse();
  const P = (u, v) => ({ x: u * dir.x + v * perp.x, z: u * dir.z + v * perp.z });
  const pts = [];
  lines.forEach((L, k) => {
    const fwd = ((k & 1) === 0) !== !!(params.rtStart & 2);
    pts.push(P(fwd ? L.u0 : L.u1, L.v), P(fwd ? L.u1 : L.u0, L.v));
  });
  let len = 0, lineLen = 0, photos = 0;
  for (let i = 1; i < pts.length; i++) len += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].z - pts[i - 1].z);
  for (const L of lines) { const l = L.u1 - L.u0; lineLen += l; photos += Math.floor(l / fp.interval) + 1; }
  const home = pts[0], last = pts[pts.length - 1];
  const rth = params.rtFinish === 'rth' ? Math.hypot(last.x - home.x, last.z - home.z) : 0;
  RT.route = { pts, len, lineLen, lines: lines.length, spacing: fp.spacing, interval: fp.interval, photos, rth,
    area: Math.abs(rtArea(V)), duration: (len + rth) / params.rtSpeed };
}

// ---------------------------------------------------------------------------- styles
const rtCss = document.createElement('style');
rtCss.textContent = `
#rtPlan { position: fixed; inset: 0; z-index: 1004; display: none; pointer-events: none; color: #e8e8ea;
  font: 13px/1.35 "Segoe UI", system-ui, "Microsoft YaHei", "PingFang SC", sans-serif; }
#rtPlan canvas { position: absolute; left: 0; top: 0; pointer-events: auto; cursor: crosshair; touch-action: none; }
#rtTop { position: absolute; left: 0; right: 0; top: 0; height: 46px; background: #000; display: flex; align-items: center; gap: 10px;
  padding: 0 14px; pointer-events: auto; border-bottom: 1px solid #222; }
#rtTop button.back { all: unset; cursor: pointer; font-size: 24px; line-height: 1; padding: 0 10px 2px 2px; color: #fff; }
#rtTop .title { font-size: 15px; font-weight: 600; }
#rtTop .hint { color: #8e8e93; font-size: 12px; margin-left: 10px; flex: 1; overflow: hidden; white-space: nowrap; text-overflow: ellipsis; }
#rtSide { position: absolute; right: 0; top: 46px; bottom: 0; width: 340px; background: #1c1c1e; overflow-y: auto; pointer-events: auto;
  padding: 4px 0 84px; box-sizing: border-box; }
.rt-sec { background: #2c2c2e; margin: 8px 10px; border-radius: 8px; padding: 11px 12px; }
.rt-lbl { display: flex; justify-content: space-between; align-items: baseline; margin-bottom: 9px; }
.rt-lbl small { color: #8e8e93; font-size: 12px; }
.rt-step { display: flex; border: 1px solid #48484a; border-radius: 5px; overflow: hidden; }
.rt-step button { all: unset; flex: 1; text-align: center; cursor: pointer; padding: 7px 0; color: #e8e8ea; font-size: 12px; }
.rt-step button + button, .rt-step input, .rt-step input + button { border-left: 1px solid #48484a; }
.rt-step button:hover, .rt-pm:hover { background: #3a3a3c; }
.rt-step input { all: unset; width: 62px; text-align: center; color: #2f8cff; font-weight: 700; font-size: 15px; background: #1c1c1e; }
.rt-slide { display: flex; align-items: center; gap: 8px; }
.rt-slide input[type=range] { flex: 1; accent-color: #f2f2f7; min-width: 60px; }
.rt-pm { all: unset; cursor: pointer; font-size: 20px; width: 24px; height: 24px; line-height: 22px; text-align: center; border-radius: 4px; }
.rt-num { all: unset; width: 58px; box-sizing: border-box; padding: 5px 6px; text-align: center; color: #2f8cff; font-weight: 700;
  border: 1px solid #48484a; border-radius: 5px; background: #1c1c1e; }
.rt-sub { color: #8e8e93; font-size: 12px; margin-top: 7px; }
.rt-sec select { width: 100%; background: #1c1c1e; color: #2f8cff; border: 1px solid #48484a; border-radius: 5px; padding: 7px; text-align-last: center; font: inherit; }
.rt-tog { display: flex; justify-content: space-between; align-items: center; gap: 10px; }
.rt-tog input { appearance: none; width: 44px; height: 26px; border-radius: 13px; background: #48484a; position: relative; cursor: pointer; flex: none; margin: 0; }
.rt-tog input::after { content: ""; position: absolute; left: 3px; top: 3px; width: 20px; height: 20px; border-radius: 50%; background: #fff; transition: left .15s; }
.rt-tog input:checked { background: #2f8cff; } .rt-tog input:checked::after { left: 21px; }
.rt-btns { display: flex; gap: 8px; }
.rt-btns button, .rt-sec > button { all: unset; flex: 1; text-align: center; cursor: pointer; padding: 8px 6px; border-radius: 5px; background: #3a3a3c; color: #e8e8ea; }
.rt-btns button:hover, .rt-sec > button:hover { background: #48484a; }
#rtGo { all: unset; position: absolute; right: 10px; bottom: 14px; width: 320px; pointer-events: auto; box-sizing: border-box; text-align: center;
  cursor: pointer; background: #2f8cff; color: #fff; font-weight: 600; font-size: 15px; padding: 12px; border-radius: 8px; }
#rtGo:disabled { background: #3a3a3c; color: #8e8e93; cursor: default; }
#rtInfo { position: absolute; left: 12px; top: 58px; background: rgba(28, 28, 30, 0.82); border-radius: 8px; padding: 8px 14px;
  display: grid; grid-template-columns: repeat(3, auto); gap: 6px 22px; pointer-events: auto; backdrop-filter: blur(6px); }
#rtInfo small { display: block; color: #aeaeb2; font-size: 11px; } #rtInfo b { font-size: 15px; font-weight: 600; }
#rtTools { position: absolute; left: 12px; top: 50%; transform: translateY(-50%); background: #f2f2f7; border-radius: 8px; display: flex;
  flex-direction: column; pointer-events: auto; overflow: hidden; box-shadow: 0 2px 10px rgba(0,0,0,.35); }
#rtTools button { all: unset; cursor: pointer; width: 42px; height: 42px; display: grid; place-items: center; color: #1c1c1e; font-size: 17px; }
#rtTools button + button { border-top: 1px solid #d1d1d6; } #rtTools button:hover { background: #e5e5ea; }
#rtLegend { position: absolute; left: 12px; bottom: 14px; background: rgba(28, 28, 30, 0.82); border-radius: 8px; padding: 7px 12px;
  pointer-events: none; font-size: 11px; color: #aeaeb2; }
#rtLegend i { display: inline-block; width: 120px; height: 6px; border-radius: 3px; margin: 0 8px; vertical-align: middle;
  background: linear-gradient(90deg, #2b5cff, #36c3ff, #ffd400, #ff8a00); }
#rtFly { position: fixed; inset: 0; z-index: 1002; display: none; pointer-events: none; color: #fff;
  font: 12px/1.3 Consolas, "Cascadia Mono", "Microsoft YaHei", monospace; }
#rtFlyBar { position: absolute; left: 50%; top: 44px; transform: translateX(-50%); display: flex; align-items: center; gap: 16px; white-space: nowrap;
  padding: 7px 14px; border-radius: 8px; background: rgba(0, 0, 0, 0.58); backdrop-filter: blur(6px); border: 1px solid rgba(255,255,255,.15); }
#rtFlyBar .st { color: #34d27b; font-weight: 700; } #rtFlyBar .st.p { color: #ffd60a; } #rtFlyBar .st.d { color: #2f8cff; }
#rtFlyBar .prog { width: 110px; height: 5px; border-radius: 3px; background: rgba(255,255,255,.2); overflow: hidden; }
#rtFlyBar .prog i { display: block; height: 100%; background: #2f8cff; }
#rtFlyBar small { color: #aeaeb2; margin-right: 4px; }
#rtFlyHint { position: absolute; left: 50%; top: 82px; transform: translateX(-50%); color: rgba(255,255,255,.75); font-size: 11px; text-shadow: 0 0 4px #000; white-space: nowrap; }
#rtGauge { position: absolute; right: 18px; top: 50%; transform: translateY(-50%); }
#rtMini { position: absolute; right: 12px; bottom: 12px; border-radius: 8px; background: rgba(0, 0, 0, 0.55); border: 1px solid rgba(255,255,255,.25); }
#rtDone { position: absolute; left: 50%; top: 40%; transform: translate(-50%, -50%); display: none; padding: 14px 22px; border-radius: 10px;
  background: rgba(0, 0, 0, 0.65); font: 600 16px "Segoe UI", system-ui, "Microsoft YaHei", sans-serif; text-align: center; }
#rtDone small { display: block; font-weight: 400; font-size: 12px; color: #c7c7cc; margin-top: 4px; }`;
document.head.appendChild(rtCss);

// ---------------------------------------------------------------------------- planner DOM
const rtPlan = document.createElement('div');
rtPlan.id = 'rtPlan';
rtPlan.innerHTML = `
<canvas></canvas>
<div id="rtTop"><button class="back" data-a="back" title="Esc">‹</button><span class="title" data-t="Mapping mission"></span><span class="hint" data-t="Click on the sea to add boundary points (3 or more). Drag a point to move it, drag + to add one, Delete removes the selected point. Drag the map to pan, wheel to zoom."></span></div>
<div id="rtInfo">
  <div><small data-t="Mapping area"></small><b data-v="area">–</b></div>
  <div><small data-t="Est. duration"></small><b data-v="dur">–</b></div>
  <div><small data-t="Est. photos"></small><b data-v="photos">–</b></div>
  <div><small data-t="Route length"></small><b data-v="len">–</b></div>
  <div><small data-t="Line spacing"></small><b data-v="spacing">–</b></div>
  <div><small data-t="Flight lines"></small><b data-v="lines">–</b></div>
</div>
<div id="rtTools">
  <button data-a="clear" data-tt="Clear area">✕</button><button data-a="del" data-tt="Delete selected point">🗑</button>
  <button data-a="rect" data-tt="Rectangle area">▭</button><button data-a="center" data-tt="Center view">⌖</button>
</div>
<div id="rtLegend"><span data-t="Start"></span><i></i><span data-t="End"></span></div>
<div id="rtSide">
  <div class="rt-sec"><div class="rt-lbl"><span data-t="Altitude"></span><small>(10~1000 m)</small></div>
    <div class="rt-step" data-k="rtAlt"><button data-d="-100">-100</button><button data-d="-10">-10</button><button data-d="-1">-1</button><input type="number" min="10" max="1000" step="1"><button data-d="1">+1</button><button data-d="10">+10</button><button data-d="100">+100</button></div>
    <div class="rt-sub" data-v="gsd"></div></div>
  <div class="rt-sec"><div class="rt-lbl"><span data-t="Speed"></span><small>(5~100 m/s)</small></div>
    <div class="rt-slide" data-k="rtSpeed" data-min="5" data-max="100" data-step="any" data-pm="0.5" data-dec="1"><button class="rt-pm">−</button><input type="range"><button class="rt-pm">+</button><input class="rt-num" type="number"></div></div>
  <div class="rt-sec"><div class="rt-lbl"><span data-t="Course angle"></span><small>(0~359°)</small></div>
    <div class="rt-slide" data-k="rtAngle" data-min="0" data-max="359" data-step="1" data-pm="1" data-dec="0"><button class="rt-pm">−</button><input type="range"><button class="rt-pm">+</button><input class="rt-num" type="number"></div></div>
  <div class="rt-sec"><div class="rt-lbl"><span data-t="Side overlap (line density)"></span><small>(10~90 %)</small></div>
    <div class="rt-slide" data-k="rtSide" data-min="10" data-max="90" data-step="1" data-pm="1" data-dec="0"><button class="rt-pm">−</button><input type="range"><button class="rt-pm">+</button><input class="rt-num" type="number"></div>
    <div class="rt-sub" data-v="sideSub"></div></div>
  <div class="rt-sec"><div class="rt-lbl"><span data-t="Front overlap"></span><small>(10~90 %)</small></div>
    <div class="rt-slide" data-k="rtFront" data-min="10" data-max="90" data-step="1" data-pm="1" data-dec="0"><button class="rt-pm">−</button><input type="range"><button class="rt-pm">+</button><input class="rt-num" type="number"></div>
    <div class="rt-sub" data-v="frontSub"></div></div>
  <div class="rt-sec"><div class="rt-lbl"><span data-t="Margin"></span><small>(-50~200 m)</small></div>
    <div class="rt-slide" data-k="rtMargin" data-min="-50" data-max="200" data-step="1" data-pm="1" data-dec="0"><button class="rt-pm">−</button><input type="range"><button class="rt-pm">+</button><input class="rt-num" type="number"></div></div>
  <div class="rt-sec"><div class="rt-lbl"><span data-t="Gimbal pitch"></span><small>(-90~0°)</small></div>
    <div class="rt-slide" data-k="rtPitch" data-min="-90" data-max="0" data-step="1" data-pm="1" data-dec="0"><button class="rt-pm">−</button><input type="range"><button class="rt-pm">+</button><input class="rt-num" type="number"></div></div>
  <div class="rt-sec"><div class="rt-lbl"><span data-t="Upon completion"></span></div>
    <select data-k="rtFinish"><option value="rth" data-t="Return to home"></option><option value="hover" data-t="Hover"></option><option value="repeat" data-t="Repeat route"></option></select></div>
  <div class="rt-sec"><div class="rt-tog"><span data-t="Record the three sensors during the flight"></span><input type="checkbox" data-k="rtRecord"></div>
    <div class="rt-sub" data-t="Uses the Data recording settings (folder, rate, formats)."></div></div>
  <div class="rt-sec rt-btns"><button data-a="start" data-t="Change start point"></button><button data-a="rect" data-t="Rectangle area"></button></div>
</div>
<button id="rtGo" data-a="go" data-t="Start flight"></button>`;
document.body.appendChild(rtPlan);
const rtCanvas = rtPlan.querySelector('canvas'), rtCtx = rtCanvas.getContext('2d');

// ---------------------------------------------------------------------------- flight HUD DOM
const rtFly = document.createElement('div');
rtFly.id = 'rtFly';
rtFly.innerHTML = `<div id="rtFlyBar"><span class="st"></span><span class="prog"><i></i></span><span data-v="pct"></span>
  <span><small data-t="ALT"></small><b data-v="alt"></b></span><span><small data-t="SPD"></small><b data-v="spd"></b></span>
  <span><small data-t="DIST"></small><b data-v="dist"></b></span><span><small data-t="ETA"></small><b data-v="eta"></b></span>
  <span><small data-t="GIMBAL"></small><b data-v="gim"></b></span></div>
<div id="rtFlyHint" data-t="Drag up / down to tilt the gimbal · Space pause · Esc exit"></div>
<canvas id="rtGauge" width="40" height="240"></canvas><canvas id="rtMini" width="240" height="240"></canvas>
<div id="rtDone"><span data-v="doneT"></span><small data-t="Press Esc to leave the flight view"></small></div>`;
document.body.appendChild(rtFly);
const rtGauge = rtFly.querySelector('#rtGauge'), rtMini = rtFly.querySelector('#rtMini');

const rtQ = (root, sel) => root.querySelector(sel);
function rtTexts() {
  for (const root of [rtPlan, rtFly]) {
    root.querySelectorAll('[data-t]').forEach(el => { el.textContent = i18n(el.dataset.t); });
    root.querySelectorAll('[data-tt]').forEach(el => { el.title = i18n(el.dataset.tt); });
  }
}

// ---------------------------------------------------------------------------- panel controls
function rtBindPanel() {
  rtPlan.querySelectorAll('.rt-step').forEach(el => {
    const k = el.dataset.k, inp = el.querySelector('input');
    el.querySelectorAll('button').forEach(b => b.addEventListener('click', () => rtSet(k, clamp(params[k] + +b.dataset.d, 10, 1000))));
    inp.addEventListener('change', () => rtSet(k, clamp(Math.round(+inp.value || params[k]), 10, 1000)));
  });
  rtPlan.querySelectorAll('.rt-slide').forEach(el => {
    const k = el.dataset.k, [minus, plus] = el.querySelectorAll('.rt-pm'), range = el.querySelector('input[type=range]'), num = el.querySelector('.rt-num');
    const mn = +el.dataset.min, mx = +el.dataset.max, pm = +el.dataset.pm;
    range.min = mn; range.max = mx; range.step = el.dataset.step;
    num.min = mn; num.max = mx; num.step = el.dataset.step === 'any' ? '0.1' : el.dataset.step;
    range.addEventListener('input', () => rtSet(k, +range.value));
    num.addEventListener('change', () => rtSet(k, clamp(+num.value || 0, mn, mx)));
    minus.addEventListener('click', () => rtSet(k, clamp(Math.round((params[k] - pm) * 10) / 10, mn, mx)));
    plus.addEventListener('click', () => rtSet(k, clamp(Math.round((params[k] + pm) * 10) / 10, mn, mx)));
  });
  rtPlan.querySelectorAll('select[data-k]').forEach(el => el.addEventListener('change', () => rtSet(el.dataset.k, el.value)));
  rtPlan.querySelectorAll('input[type=checkbox][data-k]').forEach(el => el.addEventListener('change', () => rtSet(el.dataset.k, el.checked)));
  rtPlan.querySelectorAll('[data-a]').forEach(el => el.addEventListener('click', e => { e.stopPropagation(); rtAction(el.dataset.a); }));
}
function rtSet(k, v) { params[k] = v; rtBuild(); rtSave(); rtPanel(); }
const rtFmtDur = s => {
  if (!Number.isFinite(s)) return '–';
  const [H, M, S] = LANG === 'zh' ? [' 时 ', ' 分 ', ' 秒'] : [' h ', ' m ', ' s'];
  return s >= 3600 ? `${Math.floor(s / 3600)}${H}${Math.floor(s % 3600 / 60)}${M.trim()}` : `${Math.floor(s / 60)}${M}${Math.round(s % 60)}${S}`;
};
const rtFmtLen = m => m >= 1000 ? (m / 1000).toFixed(2) + ' km' : m.toFixed(0) + ' m';
const rtFmtArea = a => a >= 1e4 ? `${(a / 1e4).toFixed(2)} ha` : `${a.toFixed(0)} m²`;
function rtPanel() {
  rtPlan.querySelectorAll('.rt-step').forEach(el => { el.querySelector('input').value = params[el.dataset.k]; });
  rtPlan.querySelectorAll('.rt-slide').forEach(el => {
    const v = params[el.dataset.k];
    el.querySelector('input[type=range]').value = v;
    el.querySelector('.rt-num').value = (+v).toFixed(+el.dataset.dec);
  });
  rtPlan.querySelectorAll('select[data-k]').forEach(el => { el.value = params[el.dataset.k]; });
  rtPlan.querySelectorAll('input[type=checkbox][data-k]').forEach(el => { el.checked = !!params[el.dataset.k]; });
  const R = RT.route, fp = rtFootprint(), zh = LANG === 'zh';
  const set = (k, t) => { const el = rtPlan.querySelector(`[data-v="${k}"]`); if (el) el.textContent = t; };
  set('area', R ? rtFmtArea(R.area) : '–');
  set('dur', R ? rtFmtDur(R.duration) : '–');
  set('photos', R ? String(R.photos) : '–');
  set('len', R ? rtFmtLen(R.len + R.rth) : '–');
  set('spacing', R ? R.spacing.toFixed(1) + ' m' : '–');
  set('lines', R ? String(R.lines) : '–');
  set('gsd', zh ? `地面覆盖 ${fp.across.toFixed(0)} × ${fp.along.toFixed(0)} m（垂直向下）` : `Ground footprint ${fp.across.toFixed(0)} × ${fp.along.toFixed(0)} m (looking straight down)`);
  set('sideSub', zh ? `航线间距 ${fp.spacing.toFixed(1)} m` : `Line spacing ${fp.spacing.toFixed(1)} m`);
  set('frontSub', zh ? `拍照间隔 ${fp.interval.toFixed(1)} m（${(fp.interval / params.rtSpeed).toFixed(2)} s）` : `Photo interval ${fp.interval.toFixed(1)} m (${(fp.interval / params.rtSpeed).toFixed(2)} s)`);
  rtQ(rtPlan, '#rtGo').disabled = !R;
}
function rtAction(a) {
  if (a === 'back') rtPlanExit();
  else if (a === 'clear') { RT.verts = []; RT.sel = -1; rtBuild(); rtSave(); rtPanel(); }
  else if (a === 'del') { if (RT.sel >= 0) { RT.verts.splice(RT.sel, 1); RT.sel = -1; rtBuild(); rtSave(); rtPanel(); } }
  else if (a === 'rect') {
    const h = RT.map.alt * Math.tan(RT_MAP_FOV / 2 * D2R) * 0.55, w = h * 1.3, { cx, cz } = RT.map;
    RT.verts = [{ x: cx - w, z: cz - h }, { x: cx + w, z: cz - h }, { x: cx + w, z: cz + h }, { x: cx - w, z: cz + h }];
    RT.sel = -1; rtBuild(); rtSave(); rtPanel();
  }
  else if (a === 'center') rtFit();
  else if (a === 'start') { params.rtStart = (params.rtStart + 1) % 4; rtBuild(); rtSave(); rtPanel(); }
  else if (a === 'go') rtFlyStart();
}

// ---------------------------------------------------------------------------- view state save / restore
function rtSaveView() {
  RT.saved = {
    pos: camera.position.clone(), quat: camera.quaternion.clone(), fov: camera.fov, target: controls.target.clone(),
    follow: params.followGroup, sensors: params.sensors, source: params.sensorSource, cineCam: params.cineCam,
    cineMode: cineCam.mode, shake: params.cineShake, dof: params.cineDof, mb: params.cineMB,
    gui: gui.domElement.style.display, sgui: simpleGui.domElement.style.display,
  };
  params.followGroup = false; params.cineShake = false;
  params.cineCam = 0; cineCam.mode = 0; cineCam.t = 1;
  controls.enabled = false;
  gui.domElement.style.display = 'none'; simpleGui.domElement.style.display = 'none';
  uiSwitches.style.display = 'none';
}
function rtRestoreView() {
  const S = RT.saved;
  cameraDriver = null;
  camera.up.set(0, 1, 0);
  camera.position.copy(S.pos); camera.quaternion.copy(S.quat);
  camera.fov = S.fov; camera.updateProjectionMatrix();
  controls.target.copy(S.target); controls.enabled = true; controls.update();
  Object.assign(params, { followGroup: S.follow, sensorSource: S.source, cineCam: S.cineCam, cineShake: S.shake, cineDof: S.dof, cineMB: S.mb });
  cineCam.mode = S.cineMode;
  if (params.sensors !== S.sensors) { params.sensors = S.sensors; layoutSensorWindows(); lidarLayout(); }
  gui.domElement.style.display = S.gui; simpleGui.domElement.style.display = S.sgui;
  uiSwitches.style.display = '';
  document.getElementById('hud').style.display = '';
  RT.saved = null;
  gui.controllersRecursive().forEach(c => c.updateDisplay());
}

// ---------------------------------------------------------------------------- planner: map camera and interaction
function rtPlanEnter() {
  if (RT.mode !== 'off') return;
  rtSaveView();
  params.cineDof = false; params.cineMB = false;                // a sharp map
  if (params.sensors) { params.sensors = false; layoutSensorWindows(); lidarLayout(); }
  document.getElementById('hud').style.display = 'none';
  camera.fov = RT_MAP_FOV; camera.updateProjectionMatrix();
  RT.mode = 'plan';
  rtPlan.style.display = 'block';
  if (RT.verts.length >= 3) rtFit(); else Object.assign(RT.map, { cx: controls.target.x, cz: controls.target.z, alt: 500 });
  cameraDriver = rtMapDriver;
  rtBuild(); rtPanel(); rtTexts(); rtResize();
}
function rtPlanExit() {
  if (RT.mode !== 'plan') return;
  rtPlan.style.display = 'none';
  RT.mode = 'off';
  rtRestoreView();
}
function rtMapDriver() {
  const M = RT.map;
  camera.up.set(0, 0, -1);                                    // north up
  camera.position.set(M.cx, M.alt, M.cz);
  camera.lookAt(M.cx, 0, M.cz);
  camera.updateMatrixWorld();
}
function rtFit() {
  const V = RT.verts.length ? RT.verts : [{ x: controls.target.x, z: controls.target.z }];
  let x0 = Infinity, x1 = -Infinity, z0 = Infinity, z1 = -Infinity;
  for (const v of V) { x0 = Math.min(x0, v.x); x1 = Math.max(x1, v.x); z0 = Math.min(z0, v.z); z1 = Math.max(z1, v.z); }
  const mapW = Math.max(200, window.innerWidth - 340), aspect = mapW / window.innerHeight, t = Math.tan(RT_MAP_FOV / 2 * D2R);
  const half = Math.max((z1 - z0) / 2, (x1 - x0) / 2 / aspect, 60) * 1.35;
  // centre the area in the map part of the screen (left of the side panel)
  RT.map.alt = clamp(half / t, 40, 9000);
  const shift = 340 / 2 / window.innerHeight * 2 * RT.map.alt * t;
  Object.assign(RT.map, { cx: (x0 + x1) / 2 + shift, cz: (z0 + z1) / 2 });
}
const _rtV = new THREE.Vector3();
function rtToScreen(x, z) { _rtV.set(x, 0, z).project(camera); return [(_rtV.x + 1) / 2 * window.innerWidth, (1 - _rtV.y) / 2 * window.innerHeight]; }
function rtToSea(px, py) {
  _rtV.set(px / window.innerWidth * 2 - 1, 1 - py / window.innerHeight * 2, 0.5).unproject(camera).sub(camera.position);
  const t = -camera.position.y / _rtV.y;
  return { x: camera.position.x + _rtV.x * t, z: camera.position.z + _rtV.z * t };
}
function rtResize() {
  const dpr = window.devicePixelRatio || 1;
  rtCanvas.width = Math.round(window.innerWidth * dpr); rtCanvas.height = Math.round(window.innerHeight * dpr);
  rtCanvas.style.width = window.innerWidth + 'px'; rtCanvas.style.height = window.innerHeight + 'px';
}
window.addEventListener('resize', () => { if (RT.mode === 'plan') rtResize(); });
function rtHit(px, py) {
  const V = RT.verts;
  for (let i = V.length - 1; i >= 0; i--) { const [x, y] = rtToScreen(V[i].x, V[i].z); if (Math.hypot(x - px, y - py) < 13) return { vert: i }; }
  if (V.length >= 2) {
    const n = V.length >= 3 ? V.length : 1;
    for (let i = 0; i < n; i++) {
      const a = V[i], b = V[(i + 1) % V.length], [x, y] = rtToScreen((a.x + b.x) / 2, (a.z + b.z) / 2);
      if (Math.hypot(x - px, y - py) < 11) return { mid: i };
    }
  }
  return null;
}
// a new point goes into the polygon edge it is closest to, so the outline stays a simple polygon
function rtInsertAt(p) {
  const V = RT.verts;
  if (V.length < 3) { V.push(p); return V.length - 1; }
  let best = 0, bd = Infinity;
  for (let i = 0; i < V.length; i++) {
    const a = V[i], b = V[(i + 1) % V.length], ex = b.x - a.x, ez = b.z - a.z, l2 = ex * ex + ez * ez || 1;
    const t = clamp(((p.x - a.x) * ex + (p.z - a.z) * ez) / l2, 0, 1), d = Math.hypot(a.x + ex * t - p.x, a.z + ez * t - p.z);
    if (d < bd) { bd = d; best = i; }
  }
  V.splice(best + 1, 0, p);
  return best + 1;
}
rtCanvas.addEventListener('contextmenu', e => e.preventDefault());
rtCanvas.addEventListener('pointerdown', e => {
  if (RT.mode !== 'plan') return;
  try { rtCanvas.setPointerCapture(e.pointerId); } catch (err) { /* synthetic pointer */ }
  const h = e.button === 0 ? rtHit(e.offsetX, e.offsetY) : null;
  if (h && h.vert !== undefined) { RT.sel = h.vert; RT.drag = { type: 'vert', i: h.vert }; }
  else if (h && h.mid !== undefined) {
    const V = RT.verts, a = V[h.mid], b = V[(h.mid + 1) % V.length];
    V.splice(h.mid + 1, 0, { x: (a.x + b.x) / 2, z: (a.z + b.z) / 2 });
    RT.sel = h.mid + 1; RT.drag = { type: 'vert', i: h.mid + 1 };
    rtBuild(); rtPanel();
  } else RT.drag = { type: 'map', x: e.offsetX, y: e.offsetY, x0: e.offsetX, y0: e.offsetY, moved: false, button: e.button };
});
rtCanvas.addEventListener('pointermove', e => {
  const D = RT.drag;
  if (!D) { rtCanvas.style.cursor = rtHit(e.offsetX, e.offsetY) ? 'grab' : 'crosshair'; return; }
  if (D.type === 'vert') { RT.verts[D.i] = rtToSea(e.offsetX, e.offsetY); rtBuild(); rtPanel(); return; }
  if (!D.moved && Math.hypot(e.offsetX - D.x0, e.offsetY - D.y0) < 5) return;
  D.moved = true;
  const a = rtToSea(D.x, D.y), b = rtToSea(e.offsetX, e.offsetY);
  RT.map.cx += a.x - b.x; RT.map.cz += a.z - b.z;
  rtMapDriver();
  D.x = e.offsetX; D.y = e.offsetY;
  rtCanvas.style.cursor = 'grabbing';
});
rtCanvas.addEventListener('pointerup', e => {
  const D = RT.drag;
  RT.drag = null;
  rtCanvas.style.cursor = 'crosshair';
  if (!D) return;
  if (D.type === 'vert') { rtSave(); return; }
  if (!D.moved && D.button === 0) {                           // a click: add a boundary point
    RT.sel = rtInsertAt(rtToSea(e.offsetX, e.offsetY));
    rtBuild(); rtSave(); rtPanel();
  }
});
rtCanvas.addEventListener('wheel', e => {
  e.preventDefault();
  const before = rtToSea(e.offsetX, e.offsetY);
  RT.map.alt = clamp(RT.map.alt * Math.exp(e.deltaY * 0.0012), 40, 9000);
  rtMapDriver();
  const after = rtToSea(e.offsetX, e.offsetY);                // keep the point under the cursor in place
  RT.map.cx += before.x - after.x; RT.map.cz += before.z - after.z;
}, { passive: false });

// ---------------------------------------------------------------------------- planner drawing
const RT_GRAD = [[43, 92, 255], [54, 195, 255], [255, 212, 0], [255, 138, 0]];
function rtGrad(t) {
  const s = clamp(t, 0, 1) * 3, i = Math.min(2, Math.floor(s)), f = s - i, a = RT_GRAD[i], b = RT_GRAD[i + 1];
  return `rgb(${a.map((v, k) => Math.round(v + (b[k] - v) * f)).join(',')})`;
}
function rtLabel(g, text, x, y) {
  g.font = '600 12px "Segoe UI", system-ui, sans-serif';
  g.lineWidth = 3; g.strokeStyle = 'rgba(0,0,0,.75)'; g.strokeText(text, x, y);
  g.fillStyle = '#fff'; g.fillText(text, x, y);
}
function rtDrawPlan() {
  const g = rtCtx, dpr = rtCanvas.width / window.innerWidth;
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, window.innerWidth, window.innerHeight);
  const V = RT.verts, S = V.map(v => rtToScreen(v.x, v.z));
  g.textAlign = 'center'; g.textBaseline = 'middle';
  if (V.length >= 2) {
    g.beginPath(); S.forEach(([x, y], i) => (i ? g.lineTo(x, y) : g.moveTo(x, y)));
    if (V.length >= 3) { g.closePath(); g.fillStyle = 'rgba(64, 140, 255, 0.22)'; g.fill(); }
    g.strokeStyle = '#3fa9ff'; g.lineWidth = 2; g.stroke();
  }
  // margin outline
  if (V.length >= 3 && params.rtMargin) {
    const O = rtOffset(V, params.rtMargin).map(v => rtToScreen(v.x, v.z));
    g.beginPath(); O.forEach(([x, y], i) => (i ? g.lineTo(x, y) : g.moveTo(x, y))); g.closePath();
    g.setLineDash([6, 5]); g.strokeStyle = 'rgba(160, 210, 255, 0.8)'; g.lineWidth = 1.2; g.stroke(); g.setLineDash([]);
  }
  // route
  const R = RT.route;
  if (R) {
    const P = R.pts.map(p => rtToScreen(p.x, p.z)), n = P.length - 1;
    g.lineCap = 'round'; g.lineJoin = 'round';
    for (let i = 0; i < n; i++) {
      g.beginPath(); g.moveTo(P[i][0], P[i][1]); g.lineTo(P[i + 1][0], P[i + 1][1]);
      g.strokeStyle = rtGrad(i / Math.max(1, n - 1)); g.lineWidth = (i & 1) ? 2 : 3; g.stroke();
      if (!(i & 1)) {                                          // direction chevron on each flight line
        const mx = (P[i][0] + P[i + 1][0]) / 2, my = (P[i][1] + P[i + 1][1]) / 2, a = Math.atan2(P[i + 1][1] - P[i][1], P[i + 1][0] - P[i][0]);
        if (Math.hypot(P[i + 1][0] - P[i][0], P[i + 1][1] - P[i][1]) > 40) {
          g.beginPath(); g.moveTo(mx + Math.cos(a) * 6, my + Math.sin(a) * 6);
          g.lineTo(mx + Math.cos(a + 2.5) * 6, my + Math.sin(a + 2.5) * 6); g.lineTo(mx + Math.cos(a - 2.5) * 6, my + Math.sin(a - 2.5) * 6);
          g.closePath(); g.fillStyle = '#fff'; g.fill();
        }
      }
    }
    if (params.rtFinish === 'rth') {                           // return leg
      g.beginPath(); g.moveTo(P[n][0], P[n][1]); g.lineTo(P[0][0], P[0][1]);
      g.setLineDash([4, 6]); g.strokeStyle = 'rgba(255,255,255,.55)'; g.lineWidth = 1.5; g.stroke(); g.setLineDash([]);
    }
    g.beginPath(); g.arc(P[0][0], P[0][1], 9, 0, Math.PI * 2); g.fillStyle = '#21c45d'; g.fill();
    g.lineWidth = 2; g.strokeStyle = '#fff'; g.stroke();
    g.fillStyle = '#fff'; g.font = '700 11px "Segoe UI", system-ui, sans-serif'; g.fillText('S', P[0][0], P[0][1] + 0.5);
  }
  // edges: length labels and "+" handles
  if (V.length >= 2) {
    const n = V.length >= 3 ? V.length : 1;
    for (let i = 0; i < n; i++) {
      const a = V[i], b = V[(i + 1) % V.length], [x, y] = rtToScreen((a.x + b.x) / 2, (a.z + b.z) / 2);
      rtLabel(g, `${Math.hypot(b.x - a.x, b.z - a.z).toFixed(1)} m`, x, y - 16);
      g.beginPath(); g.arc(x, y, 7, 0, Math.PI * 2); g.fillStyle = 'rgba(255,255,255,.85)'; g.fill();
      g.strokeStyle = '#555'; g.lineWidth = 1.5;
      g.beginPath(); g.moveTo(x - 4, y); g.lineTo(x + 4, y); g.moveTo(x, y - 4); g.lineTo(x, y + 4); g.stroke();
    }
  }
  S.forEach(([x, y], i) => {
    const sel = i === RT.sel;
    g.beginPath(); g.arc(x, y, sel ? 9 : 7.5, 0, Math.PI * 2);
    g.fillStyle = sel ? '#2f8cff' : '#fff'; g.fill(); g.lineWidth = 2; g.strokeStyle = sel ? '#fff' : '#2f8cff'; g.stroke();
  });
  // people in the water (as on a SAR map) and a north arrow
  g.fillStyle = '#ff6b3d';
  for (const P of people) { const [x, y] = rtToScreen(P.pos.x, P.pos.z); g.beginPath(); g.arc(x, y, 3.5, 0, Math.PI * 2); g.fill(); }
  const nx = window.innerWidth - 340 - 34, ny = 84;
  g.beginPath(); g.moveTo(nx, ny - 14); g.lineTo(nx + 8, ny + 8); g.lineTo(nx, ny + 3); g.lineTo(nx - 8, ny + 8); g.closePath();
  g.fillStyle = '#fff'; g.fill(); rtLabel(g, 'N', nx, ny - 24);
  // scale bar
  const a = rtToSea(30, window.innerHeight - 70), b = rtToSea(130, window.innerHeight - 70), mPer100 = Math.hypot(b.x - a.x, b.z - a.z);
  const nice = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000].find(v => v >= mPer100 * 0.6) || 5000, px = nice / mPer100 * 100;
  const sx = window.innerWidth - 340 - 30 - px, sy = window.innerHeight - 24;
  g.strokeStyle = '#fff'; g.lineWidth = 2; g.beginPath(); g.moveTo(sx, sy - 5); g.lineTo(sx, sy); g.lineTo(sx + px, sy); g.lineTo(sx + px, sy - 5); g.stroke();
  rtLabel(g, nice >= 1000 ? `${nice / 1000} km` : `${nice} m`, sx + px / 2, sy - 11);
}

// ---------------------------------------------------------------------------- flight
function rtFlyStart() {
  if (!RT.route) return;
  const fromPlan = RT.mode === 'plan';
  if (fromPlan) rtPlan.style.display = 'none';
  else rtSaveView();
  const S = RT.saved;
  params.cineDof = S.dof; params.cineMB = S.mb;               // the flight uses the normal lens settings
  camera.fov = S.fov; camera.updateProjectionMatrix(); camera.up.set(0, 1, 0);
  params.sensorSource = 0;                                    // the payload looks where the gimbal (main view) looks
  if (!params.sensors) { params.sensors = true; layoutSensorWindows(); lidarLayout(); }
  document.getElementById('hud').style.display = '';
  const R = RT.route, pts = R.pts.map(p => ({ ...p }));
  if (params.rtFinish === 'rth') pts.push({ ...pts[0] });
  const cum = [0];
  for (let i = 1; i < pts.length; i++) cum.push(cum[i - 1] + Math.hypot(pts[i].x - pts[i - 1].x, pts[i].z - pts[i - 1].z));
  const d0 = { x: pts[1].x - pts[0].x, z: pts[1].z - pts[0].z };
  RT.fly = { s: 0, pts, cum, total: cum[cum.length - 1], routeLen: R.len, yaw: Math.atan2(-d0.x, -d0.z), pitch: params.rtPitch,
    paused: false, done: false, laps: 0, rec: false, drag: null, pos: { x: pts[0].x, z: pts[0].z }, seg: 0 };
  RT.mode = 'fly';
  rtFly.style.display = 'block';
  rtQ(rtFly, '#rtDone').style.display = 'none';
  cameraDriver = rtFlyDriver;
  rtTexts();
  if (params.rtRecord && !REC.on) { RT.fly.rec = true; recStart(); }
}
function rtFlyExit() {
  if (RT.mode !== 'fly') return;
  if (RT.fly.rec && REC.on) recStop();
  rtFly.style.display = 'none';
  RT.mode = 'off'; RT.fly = null;
  rtRestoreView();
}
function rtFlyDriver(dt, dtReal) {
  const F = RT.fly, P = F.pts;
  if (!F.paused && !F.done) {
    F.s += params.rtSpeed * dt;
    if (F.s >= F.total) {
      if (params.rtFinish === 'repeat') { F.s -= F.total * Math.floor(F.s / F.total); F.laps++; }
      else {
        F.s = F.total; F.done = true;
        if (F.rec && REC.on) recStop();
        const d = rtQ(rtFly, '#rtDone');
        d.style.display = 'block';
        rtQ(d, '[data-v="doneT"]').textContent = i18n(params.rtFinish === 'rth' ? 'Mission complete: back at the start point, hovering' : 'Mission complete: hovering at the last waypoint');
      }
    }
  }
  // position on the polyline and the direction of travel
  let i = F.seg;
  while (i < P.length - 2 && F.cum[i + 1] <= F.s) i++;
  while (i > 0 && F.cum[i] > F.s) i--;
  F.seg = i;
  const a = P[i], b = P[i + 1], segL = F.cum[i + 1] - F.cum[i] || 1, t = clamp((F.s - F.cum[i]) / segL, 0, 1);
  F.pos.x = a.x + (b.x - a.x) * t; F.pos.z = a.z + (b.z - a.z) * t;
  // heading follows the route, limited to 90 deg/s (mapping flights keep the aircraft nose along the line)
  if (!F.done) {
    const want = Math.atan2(-(b.x - a.x), -(b.z - a.z));
    let d = want - F.yaw;
    d = Math.atan2(Math.sin(d), Math.cos(d));
    const maxStep = Math.PI / 2 * Math.max(dt, 1e-4);
    F.yaw += clamp(d, -maxStep, maxStep);
  }
  camera.position.set(F.pos.x, params.rtAlt, F.pos.z);
  camera.quaternion.setFromEuler(_rtE.set(F.pitch * D2R, F.yaw, 0, 'YXZ'));
  camera.updateMatrixWorld();
}
const _rtE = new THREE.Euler();
// gimbal tilt: vertical drag only (the aircraft heading sets the pan)
renderer.domElement.addEventListener('pointerdown', e => {
  if (RT.mode !== 'fly') return;
  RT.fly.drag = { y: e.clientY };
  try { renderer.domElement.setPointerCapture(e.pointerId); } catch (err) { /* synthetic pointer */ }
});
renderer.domElement.addEventListener('pointermove', e => {
  if (RT.mode !== 'fly' || !RT.fly.drag) return;
  RT.fly.pitch = clamp(RT.fly.pitch - (e.clientY - RT.fly.drag.y) * 0.25, -90, 30);
  RT.fly.drag.y = e.clientY;
});
renderer.domElement.addEventListener('pointerup', () => { if (RT.fly) RT.fly.drag = null; });
window.addEventListener('keydown', e => {
  if (e.target && /INPUT|SELECT|TEXTAREA/.test(e.target.tagName)) return;
  if (e.key === 'Escape') {
    if (RT.mode === 'fly') { rtFlyExit(); e.preventDefault(); }
    else if (RT.mode === 'plan') { rtPlanExit(); e.preventDefault(); }
  } else if (e.key === ' ' && RT.mode === 'fly') { RT.fly.paused = !RT.fly.paused; e.preventDefault(); }
  else if ((e.key === 'Delete' || e.key === 'Backspace') && RT.mode === 'plan') rtAction('del');
});

// ---------------------------------------------------------------------------- flight HUD, gimbal gauge, minimap
function rtDrawFly() {
  const F = RT.fly, zh = LANG === 'zh', remaining = Math.max(0, F.total - F.s);
  const prog = params.rtFinish === 'repeat' ? (F.s % F.total) / F.total : F.s / F.total;
  const st = rtQ(rtFly, '.st');
  st.textContent = F.done ? i18n('COMPLETE') : F.paused ? i18n('PAUSED') : i18n('MAPPING') + (F.laps ? ` ×${F.laps + 1}` : '');
  st.className = 'st' + (F.done ? ' d' : F.paused ? ' p' : '');
  rtQ(rtFly, '.prog i').style.width = (prog * 100).toFixed(1) + '%';
  const set = (k, t) => { rtQ(rtFly, `[data-v="${k}"]`).textContent = t; };
  set('pct', (prog * 100).toFixed(0) + '%');
  set('alt', params.rtAlt.toFixed(0) + ' m');
  set('spd', (F.done || F.paused ? 0 : params.rtSpeed).toFixed(1) + ' m/s');
  set('dist', rtFmtLen(Math.hypot(F.pos.x - F.pts[0].x, F.pos.z - F.pts[0].z)));
  set('eta', params.rtFinish === 'repeat' ? '∞' : rtFmtDur(remaining / params.rtSpeed));
  set('gim', F.pitch.toFixed(0) + '°');
  // gimbal pitch gauge: +30 at the top, -90 at the bottom
  const g = rtGauge.getContext('2d'), H = rtGauge.height, y = p => 12 + (30 - p) / 120 * (H - 24);
  g.clearRect(0, 0, 40, H);
  g.fillStyle = 'rgba(0,0,0,.45)'; g.fillRect(14, 8, 12, H - 16);
  g.strokeStyle = 'rgba(255,255,255,.8)'; g.lineWidth = 1;
  for (const p of [30, 0, -30, -60, -90]) { g.beginPath(); g.moveTo(10, y(p)); g.lineTo(30, y(p)); g.stroke(); }
  g.fillStyle = '#ffd60a'; g.beginPath(); g.moveTo(8, y(F.pitch)); g.lineTo(32, y(F.pitch) - 5); g.lineTo(32, y(F.pitch) + 5); g.closePath(); g.fill();
  // minimap: north up, fitted to the area
  const m = rtMini.getContext('2d'), W = rtMini.width, V = RT.verts;
  let x0 = Infinity, x1 = -Infinity, z0 = Infinity, z1 = -Infinity;
  for (const p of [...V, ...F.pts]) { x0 = Math.min(x0, p.x); x1 = Math.max(x1, p.x); z0 = Math.min(z0, p.z); z1 = Math.max(z1, p.z); }
  const sc = (W - 30) / Math.max(x1 - x0, z1 - z0, 1), cx = (x0 + x1) / 2, cz = (z0 + z1) / 2;
  const M = p => [W / 2 + (p.x - cx) * sc, W / 2 + (p.z - cz) * sc];
  m.clearRect(0, 0, W, W);
  if (V.length >= 3) {
    m.beginPath(); V.forEach((p, i) => { const [a, b] = M(p); i ? m.lineTo(a, b) : m.moveTo(a, b); }); m.closePath();
    m.fillStyle = 'rgba(64,140,255,.2)'; m.fill(); m.strokeStyle = '#3fa9ff'; m.lineWidth = 1.5; m.stroke();
  }
  m.lineWidth = 2; m.strokeStyle = 'rgba(255,255,255,.35)';
  m.beginPath(); F.pts.forEach((p, i) => { const [a, b] = M(p); i ? m.lineTo(a, b) : m.moveTo(a, b); }); m.stroke();
  m.strokeStyle = '#34d27b'; m.beginPath();
  for (let i = 0; i <= F.seg; i++) { const [a, b] = M(F.pts[i]); i ? m.lineTo(a, b) : m.moveTo(a, b); }
  { const [a, b] = M(F.pos); m.lineTo(a, b); } m.stroke();
  m.fillStyle = '#ff6b3d';
  for (const P of people) { const [a, b] = M({ x: P.pos.x, z: P.pos.z }); m.beginPath(); m.arc(a, b, 2.5, 0, Math.PI * 2); m.fill(); }
  { const [a, b] = M(F.pts[0]); m.fillStyle = '#21c45d'; m.beginPath(); m.arc(a, b, 5, 0, Math.PI * 2); m.fill(); }
  const [ax, ay] = M(F.pos), h = -F.yaw;                     // screen angle of the nose (north up)
  m.save(); m.translate(ax, ay); m.rotate(h);
  m.fillStyle = '#fff'; m.strokeStyle = '#000'; m.lineWidth = 1;
  m.beginPath(); m.moveTo(0, -9); m.lineTo(6, 7); m.lineTo(0, 3); m.lineTo(-6, 7); m.closePath(); m.fill(); m.stroke();
  m.restore();
  m.fillStyle = '#fff'; m.font = '10px Consolas, monospace'; m.textAlign = 'left'; m.fillText(zh ? '北 ↑' : 'N ↑', 8, 14);
}

HOOKS.afterRender.push(() => {
  if (RT.mode === 'plan') rtDrawPlan();
  else if (RT.mode === 'fly') rtDrawFly();
});

// ---------------------------------------------------------------------------- entry button and language
const rtEntry = document.createElement('div');
rtEntry.className = 'seg';
rtEntry.innerHTML = '<button type="button"></button>';
uiSwitches.insertBefore(rtEntry, uiSwitches.firstChild);
rtEntry.querySelector('button').addEventListener('click', rtPlanEnter);
function rtEntryText() { rtEntry.querySelector('button').textContent = '✈ ' + i18n('Flight route'); }
rtBindPanel();
rtBuild();
rtEntryText();
LANG_HOOKS.push(() => { rtEntryText(); rtTexts(); if (RT.mode === 'plan') rtPanel(); });
HOOKS.ready.push(() => {
  window.sim.route = { RT, enter: rtPlanEnter, exitPlan: rtPlanExit, fly: rtFlyStart, exitFly: rtFlyExit, build: rtBuild, toScreen: rtToScreen, toSea: rtToSea };
});
