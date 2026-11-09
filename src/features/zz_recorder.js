// ============================================================================
// Data recording: the RGB camera, the LWIR camera and the LiDAR, saved frame by frame to a local folder
// ============================================================================
// Storage: Chrome / Edge write straight into a folder the user picks (File System Access API; the choice is kept
// in IndexedDB, and the browser asks for permission again on the next visit). Without that API (Firefox, Safari)
// the frames are kept in memory and saved as one .zip when recording stops.
// One session = one sub-folder <name>_<date>_<time>/ holding:
//   rgb/000000.jpg|png        EO camera, tone-mapped as in its window, at the chosen resolution
//   ir/000000.png             LWIR image as the camera shows it (AGC, palette, noise), 8 bit
//   ir_raw/000000.pgm         16-bit radiometric image: apparent temperature in centi-kelvin (T[K] = value / 100)
//   lidar/000000.pcd|las|laz  point cloud: binary PCD (x y z intensity label return), LAS 1.2 or LAZ (point format 1,
//                             see sensor_lidar_las.js); the pulses fired since the previous frame, a spinning scanner gives
//                             its latest full turn
//   labels/000000.txt         YOLO boxes of the people (class cx cy w h, normalised); the same for RGB and IR,
//                             which share one co-registered pose
//   meta/000000.json          time, camera and LiDAR poses, intrinsics, environment, per-person boxes in pixels
//   session.json              settings, sensor descriptions, coordinate conventions, class names, frame count
// Frames follow simulation time: at R Hz a frame is taken every 1/R s of simulated time. "Never drop frames" caps
// the simulation step at 1/R s and holds the simulation while the disk catches up, so every frame is written;
// otherwise frames the machine cannot keep up with are counted as dropped.
Object.assign(params, {
  recFps: 10, recRgbRes: 1, recRgbFmt: 'jpg', recIrRes: 1, recRgb: true, recIr: true, recIrRaw: true, recLidar: true,
  recLabels: true, recLidarFmt: 'pcd', recNoDrop: true, recMaxSec: 0, recName: 'mob',
});
const REC_RGB_RES = [[640, 512], [1280, 1024], [1920, 1536], [2560, 2048]];
const REC_IR_RES = [[320, 256], [640, 512], [1280, 1024]];
const REC_QUEUE_MAX = 12;                 // frames being encoded / written at once
const REC_ZIP_MAX = 1.5e9;                // bytes held in memory without a folder
const REC = {
  on: false, starting: false, dir: null, dirName: '', session: null, sessName: '', zip: null, subdirs: new Map(),
  frame: 0, t0Sim: 0, t0Wall: 0, nextT: 0, bytes: 0, queue: 0, dropped: 0, errors: 0, lastHead: 0, pending: new Set(), msg: '',
};
const recFS = typeof window.showDirectoryPicker === 'function';

// ---------------------------------------------------------------------------- capture targets
const recRgbHdr = new THREE.WebGLRenderTarget(8, 8, { type: THREE.HalfFloatType, samples: 4 });
const recRefr = new THREE.WebGLRenderTarget(8, 8, { type: THREE.HalfFloatType, depthTexture: new THREE.DepthTexture(8, 8) });
const recRgb8 = new THREE.WebGLRenderTarget(8, 8, { depthBuffer: false });
const recIrHdr = new THREE.WebGLRenderTarget(8, 8, { type: THREE.HalfFloatType, samples: 4 });
const recIr8 = new THREE.WebGLRenderTarget(8, 8, { depthBuffer: false });
const recIrK = new THREE.WebGLRenderTarget(8, 8, { type: THREE.FloatType, depthBuffer: false, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter });
const recRgbOut = rgbOutMat.clone();
recRgbOut.uniforms.tDiffuse.value = recRgbHdr.texture;
const recIrOut = irOutMat.clone();
recIrOut.uniforms.uRadiance.value = recIrHdr.texture;
recIrOut.uniforms.uSmall.value = agcRT.texture;
const recIrKMat = new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3, depthTest: false, depthWrite: false, vertexShader: THERMAL_FULLSCREEN_VERT,
  uniforms: { uRadiance: { value: recIrHdr.texture }, uRes: { value: new THREE.Vector2() } },
  fragmentShader: THERMAL_GLSL + /* glsl */`
uniform sampler2D uRadiance;
uniform vec2 uRes;
layout(location = 0) out vec4 o;
void main() { o = vec4(bandTemp(texture(uRadiance, gl_FragCoord.xy / uRes).r), 0.0, 0.0, 1.0); }`,
});
const recSize = (rt, w, h) => { if (rt.width !== w || rt.height !== h) rt.setSize(w, h); };

function recRenderRGB(w, h) {
  recSize(recRgbHdr, w, h); recSize(recRefr, w, h); recSize(recRgb8, w, h);
  renderView(sensorCam, recRgbHdr, recRefr);
  const u = recRgbOut.uniforms;
  u.uExposure.value = compositePass.uniforms.uExposure.value; u.uFrame.value = REC.frame % 64; u.uAspect.value = w / h;
  runPass(recRgbOut, recRgb8);
  const px = new Uint8Array(w * h * 4);
  renderer.readRenderTargetPixels(recRgb8, 0, 0, w, h, px);
  return px;
}
function recRenderIR(w, h, wantImg, wantRaw) {
  recSize(recIrHdr, w, h);
  THERM.uThermal.value = 1;
  people.forEach(P => { P.glow.visible = false; P.probeGroup.visible = false; });
  renderView(sensorCam, recIrHdr, null);
  THERM.uThermal.value = 0;
  people.forEach(P => { P.glow.visible = true; P.probeGroup.visible = params.showProbes; });
  const out = {};
  if (wantImg) {
    recSize(recIr8, w, h);
    const u = recIrOut.uniforms, s = irSensorU;
    for (const k of ['uLow', 'uHigh', 'uCurveMix', 'uNETD', 'uFPN', 'uFFC', 'uTref', 'uPalette', 'uFrame']) u[k].value = s[k].value;
    u.uCurve.value.set(s.uCurve.value);
    u.uViewport.value.set(0, 0, w, h); u.uSensorRes.value.set(w, h); u.uFill.value = 1; u.uOSD.value = 0;
    runPass(recIrOut, recIr8);
    out.img = new Uint8Array(w * h * 4);
    renderer.readRenderTargetPixels(recIr8, 0, 0, w, h, out.img);
  }
  if (wantRaw) {
    recSize(recIrK, w, h);
    recIrKMat.uniforms.uRes.value.set(w, h);
    runPass(recIrKMat, recIrK);
    out.raw = new Float32Array(w * h * 4);
    renderer.readRenderTargetPixels(recIrK, 0, 0, w, h, out.raw);
  }
  return out;
}

// ---------------------------------------------------------------------------- encoders
async function recEncodeImage(px, w, h, type, quality) {
  const img = new ImageData(w, h), row = w * 4;
  for (let y = 0; y < h; y++) img.data.set(px.subarray((h - 1 - y) * row, (h - y) * row), y * row);   // GL rows are bottom-up
  if (typeof OffscreenCanvas === 'function') {
    const cv = new OffscreenCanvas(w, h);
    cv.getContext('2d').putImageData(img, 0, 0);
    return cv.convertToBlob({ type, quality });
  }
  const cv = document.createElement('canvas');
  cv.width = w; cv.height = h;
  cv.getContext('2d').putImageData(img, 0, 0);
  return new Promise(res => cv.toBlob(res, type, quality));
}
// 16-bit binary PGM (big-endian, top row first): centi-kelvin
function recPGM16(raw, w, h) {
  const head = new TextEncoder().encode(`P5\n# apparent temperature, centi-kelvin (T[K] = value / 100)\n${w} ${h}\n65535\n`);
  const buf = new Uint8Array(head.length + w * h * 2);
  buf.set(head, 0);
  const dv = new DataView(buf.buffer, head.length);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const v = Math.max(0, Math.min(65535, Math.round(raw[((h - 1 - y) * w + x) * 4] * 100)));
      dv.setUint16((y * w + x) * 2, v, false);
    }
  }
  return buf;
}
// ---------------------------------------------------------------------------- LiDAR frame: pulses since the last frame
function recLidarFrame() {
  const M = L.model, world = params.lidarFrame === 1;
  if (M.kind === 0) return lidarGather(0, 0, world, simTime);           // spinning: the latest full turn
  const s1 = L.head, cap = Math.min(L.N - 1, Math.round(M.rate / params.recFps));
  let n = (s1 - REC.lastHead + L.N) % L.N;
  if (REC.frame === 0 || n === 0) n = cap;                                // first frame: one capture interval
  REC.lastHead = s1;
  return lidarGather((s1 - n + L.N) % L.N, n, world, simTime);
}

// ---------------------------------------------------------------------------- ground-truth boxes of the people
// The part above the local water surface is what both cameras see: spheres around the skeleton (radius by body
// part), sampled at their centre and four rim points, kept where they are above the water plane, projected.
const REC_BONE_R = [[/HeadTop_End$/, 0.04], [/Head$/, 0.11], [/Neck$/, 0.07], [/Spine2$/, 0.18], [/Spine1?$/, 0.16],
  [/Hips$/, 0.15], [/Shoulder$/, 0.07], [/ForeArm$/, 0.05], [/Arm$/, 0.06], [/Hand$/, 0.05], [/UpLeg$/, 0.08],
  [/Leg$/, 0.06], [/Foot$/, 0.05], [/ToeBase$/, 0.04]];
const _recB = new THREE.Vector3(), _recP = new THREE.Vector3(), _recC = new THREE.Vector3(), _recRight = new THREE.Vector3(), _recUp = new THREE.Vector3(0, 1, 0);
function recBoneRadius(b) {
  if (b._recR === undefined) {
    const n = b.name.replace(/^mixamorig:?/, '');
    b._recR = (REC_BONE_R.find(([re]) => re.test(n)) || [0, 0])[1];
  }
  return b._recR;
}
function recPeopleBoxes(cam) {
  cam.updateMatrixWorld();
  _recRight.setFromMatrixColumn(cam.matrixWorld, 0);
  const out = [];
  people.forEach((P, id) => {
    const pl = P.wetU.plane.value;
    let x0 = 1, y0 = 1, x1 = 0, y1 = 0, nVis = 0, nAll = 0;
    for (const b of P.rig.bones) {
      const r = recBoneRadius(b);
      if (!r) continue;
      b.getWorldPosition(_recB);
      for (const [dx, dy] of [[0, 0], [1, 0], [-1, 0], [0, 1], [0, -1]]) {
        _recP.copy(_recB).addScaledVector(_recRight, dx * r).addScaledVector(_recUp, dy * r);
        nAll++;
        if (pl.x * _recP.x + pl.y * _recP.y + pl.z * _recP.z + pl.w < 0) continue;                 // under water
        _recC.copy(_recP).applyMatrix4(cam.matrixWorldInverse);
        if (_recC.z > -0.05) continue;                                                     // behind the camera
        _recC.applyMatrix4(cam.projectionMatrix);
        const u = (_recC.x + 1) / 2, v = (1 - _recC.y) / 2;
        x0 = Math.min(x0, u); x1 = Math.max(x1, u); y0 = Math.min(y0, v); y1 = Math.max(y1, v);
        nVis++;
      }
    }
    if (nVis < 3) return;
    const c = [Math.max(0, x0), Math.max(0, y0), Math.min(1, x1), Math.min(1, y1)];
    if (c[2] <= c[0] || c[3] <= c[1]) return;
    out.push({ id, class: 0, name: 'person', box: c, truncated: c[0] !== x0 || c[1] !== y0 || c[2] !== x1 || c[3] !== y1,
      visible_fraction: +(nVis / nAll).toFixed(3), distance_m: +P.pos.distanceTo(cam.position).toFixed(2),
      position: P.pos.toArray().map(v => +v.toFixed(3)) });
  });
  return out;
}

// ---------------------------------------------------------------------------- storage
function recIdb(op, val) {
  return new Promise(res => {
    try {
      const rq = indexedDB.open('oceanMob', 1);
      rq.onupgradeneeded = () => rq.result.createObjectStore('kv');
      rq.onerror = () => res(null);
      rq.onsuccess = () => {
        try {
          const tx = rq.result.transaction('kv', op === 'get' ? 'readonly' : 'readwrite'), st = tx.objectStore('kv');
          const q = op === 'get' ? st.get('recDir') : st.put(val, 'recDir');
          q.onsuccess = () => res(q.result ?? null); q.onerror = () => res(null);
        } catch (e) { res(null); }
      };
    } catch (e) { res(null); }
  });
}
async function recPickFolder() {
  if (!recFS) { REC.msg = i18n('This browser cannot write to a folder: recordings are saved as a .zip download.'); recInfo(); return; }
  try {
    const h = await window.showDirectoryPicker({ id: 'oceanMobData', mode: 'readwrite' });
    recSetFolder(h);
    recIdb('put', h);
  } catch (e) { /* cancelled */ }
}
function recSetFolder(h) { REC.dir = h; REC.dirName = h ? h.name : ''; REC.msg = ''; recInfo(); }
recIdb('get').then(h => { if (h && !REC.dir) recSetFolder(h); });

async function recWrite(path, data) {
  const size = data.size ?? data.byteLength ?? data.length;
  if (REC.session) {
    const i = path.lastIndexOf('/');
    let d = REC.session;
    if (i > 0) {
      const sub = path.slice(0, i);
      d = REC.subdirs.get(sub);
      if (!d) { d = await REC.session.getDirectoryHandle(sub, { create: true }); REC.subdirs.set(sub, d); }
    }
    const f = await d.getFileHandle(path.slice(i + 1), { create: true });
    const w = await f.createWritable();
    await w.write(data);
    await w.close();
  } else {
    const bytes = data instanceof Blob ? new Uint8Array(await data.arrayBuffer()) : typeof data === 'string' ? new TextEncoder().encode(data) : data;
    const name = REC.sessName + '/' + path, k = REC.zip.findIndex(e => e.name === name);   // rewritten files (session.json) replace
    if (k >= 0) REC.zip[k].data = bytes; else REC.zip.push({ name, data: bytes });
    if (REC.bytes > REC_ZIP_MAX && REC.on) recStop(i18n('Memory limit reached (1.5 GB): choose a folder to record longer.'));
  }
  REC.bytes += size;
}
function recTask(p) {
  REC.queue++;
  const t = p.catch(e => {
    console.error('recording:', e);
    if (++REC.errors >= 3 && REC.on) recStop(i18n('Writing failed') + ': ' + (e && e.message || e));
  }).finally(() => { REC.queue--; REC.pending.delete(t); });
  REC.pending.add(t);
}

// minimal .zip (stored, no compression) for browsers without folder access
const REC_CRC = (() => { const t = new Uint32Array(256); for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; t[n] = c >>> 0; } return t; })();
function recCrc(u8) { let c = 0xffffffff; for (let i = 0; i < u8.length; i++) c = REC_CRC[(c ^ u8[i]) & 255] ^ (c >>> 8); return (c ^ 0xffffffff) >>> 0; }
function recZip(entries) {
  const enc = new TextEncoder(), parts = [], central = [], d = new Date();
  const time = (d.getHours() << 11) | (d.getMinutes() << 5) | (d.getSeconds() >> 1);
  const date = ((d.getFullYear() - 1980) << 9) | ((d.getMonth() + 1) << 5) | d.getDate();
  let off = 0, cdSize = 0;
  for (const e of entries) {
    const nm = enc.encode(e.name), crc = recCrc(e.data), sz = e.data.length;
    const lh = new DataView(new ArrayBuffer(30));
    lh.setUint32(0, 0x04034b50, true); lh.setUint16(4, 20, true); lh.setUint16(6, 0x0800, true); lh.setUint16(8, 0, true);
    lh.setUint16(10, time, true); lh.setUint16(12, date, true); lh.setUint32(14, crc, true);
    lh.setUint32(18, sz, true); lh.setUint32(22, sz, true); lh.setUint16(26, nm.length, true); lh.setUint16(28, 0, true);
    parts.push(lh.buffer, nm, e.data);
    const ch = new DataView(new ArrayBuffer(46));
    ch.setUint32(0, 0x02014b50, true); ch.setUint16(4, 20, true); ch.setUint16(6, 20, true); ch.setUint16(8, 0x0800, true);
    ch.setUint16(10, 0, true); ch.setUint16(12, time, true); ch.setUint16(14, date, true); ch.setUint32(16, crc, true);
    ch.setUint32(20, sz, true); ch.setUint32(24, sz, true); ch.setUint16(28, nm.length, true); ch.setUint32(42, off, true);
    central.push(ch.buffer, nm);
    off += 30 + nm.length + sz; cdSize += 46 + nm.length;
  }
  const end = new DataView(new ArrayBuffer(22));
  end.setUint32(0, 0x06054b50, true); end.setUint16(8, entries.length, true); end.setUint16(10, entries.length, true);
  end.setUint32(12, cdSize, true); end.setUint32(16, off, true);
  return new Blob([...parts, ...central, end.buffer], { type: 'application/zip' });
}

// ---------------------------------------------------------------------------- session description
function recIntrinsics(w, h) {
  const fy = h / 2 / Math.tan(sensorCam.fov * D2R / 2);
  return { width: w, height: h, fx: +fy.toFixed(3), fy: +fy.toFixed(3), cx: w / 2, cy: h / 2,
    vfov_deg: +sensorCam.fov.toFixed(3), hfov_deg: +(2 * Math.atan(w / 2 / fy) * R2D).toFixed(3) };
}
function recSessionJson(final) {
  const [rw, rh] = REC_RGB_RES[params.recRgbRes], [iw, ih] = REC_IR_RES[params.recIrRes], M = L.model;
  return JSON.stringify({
    software: 'Ocean MOB simulator', created: new Date(REC.t0Wall).toISOString(), finished: final ? new Date().toISOString() : null,
    frames: REC.frame, dropped_frames: REC.dropped, capture_rate_hz: params.recFps, duration_sim_s: +(simTime - REC.t0Sim).toFixed(3),
    content: { rgb: params.recRgb, ir: params.recIr, ir_raw: params.recIrRaw, lidar: params.recLidar, labels: params.recLabels },
    sensors: {
      rgb: { ...recIntrinsics(rw, rh), format: params.recRgbFmt, model: 'pinhole, no distortion; principal point at the image centre' },
      ir: { ...recIntrinsics(iw, ih), band: '8-14 um LWIR microbolometer', palette: PALETTES[params.palette],
        raw: '16-bit PGM, value = apparent temperature in centi-kelvin (T[K] = value / 100), atmosphere included, no sensor noise' },
      lidar: { model: M.name, wavelength_nm: params.lidarBand, returns_per_pulse: L.R, range_10pct_m: params.lidarR10, max_range_m: params.lidarRmax,
        frame: params.lidarFrame === 1 ? 'world: x east, y north, z up (m)' : 'sensor: x forward, y left, z up (m, ROS REP-103)',
        format: params.recLidarFmt,
        fields: params.recLidarFmt === 'pcd'
          ? 'binary PCD: x y z intensity(apparent reflectance, 1 = white card, >1 retroreflector) label return(1 = first)'
          : 'LAS 1.2 point format 1' + (params.recLidarFmt === 'laz' ? ', LASzip compressed' : '') +
            ': x y z (0.001 m), intensity = reflectance x 32768 (clamped at 65535), return number / number of returns,' +
            ' classification (9 water, 7 noise, 1 object, 19 person, 20 retroreflector), user data = label below,' +
            ' point source id = frame, GPS time = simulation time (s), scan angle = beam angle from the forward axis (deg)',
        labels: { 1: 'water', 2: 'foam', 3: 'person', 4: 'retroreflector', 5: 'object', 6: 'clutter/noise', 7: 'blooming' } },
    },
    labels: { format: 'YOLO: class cx cy w h, normalised to the image; valid for RGB and IR (one co-registered pose)', classes: ['person'],
      box: 'part of the person above the local water surface; no occlusion test against waves or other people' },
    conventions: {
      world: 'three.js world: x east, y up, z south (m); sea level y = 0',
      poses: 'position + quaternion [x, y, z, w] of the sensor in the world; the cameras look along their local -z with +y up',
      lidar_mount: M.kind >= 1 ? 'boresighted with the cameras (same pose)' : 'spinning scanner on the gimbal, yaw-aligned with the cameras',
    },
    layout: { 'rgb/': 'EO images', 'ir/': 'LWIR display images', 'ir_raw/': 'LWIR radiometric 16-bit PGM', 'lidar/': 'point cloud per frame (PCD, LAS or LAZ)',
      'labels/': 'YOLO txt per frame', 'meta/': 'per-frame JSON: time, poses, intrinsics, environment, boxes in pixels' },
  }, null, 2);
}
function recFrameMeta(f, boxes, nPts) {
  const [rw, rh] = REC_RGB_RES[params.recRgbRes], [iw, ih] = REC_IR_RES[params.recIrRes];
  const px = (b, w, h) => [b[0] * w, b[1] * h, b[2] * w, b[3] * h].map(v => +v.toFixed(1));
  return JSON.stringify({
    frame: f, sim_time_s: +simTime.toFixed(4), t_rel_s: +(simTime - REC.t0Sim).toFixed(4), wall_time: new Date().toISOString(),
    camera: { position: sensorCam.position.toArray().map(v => +v.toFixed(4)), quaternion: sensorCam.quaternion.toArray().map(v => +v.toFixed(6)),
      vfov_deg: +sensorCam.fov.toFixed(3) },
    lidar: { position: L.pos.toArray().map(v => +v.toFixed(4)), quaternion: L.q.toArray().map(v => +v.toFixed(6)), points: nPts },
    environment: {
      sea_state: params.seaState, Hs_m: +sea.Hs.toFixed(3), wind_sea_Hs_m: +(sea.HsW ?? 0).toFixed(3), Tp_s: +sea.Tp.toFixed(2),
      wind_speed_ms: params.windSpeed, wind_from_deg: params.windFrom, weather: params.weather, time_of_day_h: +params.timeOfDay.toFixed(3),
      sun_elevation_deg: +sunElDeg.toFixed(2), air_temp_c: params.airTempC, water_temp_c: params.waterTempC, humidity: params.humidity,
      rain: +(params.rain || 0).toFixed(2), sea_fog_visibility_km: +(params.seaFogKm || 0).toFixed(2),
    },
    objects: boxes.map(b => ({ id: b.id, class: b.name, bbox_rgb_xyxy: px(b.box, rw, rh), bbox_ir_xyxy: px(b.box, iw, ih),
      truncated: b.truncated, visible_fraction: b.visible_fraction, distance_m: b.distance_m, position: b.position })),
  });
}

// ---------------------------------------------------------------------------- capture one frame
function recCapture() {
  const f = REC.frame, name = String(f).padStart(6, '0');
  if (params.recRgb) {
    const [w, h] = REC_RGB_RES[params.recRgbRes], px = recRenderRGB(w, h);
    const type = params.recRgbFmt === 'png' ? 'image/png' : 'image/jpeg';
    recTask(recEncodeImage(px, w, h, type, 0.92).then(b => recWrite(`rgb/${name}.${params.recRgbFmt}`, b)));
  }
  if (params.recIr || params.recIrRaw) {
    const [w, h] = REC_IR_RES[params.recIrRes], ir = recRenderIR(w, h, params.recIr, params.recIrRaw);
    if (ir.img) recTask(recEncodeImage(ir.img, w, h, 'image/png').then(b => recWrite(`ir/${name}.png`, b)));
    if (ir.raw) recTask(Promise.resolve().then(() => recWrite(`ir_raw/${name}.pgm`, recPGM16(ir.raw, w, h))));
  }
  renderer.setRenderTarget(null);
  let nPts = 0;
  if (params.recLidar && params.lidar) {
    const lf = recLidarFrame(), fmt = params.recLidarFmt;
    nPts = lf.n;
    recTask(lidarEncode(lf, fmt, `${L.model.name}, frame ${f}`, f).then(b => recWrite(`lidar/${name}.${fmt}`, b)));
  }
  const boxes = recPeopleBoxes(sensorCam);
  if (params.recLabels) {
    const txt = boxes.map(b => {
      const [x0, y0, x1, y1] = b.box;
      return `0 ${((x0 + x1) / 2).toFixed(6)} ${((y0 + y1) / 2).toFixed(6)} ${(x1 - x0).toFixed(6)} ${(y1 - y0).toFixed(6)}`;
    }).join('\n');
    recTask(recWrite(`labels/${name}.txt`, txt ? txt + '\n' : ''));
  }
  recTask(recWrite(`meta/${name}.json`, recFrameMeta(f, boxes, nPts)));
  REC.frame++;
}

// ---------------------------------------------------------------------------- start / stop
function recStamp() {
  const d = new Date(), p = v => String(v).padStart(2, '0');
  return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
}
async function recStart() {
  if (REC.on || REC.starting) return;
  REC.starting = true;
  try {
    REC.sessName = `${(params.recName || 'mob').replace(/[^\w\-]+/g, '_')}_${recStamp()}`;
    REC.session = null; REC.zip = null; REC.subdirs.clear();
    if (REC.dir) {
      if (REC.dir.queryPermission && (await REC.dir.queryPermission({ mode: 'readwrite' })) !== 'granted' &&
          (await REC.dir.requestPermission({ mode: 'readwrite' })) !== 'granted') {
        REC.msg = i18n('Permission to write to the folder was refused.'); recInfo(); return;
      }
      REC.session = await REC.dir.getDirectoryHandle(REC.sessName, { create: true });
    } else REC.zip = [];
    if (!params.sensors) { params.sensors = true; layoutSensorWindows(); }
    if (params.recLidar && !params.lidar) { params.lidar = true; lidarLayout(); }
    Object.assign(REC, { frame: 0, t0Sim: simTime, t0Wall: Date.now(), nextT: simTime, bytes: 0, dropped: 0, errors: 0, lastHead: L.head, msg: '', on: true });
    recTask(recWrite('session.json', recSessionJson(false)));
    gui.controllersRecursive().forEach(c => c.updateDisplay());
  } catch (e) {
    REC.msg = i18n('Could not start recording') + ': ' + (e && e.message || e);
  } finally {
    REC.starting = false;
    recInfo(); recButton();
  }
}
async function recStop(why) {
  if (!REC.on) return;
  REC.on = false;
  stepLimit = Infinity;
  REC.msg = why || '';
  recButton(); recInfo();
  await Promise.allSettled([...REC.pending]);
  try { await recWrite('session.json', recSessionJson(true)); } catch (e) { console.error(e); }
  if (REC.zip) {
    const blob = recZip(REC.zip);
    REC.zip = null;
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = REC.sessName + '.zip';
    document.body.appendChild(a); a.click(); a.remove();
    setTimeout(() => URL.revokeObjectURL(a.href), 10000);
  }
  if (!why) REC.msg = (LANG === 'zh' ? `已保存 ${REC.frame} 帧` : `Saved ${REC.frame} frames`) + ' → ' + (REC.session ? `${REC.dirName || '.'}/${REC.sessName}/` : `${REC.sessName}.zip`);
  recInfo();
}
const recToggle = () => (REC.on ? recStop() : recStart());

// ---------------------------------------------------------------------------- per frame
HOOKS.afterRender.push(() => {
  if (!REC.on) return;
  const dt = 1 / params.recFps;
  if (simTime >= REC.nextT - 1e-6) {
    if (REC.queue > REC_QUEUE_MAX && !params.recNoDrop) REC.dropped++;
    else recCapture();
    REC.nextT += dt;
    if (simTime >= REC.nextT) {                       // the simulation ran ahead of the capture clock
      const skip = Math.floor((simTime - REC.nextT) / dt) + 1;
      REC.dropped += skip; REC.nextT += skip * dt;
    }
  }
  // never drop: at most one capture interval per rendered frame, and hold while the writer is busy
  stepLimit = params.recNoDrop ? (REC.queue > REC_QUEUE_MAX ? 0 : dt / Math.max(params.timeScale, 1e-3)) : Infinity;
  if (params.recMaxSec > 0 && simTime - REC.t0Sim >= params.recMaxSec - 1e-6) recStop();
});

// ---------------------------------------------------------------------------- panel and badge
const fRec = gui.addFolder('Data recording');
{ const lf = gui.folders.find(f => f._title === 'LiDAR'); if (lf) gui.$children.insertBefore(fRec.domElement, lf.domElement.nextSibling); }
const recLoc = document.createElement('div');
recLoc.className = 'sea-info';
fRec.$children.appendChild(recLoc);
fRec.add({ f: recPickFolder }, 'f').name('Choose storage folder…');
fRec.add(params, 'recName').name('Session name');
fRec.add(params, 'recFps', { '1 Hz': 1, '2 Hz': 2, '5 Hz': 5, '10 Hz': 10, '15 Hz': 15, '20 Hz': 20, '30 Hz': 30 }).name('Capture rate');
fRec.add(params, 'recRgbRes', Object.fromEntries(REC_RGB_RES.map(([w, h], i) => [`${w} × ${h}`, i]))).name('RGB resolution');
fRec.add(params, 'recRgbFmt', { 'JPEG (quality 92)': 'jpg', 'PNG (lossless)': 'png' }).name('RGB format');
fRec.add(params, 'recIrRes', Object.fromEntries(REC_IR_RES.map(([w, h], i) => [`${w} × ${h}` + (w === 640 ? ' (native)' : ''), i]))).name('IR resolution');
fRec.add(params, 'recRgb').name('Save RGB images');
fRec.add(params, 'recIr').name('Save IR images');
fRec.add(params, 'recIrRaw').name('Save IR temperatures (16-bit)');
fRec.add(params, 'recLidar').name('Save LiDAR point clouds');
fRec.add(params, 'recLabels').name('Save labels (YOLO boxes)');
fRec.add(params, 'recLidarFmt', { 'PCD (binary)': 'pcd', 'LAS 1.2': 'las', 'LAZ (LASzip)': 'laz' }).name('LiDAR format');
fRec.add(params, 'lidarFrame', { 'Sensor (x fwd, y left, z up)': 0, 'World (x east, y north, z up)': 1 }).name('LiDAR coordinates').listen();
fRec.add(params, 'recNoDrop').name('Never drop frames (slows sim)');
fRec.add(params, 'recMaxSec', 0, 600, 1).name('Stop after (s, 0 = manual)');
const recBtn = fRec.add({ f: recToggle }, 'f').name('● Start recording');
const recStatus = document.createElement('div');
recStatus.className = 'sea-info';
fRec.$children.appendChild(recStatus);
const recSettings = fRec.controllers.filter(c => c !== recBtn);

const recBadge = document.createElement('div');
recBadge.id = 'recBadge';
recBadge.innerHTML = '<span class="dot"></span><span class="txt"></span><button type="button"></button>';
document.body.appendChild(recBadge);
const recCss = document.createElement('style');
recCss.textContent = `
#recBadge { position: fixed; top: 8px; left: 50%; transform: translateX(-50%); z-index: 1003; display: none; align-items: center; gap: 8px;
  padding: 5px 6px 5px 11px; border-radius: 16px; background: rgba(40, 6, 6, 0.72); border: 1px solid rgba(255, 110, 100, 0.55);
  color: #ffe2df; font: 12px/1 Consolas, "Cascadia Mono", "Microsoft YaHei", monospace; backdrop-filter: blur(6px); }
#recBadge .dot { width: 9px; height: 9px; border-radius: 50%; background: #ff3b30; animation: recBlink 1s steps(2, start) infinite; }
#recBadge button { all: unset; cursor: pointer; padding: 3px 9px; border-radius: 10px; background: rgba(255, 255, 255, 0.14); color: #fff; }
#recBadge button:hover { background: rgba(255, 255, 255, 0.26); }
@keyframes recBlink { to { visibility: hidden; } }`;
document.head.appendChild(recCss);
recBadge.querySelector('button').addEventListener('click', () => recStop());

const recMB = b => (b / 1048576).toFixed(b < 1048576 * 10 ? 1 : 0) + ' MB';
const recClock = s => `${String(Math.floor(s / 60)).padStart(2, '0')}:${(s % 60).toFixed(1).padStart(4, '0')}`;
function recInfo() {
  const zh = LANG === 'zh';
  const where = REC.dir ? `${REC.dirName}/` : (recFS ? i18n('No folder chosen: the recording downloads as a .zip') : i18n('This browser saves recordings as a .zip download'));
  recLoc.innerHTML = `<b>${i18n('Storage location')}</b><span>${where}${REC.on && REC.session ? REC.sessName + '/' : ''}</span>` +
    (REC.msg ? `<p>${REC.msg}</p>` : '');
  const t = REC.on ? simTime - REC.t0Sim : 0;
  recStatus.innerHTML = REC.on
    ? `<b>${zh ? '采集中' : 'Recording'} ${recClock(t)}</b><span>${REC.frame} ${zh ? '帧' : 'frames'} · ${recMB(REC.bytes)} · ${zh ? '队列' : 'queue'} ${REC.queue} · ${zh ? '丢帧' : 'dropped'} ${REC.dropped}</span>`
    : `<span>${zh ? '未在采集' : 'Not recording'}</span>`;
  recBadge.style.display = REC.on ? 'flex' : 'none';
  recBadge.querySelector('.txt').textContent = REC.on
    ? `${zh ? '采集' : 'REC'} ${recClock(t)} · ${REC.frame} ${zh ? '帧' : 'fr'} · ${recMB(REC.bytes)}${REC.dropped ? ` · ${zh ? '丢' : 'drop'} ${REC.dropped}` : ''}`
    : '';
  recBadge.querySelector('button').textContent = zh ? '■ 停止' : '■ Stop';
}
function recButton() {
  recBtn._enName = REC.on ? '■ Stop recording' : '● Start recording';
  recBtn.name(i18n(recBtn._enName));
  recSettings.forEach(c => c.enable(!REC.on));          // settings are fixed for the length of a session
}
let recUiT = 0;
HOOKS.update.push((dt, dtReal) => { recUiT -= dtReal; if (recUiT <= 0) { recUiT = 0.25; if (REC.on) recInfo(); } });
LANG_HOOKS.push(() => { recButton(); recInfo(); });
recInfo();
HOOKS.ready.push(() => {
  window.sim.rec = { REC, start: recStart, stop: recStop, setFolder: recSetFolder, zip: recZip, peopleBoxes: () => recPeopleBoxes(sensorCam) };
});
