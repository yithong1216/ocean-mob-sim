// ============================================================================
// Cinematic camera presets (feature module; the lens and post chain are in cine_post.js)
// ============================================================================
// Free orbit / sea level / drone orbit / helicopter overhead / rescue boat POV, with smooth transitions, focal length
// and handheld shake. The preset pose is applied just before the frame is rendered and the orbit camera is restored
// afterwards (cineCamApply / cineCamRestore, called from cine_post.js), so OrbitControls keeps its own state; mouse
// orbit and zoom still steer every preset (azimuth and distance around the group).
// Surface probes 240-245 (the people use 0-239, the orbit camera 255): rescue boat hull and the sea-level camera.
Object.assign(params, {
  cineCam: 0, cineFocal: Math.round(120 / Math.tan(camera.fov * D2R / 2)) / 10, cineBlend: 2.5,
  cineShake: false, cineShakeAmt: 0.5,
});
const CINE_CAM_MODES = { 'Free orbit': 0, 'Sea level (bobbing on the swell)': 1, 'Drone orbit': 2, 'Helicopter overhead': 3, 'Rescue boat POV': 4 };
const CINE_PROBE = 240;                    // 240 bow, 241 stern, 242 port, 243 starboard, 244 boat centre, 245 sea-level camera
const cineCam = {
  mode: 0, t: 1, time: 0, spin: 0,
  from: { p: new THREE.Vector3(), q: new THREE.Quaternion() },     // pose shown when the transition started
  pose: { p: new THREE.Vector3(), q: new THREE.Quaternion() },     // pose rendered this frame
  shown: { p: new THREE.Vector3(), q: new THREE.Quaternion() },    // last pose actually shown
  active: false, applied: false,
  heightAboveWater: 3, waterVy: 0, splash: 0,                      // read by the lens droplets
  sea: { init: false, y: 0, vy: 0, n: new THREE.Vector3(0, 1, 0) },
  boat: { init: false, h: 0, vh: 0, p: 0, vp: 0, r: 0, vr: 0 },
  saved: { p: new THREE.Vector3(), q: new THREE.Quaternion(), c: new THREE.Vector2(), xz: new THREE.Vector2(), dome: new THREE.Vector3() },
};
const _cineM = new THREE.Matrix4(), _cineV = new THREE.Vector3(), _cineV2 = new THREE.Vector3();
const _cineQ = new THREE.Quaternion(), _cineQ2 = new THREE.Quaternion(), _cineE = new THREE.Euler();
const CINE_UP = new THREE.Vector3(0, 1, 0);
function cineLook(q, eye, tx, ty, tz) { _cineV.set(tx, ty, tz); _cineM.lookAt(eye, _cineV, CINE_UP); return q.setFromRotationMatrix(_cineM); }
function cineQuery(i, x, z) { queryData[i * 4] = x; queryData[i * 4 + 1] = z; }
// damped second-order response x'' = w^2 (target - x) - 2 zeta w x' (semi-implicit Euler substeps)
function cineSpring(s, key, vkey, target, w, zeta, dt) {
  const n = Math.max(1, Math.ceil(dt / 0.01)), h = dt / n;
  for (let i = 0; i < n; i++) { s[vkey] += (w * w * (target - s[key]) - 2 * zeta * w * s[vkey]) * h; s[key] += s[vkey] * h; }
}
function cineSetFocal(f) {
  camera.fov = 2 * Math.atan(12 / f) * R2D;           // full-frame equivalent (24 mm sensor height)
  camera.updateProjectionMatrix();
  skyU.uPixAng.value = 2 * Math.tan(camera.fov * D2R / 2) / drawSize.y;
}
function cineSetMode(m) {
  const C = cineCam;
  C.from.p.copy(C.active ? C.shown.p : camera.position);
  C.from.q.copy(C.active ? C.shown.q : camera.quaternion);
  C.mode = m; C.t = params.cineBlend > 0.01 ? 0 : 1; C.spin = 0; C.sea.init = false; C.boat.init = false;
}

HOOKS.lateUpdate.push((dt, dtReal) => {
  const C = cineCam, P = C.pose.p, Q = C.pose.q, tgt = controls.target;
  C.time += dtReal;
  const az = Math.atan2(camera.position.x - tgt.x, camera.position.z - tgt.z);
  const dist = camera.position.distanceTo(tgt), sx = Math.sin(az), sz = Math.cos(az);
  // probe queries for the next frame (always written so a preset starts with valid water heights)
  const Db = clamp(dist * 1.4, 14, 90), bx = tgt.x + sx * Db, bz = tgt.z + sz * Db, fx = -sx, fz = -sz;
  const Ds = clamp(dist, 3, 60), seaX = tgt.x + sx * Ds, seaZ = tgt.z + sz * Ds;
  cineQuery(CINE_PROBE, bx + fx * 3, bz + fz * 3); cineQuery(CINE_PROBE + 1, bx - fx * 3, bz - fz * 3);
  cineQuery(CINE_PROBE + 2, bx + fz * 1.25, bz - fx * 1.25); cineQuery(CINE_PROBE + 3, bx - fz * 1.25, bz + fx * 1.25);
  cineQuery(CINE_PROBE + 4, bx, bz); cineQuery(CINE_PROBE + 5, seaX, seaZ);

  let hw = probeBuf[CAM_PROBE * 4], vy = probe(CAM_PROBE).vy;
  switch (C.mode) {
    case 0:
      P.copy(camera.position); Q.copy(camera.quaternion);
      break;
    case 1: {
      // a floating housing a little above the surface: heaves with the swell and tilts with the local slope
      const w = probe(CINE_PROBE + 5), S = C.sea;
      if (!S.init) { S.y = w.h + 0.45; S.vy = 0; S.n.set(-w.sx, 1, -w.sz).normalize(); S.init = true; }
      cineSpring(S, 'y', 'vy', w.h + 0.45, 4.0, 0.55, dt);
      S.y = Math.max(S.y, w.h + 0.24);
      S.n.lerp(_cineV2.set(-w.sx, 1, -w.sz).normalize(), 1 - Math.exp(-dt * 5)).normalize();
      P.set(seaX, S.y, seaZ);
      cineLook(Q, P, tgt.x, 0.3, tgt.z);
      Q.premultiply(_cineQ2.setFromUnitVectors(CINE_UP, _cineV2.copy(CINE_UP).lerp(S.n, 0.6).normalize()));
      hw = w.h; vy = w.vy;
      break;
    }
    case 2: {
      // drone: slow orbit at altitude, hover wander and a slight bank into the turn
      C.spin += dtReal * 0.05;
      const a = az + C.spin, R = clamp(dist * 1.8, 22, 160), alt = clamp(R * 0.5, 12, 90), T = C.time;
      P.set(tgt.x + Math.sin(a) * R + 0.5 * Math.sin(T * 0.23), alt + 0.4 * Math.sin(T * 0.31 + 1), tgt.z + Math.cos(a) * R + 0.5 * Math.cos(T * 0.19));
      cineLook(Q, P, tgt.x, 0, tgt.z);
      Q.multiply(_cineQ2.setFromAxisAngle(_cineV.set(0, 0, 1), -0.03));
      hw = 0; vy = 0;
      break;
    }
    case 3: {
      // helicopter: high and almost straight down, slow yaw, hover sway and rotor vibration
      C.spin += dtReal * 0.03;
      const alt = clamp(dist * 4, 45, 260), off = alt * 0.2, a = az + C.spin, T = C.time;
      P.set(tgt.x + Math.sin(a) * off + 1.2 * Math.sin(T * 0.17), alt + 0.8 * Math.sin(T * 0.29), tgt.z + Math.cos(a) * off + 1.2 * Math.cos(T * 0.13));
      cineLook(Q, P, tgt.x, 0, tgt.z);
      const vib = 0.0005;
      _cineE.set(vib * Math.sin(T * 71) + 0.0025 * Math.sin(T * 0.7), vib * Math.sin(T * 83 + 1) + 0.002 * Math.sin(T * 0.43), 0.0015 * Math.sin(T * 0.53), 'YXZ');
      Q.multiply(_cineQ2.setFromEuler(_cineE));
      hw = 0; vy = 0;
      break;
    }
    case 4: {
      // rescue boat (7 m RHIB) holding station off the group: heave, pitch and roll from the hull probes
      const B = C.boat, hb = probe(CINE_PROBE).h, hs = probe(CINE_PROBE + 1).h, hp = probe(CINE_PROBE + 2).h;
      const hsb = probe(CINE_PROBE + 3).h, hc = probe(CINE_PROBE + 4).h;
      const heaveT = (hb + hs + hp + hsb + 2 * hc) / 6, pitchT = Math.atan2(hb - hs, 6), rollT = Math.atan2(hp - hsb, 2.5);
      if (!B.init) { Object.assign(B, { init: true, h: heaveT, vh: 0, p: pitchT, vp: 0, r: rollT, vr: 0 }); }
      cineSpring(B, 'h', 'vh', heaveT, 2.2, 0.4, dt);
      cineSpring(B, 'p', 'vp', pitchT, 2.6, 0.35, dt);
      cineSpring(B, 'r', 'vr', rollT, 1.9, 0.18, dt);
      const yaw = Math.atan2(-fx, -fz);
      _cineE.set(B.p, yaw, -B.r, 'YXZ');
      _cineQ.setFromEuler(_cineE);                                         // boat attitude
      P.set(0, 2.25, 0.6).applyQuaternion(_cineQ).add(_cineV.set(bx, B.h, bz));   // eye at the console
      cineLook(Q, P, tgt.x, 0.3, tgt.z);
      // the operator compensates about half of the pitch and roll
      _cineE.set(B.p, 0, -B.r, 'YXZ');
      _cineQ2.setFromEuler(_cineE);
      _cineQ.setFromAxisAngle(CINE_UP, yaw);
      _cineQ2.premultiply(_cineQ).multiply(_cineQ.invert());
      Q.premultiply(_cineQ.identity().slerp(_cineQ2, 0.5));
      // bow slamming into a crest throws spray at the lens
      const bowDeck = B.h + 0.9 + Math.sin(B.p) * 3;
      if (hb > bowDeck - 0.3 && probe(CINE_PROBE).vy > 0.4) C.splash += 25 * dt;
      hw = hc; vy = probe(CINE_PROBE + 4).vy;
      break;
    }
  }
  // smooth transition from the pose shown when the preset changed
  if (C.t < 1) {
    C.t = Math.min(1, C.t + dtReal / Math.max(params.cineBlend, 0.01));
    const s = C.t * C.t * C.t * (C.t * (C.t * 6 - 15) + 10);
    P.lerpVectors(C.from.p, P, s);
    _cineQ.copy(Q); Q.slerpQuaternions(C.from.q, _cineQ, s);
  }
  // handheld: a few incommensurate slow sines (breathing, hand tremor); always a little on the boat
  const A = (params.cineShake ? params.cineShakeAmt * 0.0045 : 0) + (C.mode === 4 ? 0.0018 : 0);
  if (A > 0) {
    const T = C.time;
    _cineE.set(A * (0.6 * Math.sin(T * 1.13) + 0.3 * Math.sin(T * 2.71 + 1.3) + 0.12 * Math.sin(T * 6.3 + 0.4)),
      A * (0.6 * Math.sin(T * 0.91 + 2.1) + 0.3 * Math.sin(T * 2.27 + 0.2) + 0.12 * Math.sin(T * 5.7 + 2.9)),
      A * 0.5 * (0.7 * Math.sin(T * 0.77 + 0.9) + 0.3 * Math.sin(T * 1.93)), 'YXZ');
    Q.multiply(_cineQ2.setFromEuler(_cineE));
    P.y += A * 2.5 * Math.sin(T * 1.6 + 0.3);
  }
  C.active = C.mode !== 0 || C.t < 1 || A > 0;
  C.shown.p.copy(C.active ? P : camera.position); C.shown.q.copy(C.active ? Q : camera.quaternion);
  C.heightAboveWater = C.shown.p.y - (C.mode === 2 || C.mode === 3 ? 0 : hw);
  C.waterVy = vy;
});

// called from cine_post.js just before / after the frame (and the sensor windows) are rendered
function cineCamApply() {
  const C = cineCam, S = C.saved;
  if (!C.active) return;
  S.p.copy(camera.position); S.q.copy(camera.quaternion);
  S.c.copy(oceanU.uCenter.value); S.xz.copy(skyU.uCamXZ.value); S.dome.copy(skyDome.position);
  camera.position.copy(C.pose.p); camera.quaternion.copy(C.pose.q);
  camera.updateMatrixWorld();
  oceanU.uCenter.value.set(camera.position.x, camera.position.z);
  skyU.uCamXZ.value.set(camera.position.x, camera.position.z);
  skyDome.position.copy(camera.position);
  if (params.sensorSource === 0) { sensorCam.position.copy(camera.position); sensorCam.quaternion.copy(camera.quaternion); sensorCam.updateMatrixWorld(); }
  C.applied = true;
}
function cineCamRestore() {
  const C = cineCam, S = C.saved;
  if (!C.applied) return;
  camera.position.copy(S.p); camera.quaternion.copy(S.q);
  camera.updateMatrixWorld();
  oceanU.uCenter.value.copy(S.c); skyU.uCamXZ.value.copy(S.xz); skyDome.position.copy(S.dome);
  C.applied = false;
}

const cineGui = gui.addFolder('Cinematic');
const fCineCam = cineGui.addFolder('Camera');
fCineCam.add(params, 'cineCam', CINE_CAM_MODES).name('Camera preset').onChange(cineSetMode);
fCineCam.add(params, 'cineFocal', 12, 135, 0.5).name('Focal length (mm, FF)').onChange(cineSetFocal);
fCineCam.add(params, 'cineBlend', 0, 6, 0.1).name('Transition (s)');
fCineCam.add(params, 'cineShake').name('Handheld shake');
fCineCam.add(params, 'cineShakeAmt', 0, 2, 0.01).name('Shake amount');
