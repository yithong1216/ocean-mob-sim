// ============================================================================
// Search-and-rescue gear in the water: lifebuoy (SOLAS light, grab line, retroreflective tape), canopied life
// raft, floating debris, a rescue vessel on the horizon, and a fluorescein sea-marker dye patch.
// All geometry is procedural. Buoyancy uses the GPU surface probes 246..254 like the people do
// (240-245 belong to the cinematic module, 255 to the camera).
// ============================================================================
const UW_PROBE0 = 246;      // 246 lifebuoy, 247-248 raft, 249-251 debris, 252-254 vessel
const _ug = {
  r: new THREE.Vector3(), pv: new THREE.Vector3(), F: new THREE.Vector3(), T: new THREE.Vector3(), n: new THREE.Vector3(),
  qi: new THREE.Quaternion(), dq: new THREE.Quaternion(), v: new THREE.Vector3(), m: new THREE.Matrix4(),
};

// sprites (light glows) must not add to the LWIR radiance image
function uwNoThermal(mat) {
  mat.onBeforeCompile = sh => {
    sh.uniforms.uThermal = THERM.uThermal;
    sh.fragmentShader = 'uniform float uThermal;\n' + sh.fragmentShader.replace('void main() {', 'void main() {\n  if (uThermal > 0.5) discard;');
  };
  mat.customProgramCacheKey = () => 'uw-nothermal';
  return mat;
}
// opaque solid that is not in contact with the water (vessel): air-temperature paint, hot exhaust, aerial haze
function uwSolidMat(o, dT = 0, eps = 0.9) {
  const mat = new THREE.MeshStandardMaterial(o);
  const dTu = { value: new THREE.Vector2(dT, eps) };
  mat.onBeforeCompile = sh => {
    Object.assign(sh.uniforms, THERM, { uDT: dTu, uFogExt: oceanU.uFogExt });
    sh.fragmentShader = THERMAL_UNIFORMS_GLSL + 'uniform vec2 uDT;\nuniform vec4 uFogExt;\n' + THERMAL_GLSL + sh.fragmentShader
      .replace('#include <dithering_fragment>', `#include <dithering_fragment>
  {
    vec3 uwDirW = normalize((vec4(-vViewPosition, 0.0) * viewMatrix).xyz);   // camera -> point, world
    float uwDist = length(vViewPosition);
    if (uThermal > 0.5) {
      vec3 nW = normalize((vec4(normal, 0.0) * viewMatrix).xyz);
      float Lsky = skyRadianceLWIR(max(reflect(uwDirW, nW).y, 0.0), uTair, uRH, uCloudFrac, uTcloud);
      float T = uTair + uDT.x + (uDT.x < 20.0 ? 4.0 * uSunIrr / 800.0 * max(nW.y, 0.3) : 0.0);
      float tau = pathTransmittanceLWIR(uwDist, uRH, uTair);
      gl_FragColor = vec4((uDT.y * planckBand(T) + (1.0 - uDT.y) * Lsky) * tau + planckBand(uTair) * (1.0 - tau), 0.0, 0.0, 1.0);
    } else {
#ifdef USE_ENVMAP
      // aerial perspective matching the ocean's: haze towards the horizon sky in the view direction
      float hr = max(cameraPosition.y, 0.0) / 1200.0;
      float dens = hr > 1e-3 ? (1.0 - exp(-hr)) / hr : 1.0;
      vec3 Tf = exp(-max(uFogExt.rgb, vec3(uFogExt.w)) * uwDist * dens);
      vec3 fogC = textureCubeUV(envMap, envMapRotation * normalize(vec3(uwDirW.x, 0.01, uwDirW.z)), 0.25).rgb;
      gl_FragColor.rgb = gl_FragColor.rgb * Tf + fogC * (1.0 - Tf);
#endif
    }
  }`);
  };
  mat.customProgramCacheKey = () => 'uw-solid';
  return mat;
}
function uwWet(mat, B, dark = 0.85) { makeWet(mat, B.wetU, dark, 6); return mat; }
function uwGlow(color, scale) {
  const s = new THREE.Sprite(uwNoThermal(new THREE.SpriteMaterial({ map: GLOW_TEX, color, transparent: true, opacity: 0, depthWrite: false, blending: THREE.AdditiveBlending })));
  s.scale.setScalar(scale);
  return s;
}

// ---- generic floating rigid body (same force model as the people: probe spheres, drag, windage)
// elems: { p: [x, y, z] body frame, r: sphere radius, V: volume (0 = drag-only point), area, cd, lin, wind }
function uwMakeBody(o) {
  const B = Object.assign({
    pos: new THREE.Vector3(), vel: new THREE.Vector3(), w: new THREE.Vector3(), q: new THREE.Quaternion(),
    qx: o.probes.map(() => 0), qz: o.probes.map(() => 0), ps: [], ready: 0, sub: 0, added: 0, cAng: 1, visible: true,
    wetU: { plane: { value: new THREE.Vector4(0, 1, 0, 0) }, base: { value: o.wetBase ?? 0.25 }, head: { value: new THREE.Vector3(0, -1e4, 0) } },
  }, o);
  B.elems = o.elems.map(e => {
    let k = 0, best = 1e9;
    o.probes.forEach((p, i) => { const d = (p[0] - e.p[0]) ** 2 + (p[1] - e.p[2]) ** 2; if (d < best) { best = d; k = i; } });
    return { r: 0.05, V: 0, area: 0, cd: 1, lin: 0, wind: 0, ...e, local: new THREE.Vector3(...e.p), k };
  });
  B.Vtot = B.elems.reduce((s, e) => s + e.V, 0);
  B.root.traverse(m => { if (m.isMesh) { m.layers.enable(PEOPLE_LAYER); m.frustumCulled = false; } });
  scene.add(B.root);
  return B;
}
function uwStepBody(B, dt, t) {
  const { r, pv, F, T, qi, dq } = _ug;
  const m = B.mass, dts = t - probeTime, kp = sea.kp;
  F.set(0, -m * G, 0); T.set(0, 0, 0);
  let Vsub = 0;
  for (const e of B.elems) {
    const s = B.ps[e.k];
    r.copy(e.local).applyQuaternion(B.q);
    const px = B.pos.x + r.x, py = B.pos.y + r.y, pz = B.pos.z + r.z;
    // elements sit up to ~0.8 m from their probe: limit the (ripple-dominated) slope used to extrapolate
    const h = s.h + s.vy * dts + clamp(s.sx, -0.35, 0.35) * (px - B.qx[e.k]) + clamp(s.sz, -0.35, 0.35) * (pz - B.qz[e.k]);
    const depth = h - py;
    const frac = e.V > 0 ? capFraction(depth, e.r) : depth > 0 ? 1 : 0;
    Vsub += e.V * frac;
    pv.crossVectors(B.w, r).add(B.vel);
    let fx = 0, fy = RHO * G * e.V * frac, fz = 0;
    if (frac > 0) {
      const dec = Math.exp(-kp * Math.max(depth, 0));
      const rx = pv.x - (s.vx * dec + sea.current.x), ry = pv.y - s.vy * dec, rz = pv.z - (s.vz * dec + sea.current.z);
      const c = (0.5 * RHO * e.cd * e.area * Math.hypot(rx, ry, rz) + e.lin) * frac;
      fx -= c * rx; fy -= c * ry; fz -= c * rz;
    }
    if (frac < 1 && e.wind > 0) {
      const ax = sea.wind.x - pv.x, az = sea.wind.z - pv.z;
      const c = 0.5 * RHO_AIR * 1.0 * e.wind * (1 - frac) * Math.hypot(ax, az);
      fx += c * ax; fz += c * az;
    }
    F.x += fx; F.y += fy; F.z += fz;
    T.x += r.y * fz - r.z * fy; T.y += r.z * fx - r.x * fz; T.z += r.x * fy - r.y * fx;
  }
  B.sub = B.Vtot > 0 ? Vsub / B.Vtot : 0;
  T.addScaledVector(B.w, -B.cAng);
  const madd = B.added + 0.5 * RHO * Vsub, sc = 1 + madd / m;
  B.vel.addScaledVector(F, dt / (m + madd));
  if (B.vel.lengthSq() > 100) B.vel.setLength(10);
  B.pos.addScaledVector(B.vel, dt);
  qi.copy(B.q).invert();
  T.applyQuaternion(qi);
  pv.copy(B.w).applyQuaternion(qi);
  pv.x += T.x / (B.I[0] * sc) * dt; pv.y += T.y / (B.I[1] * sc) * dt; pv.z += T.z / (B.I[2] * sc) * dt;
  B.w.copy(pv.applyQuaternion(B.q));
  if (B.w.lengthSq() > 64) B.w.setLength(8);
  dq.set(B.w.x * dt * 0.5, B.w.y * dt * 0.5, B.w.z * dt * 0.5, 0).multiply(B.q);
  B.q.set(B.q.x + dq.x, B.q.y + dq.y, B.q.z + dq.z, B.q.w + dq.w).normalize();
}
function uwPlaceBody(B, x, z, yaw = Math.random() * 6.28) {
  B.pos.set(x, B.y0 ?? 0, z); B.vel.set(0, 0, 0); B.w.set(0, 0, 0);
  B.q.setFromAxisAngle(_ug.v.set(0, 1, 0), yaw);
  B.ready = 0;
}
// after the physics: write this body's probe queries (sampled next frame) and the waterline plane
function uwPostBody(B) {
  B.root.position.copy(B.pos); B.root.quaternion.copy(B.q);
  B.probes.forEach((p, k) => {
    _ug.r.set(p[0], 0, p[1]).applyQuaternion(B.q).add(B.pos);
    queryData[(B.probeBase + k) * 4] = _ug.r.x; queryData[(B.probeBase + k) * 4 + 1] = _ug.r.z;
    B.qx[k] = _ug.r.x; B.qz[k] = _ug.r.z;
  });
  const s = B.ps[0];
  if (s) {
    _ug.n.set(-s.sx, 1, -s.sz).normalize();
    const h = s.h + s.sx * (B.pos.x - B.qx[0]) + s.sz * (B.pos.z - B.qz[0]);
    B.wetU.plane.value.set(_ug.n.x, _ug.n.y, _ug.n.z, -(_ug.n.x * B.pos.x + _ug.n.y * h + _ug.n.z * B.pos.z));
  }
}

// ---- rope festoons (grab lines) hanging between attachment points around a ring
function uwFestoon(R, y, n, sag, rad, mat, a0 = 0) {
  const g = new THREE.Group();
  for (let i = 0; i < n; i++) {
    const pts = [];
    for (let k = 0; k <= 12; k++) {
      const t = k / 12, a = a0 + (i + t) / n * Math.PI * 2, s = Math.sin(Math.PI * t);
      pts.push(new THREE.Vector3((R + sag * 0.6 * s) * Math.cos(a), y - sag * s, (R + sag * 0.6 * s) * Math.sin(a)));
    }
    g.add(new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), 20, rad, 5), mat));
  }
  return g;
}

// ---- lifebuoy: SOLAS ring (0.76 m outer / 0.48 m inner), orange with white quarters, retroreflective tape,
//      a festooned grab line and a self-igniting light
const uwBuoy = (() => {
  const R = 0.31, r = 0.07;
  const root = new THREE.Group();
  const B = { root, mass: 3.2, I: [0.16, 0.3, 0.16], probeBase: UW_PROBE0, probes: [[0, 0]], wetBase: 0.2, cAng: 0.4 };
  B.wetU = { plane: { value: new THREE.Vector4(0, 1, 0, 0) }, base: { value: 0.2 }, head: { value: new THREE.Vector3(0, -1e4, 0) } };
  const orange = uwWet(new THREE.MeshPhysicalMaterial({ color: 0xff5208, roughness: 0.42, clearcoat: 0.5, clearcoatRoughness: 0.3 }), B);
  const white = uwWet(new THREE.MeshPhysicalMaterial({ color: 0xf2f2ee, roughness: 0.4, clearcoat: 0.5, clearcoatRoughness: 0.3 }), B);
  const tape = uwWet(new THREE.MeshStandardMaterial({ color: 0xd8dde0, roughness: 0.22, metalness: 0.75, emissive: 0xffffff, emissiveIntensity: 0 }), B, 0.95);
  const rope = uwWet(new THREE.MeshStandardMaterial({ color: 0xe8e2c8, roughness: 0.9 }), B, 0.7);
  const ring = new THREE.Group();
  for (let i = 0; i < 4; i++) {
    const a = i * Math.PI / 2;
    const seg = new THREE.Mesh(new THREE.TorusGeometry(R, r, 14, 18, Math.PI / 2 - 0.36), orange);
    seg.rotation.z = a + 0.18;
    const band = new THREE.Mesh(new THREE.TorusGeometry(R, r, 14, 5, 0.36), white);
    band.rotation.z = a - 0.18;
    const tp = new THREE.Mesh(new THREE.TorusGeometry(R, r * 1.03, 14, 3, 0.1), tape);
    tp.rotation.z = a - 0.05;
    ring.add(seg, band, tp);
  }
  ring.rotation.x = -Math.PI / 2;
  root.add(ring);
  root.add(uwFestoon(R + r + 0.005, -0.01, 4, 0.06, 0.0065, rope, Math.PI / 4));
  // SOLAS light on a white quarter: orange housing, clear lens, flashing ~60/min
  const light = new THREE.Group();
  const housing = new THREE.Mesh(new THREE.CylinderGeometry(0.034, 0.04, 0.17, 16), uwWet(new THREE.MeshPhysicalMaterial({ color: 0xff7a00, roughness: 0.5, clearcoat: 0.4 }), B));
  housing.position.y = 0.085;
  const bulbMat = uwWet(new THREE.MeshStandardMaterial({ color: 0xf6f8ff, roughness: 0.1, emissive: 0xffffff, emissiveIntensity: 0, transparent: true, opacity: 0.85 }), B, 1);
  const lens = new THREE.Mesh(new THREE.SphereGeometry(0.03, 14, 10, 0, Math.PI * 2, 0, Math.PI / 2), bulbMat);
  lens.position.y = 0.17;
  light.add(housing, lens);
  light.position.set(R, r * 0.7, 0);
  root.add(light);
  const glow = uwGlow(0xfff4dc, 0.5);
  glow.position.set(R, r * 0.7 + 0.19, 0);
  root.add(glow);
  B.elems = Array.from({ length: 8 }, (_, i) => {
    const a = i / 8 * Math.PI * 2;
    return { p: [R * Math.cos(a), 0, R * Math.sin(a)], r, V: 2 * Math.PI * Math.PI * R * r * r / 8, area: 0.012, cd: 1.0, lin: 1.2, wind: 0.02 };
  });
  B.elems.push({ p: [R, 0.16, 0], V: 0, wind: 0.008 });
  const body = uwMakeBody(B);
  Object.assign(body, { tape, bulbMat, glow, strobePh: Math.random() });
  return body;
})();

// ---- life raft: two stacked orange buoyancy tubes, canopy on an inflatable arch, ballast pockets, lifeline
//      (95 kg: raft, survival pack and some water on the floor, so the lower tube sits in the water)
const uwRaft = (() => {
  const R = 0.82, r = 0.15;
  const root = new THREE.Group();
  const B = { root, mass: 95, I: [30, 55, 30], probeBase: UW_PROBE0 + 1, probes: [[0.6 * R, 0], [-0.6 * R, 0]], wetBase: 0.3, added: 150, cAng: 40 };
  B.wetU = { plane: { value: new THREE.Vector4(0, 1, 0, 0) }, base: { value: 0.3 }, head: { value: new THREE.Vector3(0, -1e4, 0) } };
  const tubeM = uwWet(new THREE.MeshPhysicalMaterial({ color: 0xe8480e, roughness: 0.5, clearcoat: 0.35, clearcoatRoughness: 0.4 }), B);
  const canopyM = uwWet(new THREE.MeshPhysicalMaterial({ color: 0xff6312, roughness: 0.62, sheen: 0.4, sheenColor: 0xffb070, side: THREE.DoubleSide }), B);
  const innerM = uwWet(new THREE.MeshStandardMaterial({ color: 0x4a1a08, roughness: 0.9, side: THREE.BackSide }), B);
  const floorM = uwWet(new THREE.MeshStandardMaterial({ color: 0x2b2d30, roughness: 0.8 }), B);
  const ballastM = uwWet(new THREE.MeshStandardMaterial({ color: 0xd6a21a, roughness: 0.75, side: THREE.DoubleSide }), B);
  const tapeM = uwWet(new THREE.MeshStandardMaterial({ color: 0xd8dde0, roughness: 0.22, metalness: 0.75, emissive: 0xffffff, emissiveIntensity: 0 }), B, 0.95);
  const ropeM = uwWet(new THREE.MeshStandardMaterial({ color: 0xdedad0, roughness: 0.9 }), B, 0.7);
  for (const y of [0.12, 0.41]) {
    const t = new THREE.Mesh(new THREE.TorusGeometry(R, r, 14, 48), tubeM);
    t.rotation.x = -Math.PI / 2; t.position.y = y;
    root.add(t);
  }
  const floor = new THREE.Mesh(new THREE.CylinderGeometry(R, R * 0.97, 0.04, 40), floorM);
  floor.position.y = -0.01;
  root.add(floor);
  for (let i = 0; i < 4; i++) {
    const a = (i + 0.5) / 4 * Math.PI * 2;
    const bag = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.08, 0.36, 14, 1, true), ballastM);
    bag.position.set(0.48 * Math.cos(a), -0.2, 0.48 * Math.sin(a));
    root.add(bag);
  }
  // canopy: dome on top of the upper tube with an entrance gap facing +x
  const prof = [];
  for (let i = 0; i <= 14; i++) { const f = i / 14 * Math.PI / 2; prof.push(new THREE.Vector2(Math.max(R * 1.02 * Math.cos(f), 0.001), 0.52 + 0.88 * Math.sin(f))); }
  const gap = 0.95;
  const dome = new THREE.Mesh(new THREE.LatheGeometry(prof, 44, Math.PI / 2 + gap / 2, Math.PI * 2 - gap), canopyM);
  const inner = new THREE.Mesh(new THREE.LatheGeometry(prof.map(p => new THREE.Vector2(p.x * 0.97, p.y - 0.02)), 44, Math.PI / 2 + gap / 2, Math.PI * 2 - gap), innerM);
  root.add(dome, inner);
  const arch = new THREE.Mesh(new THREE.TorusGeometry(R * 1.0, 0.055, 10, 28, Math.PI), tubeM);
  arch.scale.set(1, 1.07, 1); arch.position.y = 0.52; arch.rotation.y = Math.PI / 2;
  root.add(arch);
  // retroreflective patches on the canopy
  for (let i = 0; i < 4; i++) {
    const a = Math.PI / 4 + i * Math.PI / 2, f = 0.75;
    const p = new THREE.Vector3(R * 1.03 * Math.cos(f) * Math.cos(a), 0.52 + 0.89 * Math.sin(f), -R * 1.03 * Math.cos(f) * Math.sin(a));
    const patch = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.16, 0.006), tapeM);
    patch.position.copy(p);
    patch.lookAt(p.clone().multiplyScalar(2).setY(p.y * 2 - 0.52));
    root.add(patch);
  }
  // canopy light on the apex
  const lampM = uwWet(new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0xfff2d0, emissiveIntensity: 0, roughness: 0.2 }), B, 1);
  const lamp = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.035, 0.07, 12), lampM);
  lamp.position.y = 1.42;
  root.add(lamp);
  const glow = uwGlow(0xfff0d8, 0.32);
  glow.position.y = 1.48;
  root.add(glow);
  root.add(uwFestoon(R + r + 0.004, 0.42, 8, 0.07, 0.008, ropeM));
  B.elems = [];
  for (let i = 0; i < 12; i++) {
    const a = i / 12 * Math.PI * 2, c = Math.cos(a), s = Math.sin(a);
    B.elems.push({ p: [R * c, 0.12, R * s], r, V: 2 * Math.PI * Math.PI * R * r * r / 12, area: 0.06, cd: 0.9, lin: 4 });
    B.elems.push({ p: [R * c, 0.41, R * s], r, V: 2 * Math.PI * Math.PI * R * r * r / 12, area: 0.04, cd: 0.9, lin: 2, wind: 0.08 });
  }
  for (let i = 0; i < 4; i++) {
    const a = (i + 0.5) / 4 * Math.PI * 2;
    B.elems.push({ p: [0.48 * Math.cos(a), -0.22, 0.48 * Math.sin(a)], V: 0, area: 0.14, cd: 1.6, lin: 25 });  // ballast pockets
  }
  B.elems.push({ p: [0, 1.0, 0], V: 0, wind: 1.2 });       // canopy windage
  B.y0 = -0.06;
  const body = uwMakeBody(B);
  Object.assign(body, { tape: tapeM, lampM, glow });
  return body;
})();

// ---- debris: a plank, a jerrycan, a cool box
const uwDebris = (() => {
  const list = [];
  const mk = (i, build, mass, I, elems, wetBase) => {
    const root = new THREE.Group();
    const B = { root, mass, I, probeBase: UW_PROBE0 + 3 + i, probes: [[0, 0]], wetBase, cAng: mass * 0.05 };
    B.wetU = { plane: { value: new THREE.Vector4(0, 1, 0, 0) }, base: { value: wetBase }, head: { value: new THREE.Vector3(0, -1e4, 0) } };
    build(root, B);
    B.elems = elems;
    list.push(uwMakeBody(B));
  };
  // weathered plank 1.4 x 0.22 x 0.045 m, ~550 kg/m3: floats about half submerged
  mk(0, (root, B) => {
    const g = new THREE.BoxGeometry(1.4, 0.045, 0.22, 8, 1, 2);
    root.add(new THREE.Mesh(g, uwWet(new THREE.MeshStandardMaterial({ color: 0x8a6a48, roughness: 0.88 }), B, 0.6)));
  }, 7.6, [0.03, 1.25, 1.24], [-0.5, 0, 0.5].flatMap(x => [-0.07, 0.07].map(z => ({ p: [x, 0, z], r: 0.0225, V: 0.01386 / 6, area: 0.03, cd: 1.1, lin: 2, wind: 0.01 }))), 0.6);
  // 20 l jerrycan on its side, nearly empty: rides high
  mk(1, (root, B) => {
    const m = uwWet(new THREE.MeshPhysicalMaterial({ color: 0xc81e1e, roughness: 0.45, clearcoat: 0.3 }), B);
    const box = new THREE.Mesh(new THREE.BoxGeometry(0.46, 0.17, 0.34, 2, 1, 2), m);
    const handle = new THREE.Mesh(new THREE.TorusGeometry(0.05, 0.012, 6, 12, Math.PI), m);
    handle.position.set(0.12, 0.085, 0); handle.rotation.y = Math.PI / 2;
    const spout = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.05, 10), m);
    spout.rotation.z = Math.PI / 2; spout.position.set(0.255, 0.04, 0.1);
    root.add(box, handle, spout);
  }, 2.2, [0.03, 0.06, 0.045], [-0.15, 0.15].flatMap(x => [-0.11, 0.11].map(z => ({ p: [x, 0, z], r: 0.085, V: 0.0066, area: 0.02, cd: 1.0, lin: 1.5, wind: 0.018 }))), 0.3);
  // cool box with a lid seam
  mk(2, (root, B) => {
    const m = uwWet(new THREE.MeshPhysicalMaterial({ color: 0xe9ecea, roughness: 0.5, clearcoat: 0.2 }), B);
    const lid = uwWet(new THREE.MeshPhysicalMaterial({ color: 0x2f6fb0, roughness: 0.5, clearcoat: 0.2 }), B);
    const box = new THREE.Mesh(new THREE.BoxGeometry(0.56, 0.3, 0.36), m);
    const top = new THREE.Mesh(new THREE.BoxGeometry(0.58, 0.06, 0.38), lid);
    top.position.y = 0.18;
    root.add(box, top);
  }, 4.5, [0.09, 0.16, 0.14], [-0.18, 0.18].flatMap(x => [-0.1, 0.1].map(z => ({ p: [x, 0.03, z], r: 0.17, V: 0.0165, area: 0.04, cd: 1.0, lin: 2, wind: 0.04 }))), 0.3);
  return list;
})();

// ---- rescue vessel on the horizon: low-poly 26 m patrol / rescue boat, kinematic, pitching and rolling on the swell
function uwHullGeometry(L, Bm, D) {
  const g = new THREE.BoxGeometry(L, D, Bm, 28, 6, 8);
  const pos = g.attributes.position, col = [];
  const cTop = new THREE.Color(0xb3301c), cBoot = new THREE.Color(0x141414), cBottom = new THREE.Color(0x5a1a14), cRub = new THREE.Color(0x1d1f22);
  for (let i = 0; i < pos.count; i++) {
    let x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
    const u = x / L + 0.5, yb = (y + D / 2) / D;
    const plan = u > 0.5 ? Math.sqrt(Math.max(0, 1 - ((u - 0.5) / 0.5) ** 1.8)) : 1 - 0.1 * ((0.5 - u) / 0.5) ** 2;
    const sec = 0.22 + 0.78 * Math.pow(yb, 0.45);            // deadrise: V-shaped below, flared above
    z *= plan * sec;
    y += 0.9 * u * u * yb;                                    // sheer rises forward
    y += D * 0.75 * Math.max(0, (u - 0.62) / 0.38) ** 2 * (1 - yb);   // forefoot
    pos.setXYZ(i, x, y, z);
    const c = y > D / 2 - 0.25 ? cRub : y > 0.35 ? cTop : y > -0.15 ? cBoot : cBottom;
    col.push(c.r, c.g, c.b);
  }
  g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
  g.computeVertexNormals();
  return g;
}
const uwVessel = (() => {
  const L = 26, Bm = 6.4, D = 3.6;
  const root = new THREE.Group(), hullG = new THREE.Group();
  const hull = new THREE.Mesh(uwHullGeometry(L, Bm, D), uwSolidMat({ vertexColors: true, roughness: 0.55 }, 1.5, 0.92));
  hull.position.y = D / 2 - 1.7;                              // 1.7 m draft
  hullG.add(hull);
  const white = uwSolidMat({ color: 0xe8ebee, roughness: 0.45 }, 1.5, 0.9);
  const glass = uwSolidMat({ color: 0x0b1218, roughness: 0.08, metalness: 0.4 }, 0.5, 0.85);
  const grey = uwSolidMat({ color: 0x8a9096, roughness: 0.5 }, 1.0, 0.9);
  const hot = uwSolidMat({ color: 0x1a1a1a, roughness: 0.6 }, 55, 0.95);   // exhaust stack: ~75 C in LWIR
  const box = (w, h, d, m, x, y, z) => { const b = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), m); b.position.set(x, y, z); hullG.add(b); return b; };
  box(9, 2.4, 5.0, white, 1.0, 2.3 + 1.2, 0);                 // deckhouse
  box(9.05, 0.75, 5.05, glass, 1.0, 2.3 + 1.75, 0);           // window band
  box(5, 1.9, 4.4, white, 1.6, 4.6 + 0.95, 0);                 // wheelhouse
  box(5.05, 0.8, 4.45, glass, 1.6, 4.6 + 1.2, 0);
  box(1.0, 0.25, 5.4, white, 2.6, 6.6, 0);                     // bridge wings / roof
  const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.18, 5.5, 8), grey);
  mast.position.set(0.8, 6.5 + 2.75, 0); hullG.add(mast);
  box(0.15, 0.15, 2.4, grey, 0.8, 8.4, 0);                      // yard
  const radar = box(0.35, 0.2, 2.6, grey, 0.8, 9.6, 0);
  box(0.8, 2.2, 0.8, hot, -2.6, 4.6, 1.7);                      // exhaust stacks
  box(0.8, 2.2, 0.8, hot, -2.6, 4.6, -1.7);
  box(4.5, 0.9, 2.4, uwSolidMat({ color: 0xff6a1a, roughness: 0.5 }, 1.5, 0.9), -8.5, 2.4, 0);   // rescue RIB on the stern deck
  // navigation lights (night): masthead white, sidelights red / green, stern white
  const lights = [[0xffffff, 0.8, 10.2, 0, 3.5], [0xff2a1a, 3.0, 5.6, -2.8, 2.2], [0x2aff5a, 3.0, 5.6, 2.8, 2.2], [0xffffff, -12.5, 2.6, 0, 2.2]]
    .map(([c, x, y, z, s]) => { const g = uwGlow(c, s); g.position.set(x, y, z); hullG.add(g); return g; });
  root.add(hullG);
  root.traverse(m => { if (m.isMesh) m.frustumCulled = false; });
  scene.add(root);
  return { root, hullG, radar, lights, L, Bm, theta: null, heave: 0, pitch: 0, roll: 0, ps: [], probeBase: UW_PROBE0 + 6, pos: new THREE.Vector3(), yaw: 0 };
})();

// ---- fluorescein sea-marker dye: released from the first person's lifejacket, drifting with the surface
//      current, spreading (scale-dependent diffusion) and stretched downwind into streaks
const UW_DYE_N = 14;
const uwDyeU = {
  uDye: { value: Array.from({ length: UW_DYE_N }, () => new THREE.Vector4()) },    // x, z, sigma across, mass
  uDyeS: { value: new Float32Array(UW_DYE_N) },                                    // sigma along the stretch axis
  uDyeAx: { value: new THREE.Vector4(1, 0, 0, 0) },                                // stretch axis, time
  uDyeBox: { value: new THREE.Vector4(0, 0, 0, 0) },                               // patch centre, radius, enabled
};
Object.assign(oceanU, uwDyeU);
injectShader(ocean.material, 'ocean-frag-decl', /* glsl */`
#define UW_DYE_N ${UW_DYE_N}
uniform vec4 uDye[UW_DYE_N];
uniform float uDyeS[UW_DYE_N];
uniform vec4 uDyeAx, uDyeBox;
float uwDyeConc(vec2 xz) {
  if (uDyeBox.w < 0.5 || distance(xz, uDyeBox.xy) > uDyeBox.z) return 0.0;
  vec2 ax = uDyeAx.xy, ay = vec2(-ax.y, ax.x);
  float c = 0.0;
  for (int i = 0; i < UW_DYE_N; i++) {
    vec4 d = uDye[i];
    if (d.w <= 0.0) continue;
    vec2 r = xz - d.xy;
    float a = dot(r, ax) / uDyeS[i], b = dot(r, ay) / d.z;
    c += d.w / (6.2831853 * uDyeS[i] * d.z) * exp(-0.5 * (a * a + b * b));
  }
  return c;
}
// coverage of the dye: torn into filaments and windrows by the near-surface turbulence
float uwDyeCover(vec2 xz, out float conc) {
  conc = uwDyeConc(xz);
  if (conc <= 1e-3) return 0.0;
  float t = uDyeAx.z;
  float n1 = texture(uFoamTex, xz * 0.071 + t * vec2(0.0011, 0.0007)).b;
  float n2 = texture(uFoamTex, mat2(0.8, -0.6, 0.6, 0.8) * xz * 0.19 - t * vec2(0.0008, 0.0013)).b;
  float tear = clamp(1.9 * n1 * (0.35 + 1.3 * n2) - 0.1, 0.0, 2.2);
  float x = conc * tear;
  return 1.0 - exp(-x * (0.3 + 0.5 * x));     // fairly sharp, ragged edges
}
float uwDyeCover(vec2 xz) { float c; return uwDyeCover(xz, c); }
`);
injectShader(ocean.material, 'ocean-frag-combine', /* glsl */`
  {
    // fluorescein just below the surface: absorbs blue, fluoresces green; concentrated cores turn yellow-orange
    float uwConc;
    float uwDy = uwDyeCover(xz, uwConc) * (0.75 + 0.5 * smoothstep(0.3, 0.8, streak));
    if (uwDy > 0.0) {
      vec3 uwAlb = mix(vec3(0.075, 0.36, 0.022), vec3(0.30, 0.34, 0.012), smoothstep(25.0, 160.0, uwConc));
      vec3 uwDyeBody = uwAlb * (skyHemi * 1.6 + sunE * max(Ld.y, 0.0) * 0.8);
      col += (uwDyeBody - body) * clamp(uwDy, 0.0, 1.0) * (1.0 - Fr) * (1.0 - foamMask * 0.95);
    }
  }
  //`);
const uwDye = { puffs: [], t0: null, acc: 0 };
function uwDyeRelease() { uwDye.puffs.length = 0; uwDye.t0 = simTime; uwDye.acc = 1e9; }
function uwUpdateDye(dt) {
  const box = uwDyeU.uDyeBox.value;
  uwDyeU.uDyeAx.value.z = simTime % 10000;
  if (!params.uwDye || !people.length) { box.w = 0; return; }
  if (uwDye.t0 === null) uwDyeRelease();
  const P = people[0], age = simTime - uwDye.t0, U = params.windSpeed;
  // the packet empties over ~25 minutes, strongly at first
  uwDye.acc += dt;
  if (age < 1500 && uwDye.acc >= 3) {
    uwDye.acc = 0;
    uwDye.puffs.push({ x: P.pos.x + rand(-0.2, 0.2), z: P.pos.z + rand(-0.2, 0.2), s2: 0.12, m: age < 20 ? 70 : 40 * (1 - age / 2000), age: 0 });
  }
  if (dt > 0) {
    // dye lives in the top few centimetres: surface current plus a little extra wind drift
    const vx = sea.current.x + 0.006 * sea.wind.x, vz = sea.current.z + 0.006 * sea.wind.z;
    for (const p of uwDye.puffs) {
      const s = Math.sqrt(p.s2);
      p.s2 += 2 * (0.008 + 0.0045 * s) * (0.4 + U / 10) * dt;   // Okubo-like: diffusivity grows with patch size
      p.x += vx * dt; p.z += vz * dt;
      p.m *= Math.exp(-dt / 2400);                               // photobleaching / mixing below the visible layer
      p.age += dt;
    }
  }
  // keep a fixed number of puffs: merge the adjacent (in age) pair that overlaps most
  while (uwDye.puffs.length > UW_DYE_N) {
    let bi = 0, best = 1e18;
    for (let i = 0; i < uwDye.puffs.length - 2; i++) {
      const a = uwDye.puffs[i], b = uwDye.puffs[i + 1];
      const d = ((a.x - b.x) ** 2 + (a.z - b.z) ** 2) / (a.s2 + b.s2);
      if (d < best) { best = d; bi = i; }
    }
    const a = uwDye.puffs[bi], b = uwDye.puffs[bi + 1], m = a.m + b.m;
    const x = (a.x * a.m + b.x * b.m) / m, z = (a.z * a.m + b.z * b.m) / m;
    const s2 = (a.m * (a.s2 + 0.5 * ((a.x - x) ** 2 + (a.z - z) ** 2)) + b.m * (b.s2 + 0.5 * ((b.x - x) ** 2 + (b.z - z) ** 2))) / m;
    uwDye.puffs.splice(bi, 2, { x, z, s2, m, age: (a.age * a.m + b.age * b.m) / m });
  }
  // upload; puffs are stretched along the wind by the vertical shear of the drift current
  const ax = sea.toward[0], az = sea.toward[1];
  uwDyeU.uDyeAx.value.x = ax; uwDyeU.uDyeAx.value.y = az;
  let cx = 0, cz = 0, mt = 0, rad = 0;
  uwDye.puffs.forEach(p => { cx += p.x * p.m; cz += p.z * p.m; mt += p.m; });
  if (mt > 0) { cx /= mt; cz /= mt; }
  for (let i = 0; i < UW_DYE_N; i++) {
    const p = uwDye.puffs[i];
    if (!p) { uwDyeU.uDye.value[i].set(0, 0, 1, 0); uwDyeU.uDyeS.value[i] = 1; continue; }
    const s = Math.sqrt(p.s2), sa = s * (1 + 0.9 * p.age / (p.age + 240) * (U / 8));
    uwDyeU.uDye.value[i].set(p.x, p.z, s, p.m);
    uwDyeU.uDyeS.value[i] = sa;
    rad = Math.max(rad, Math.hypot(p.x - cx, p.z - cz) + 3.5 * sa);
  }
  box.set(cx, cz, rad, uwDye.puffs.length ? 1 : 0);
}

// ---- placement, per-frame physics and visuals
let uwGearPhys = 0, uwGearT = 0;
function uwRegroupGear() {
  const c = groupCentroid(new THREE.Vector3()), [wx, wz] = sea.toward;
  const at = (along, across) => [c.x + wx * along - wz * across, c.z + wz * along + wx * across];
  uwPlaceBody(uwBuoy, ...at(-3.5, 2.5));
  uwPlaceBody(uwRaft, ...at(9, -4));
  uwPlaceBody(uwDebris[0], ...at(-7, -5));
  uwPlaceBody(uwDebris[1], ...at(2, 8));
  uwPlaceBody(uwDebris[2], ...at(-11, 4));
  uwVessel.theta = Math.atan2(lightDir.z, lightDir.x) + 0.32;
}
uwRegroupGear();
const uwFloaters = [uwBuoy, uwRaft, ...uwDebris];
function uwGearShown(B) { return B === uwBuoy ? params.uwBuoy : B === uwRaft ? params.uwRaft : params.uwDebris; }
// shadow columns for the underwater light shafts (uw_underwater.js)
function uwGearOccluders(out) {
  if (params.uwRaft) out.push([uwRaft.pos.x, uwRaft.pos.z, 0.95, 0.92]);
  if (params.uwBuoy) out.push([uwBuoy.pos.x, uwBuoy.pos.z, 0.38, 0.5]);
  if (params.uwDebris) uwDebris.forEach(d => out.push([d.pos.x, d.pos.z, 0.28, 0.55]));
}
let uwGearFrames = 0, uwGearPlaced = false;
HOOKS.update.push(dt => {
  uwGearFrames++;
  // the people load asynchronously: place the gear around them (and the vessel towards the sun) once they exist
  if (!uwGearPlaced && people.length) { uwRegroupGear(); uwGearPlaced = true; uwGearFrames = 0; }
  // read this frame's probe samples (queried last frame)
  for (const B of uwFloaters) B.ps = B.probes.map((_, k) => probe(B.probeBase + k));
  uwGearPhys += dt;
  let steps = 0, tSub = simTime - uwGearPhys;
  const live = uwGearFrames > 2;
  while (uwGearPhys >= PHYS_DT && steps < 12) {
    if (live) for (const B of uwFloaters) if (uwGearShown(B)) uwStepBody(B, PHYS_DT, tSub);
    uwGearPhys -= PHYS_DT; tSub += PHYS_DT; steps++;
  }
  if (steps === 12) uwGearPhys = 0;
  for (const B of uwFloaters) { B.root.visible = uwGearShown(B); uwPostBody(B); }

  // lifebuoy SOLAS light (~60 flashes/min) and raft canopy light (steady at night)
  uwGearT += dt;
  const flash = ((uwGearT * 1.0 + uwBuoy.strobePh) % 1) < 0.1 ? 1 : 0;
  uwBuoy.bulbMat.emissiveIntensity = flash * (2 + 300 * nightF);
  uwBuoy.glow.material.opacity = flash * (0.12 + 0.88 * nightF);
  uwRaft.lampM.emissiveIntensity = 1 + 150 * nightF;
  uwRaft.glow.material.opacity = 0.6 * nightF;
  // retroreflective tape: bright when the camera is close to the direction of the light source
  for (const B of [uwBuoy, uwRaft]) {
    _ug.v.copy(camera.position).sub(B.pos).normalize();
    const E = oceanU.uSunE.value, lum = 0.2126 * E.x + 0.7152 * E.y + 0.0722 * E.z;
    B.tape.emissiveIntensity = lum * 0.4 * Math.pow(Math.max(_ug.v.dot(lightDir), 0), 60);
  }

  // rescue vessel: works a slow arc ~1.5 km out, heave / pitch / roll low-passed from three probes
  const V = uwVessel;
  V.root.visible = params.uwVessel;
  if (params.uwVessel) {
    const c = groupCentroid(_ug.r), Rv = 1500;
    V.theta += 2.2 / Rv * dt;
    V.pos.set(c.x + Rv * Math.cos(V.theta), 0, c.z + Rv * Math.sin(V.theta));
    V.yaw = -V.theta;                                     // heading along the arc (bow towards increasing theta)
    const fx = -Math.sin(V.theta), fz = Math.cos(V.theta);      // bow direction
    const ps = [0, 1, 2].map(k => probe(V.probeBase + k));
    if (live) {
      const k = 1 - Math.exp(-dt / 2.5);
      const hb = ps[0].h, hs = ps[1].h, hp = ps[2].h;
      V.heave += ((hb + hs) * 0.5 * 0.6 - V.heave) * k;
      V.pitch += (Math.atan2(hb - hs, V.L * 0.8) * 0.6 - V.pitch) * k;
      V.roll += (-Math.atan2(hp - (hb + hs) * 0.5, V.Bm) * 0.8 - V.roll) * k;
    }
    const roll = V.roll + 0.035 * Math.sin(simTime * 2 * Math.PI / 7.5);
    V.root.position.set(V.pos.x, V.heave, V.pos.z);
    V.root.rotation.set(0, Math.atan2(-fz, fx), 0);
    V.hullG.rotation.set(roll, 0, V.pitch);
    V.radar.rotation.y = simTime * 2.5;
    V.lights.forEach(l => { l.material.opacity = nightF * 0.9; });
    const q = [[fx * V.L * 0.4, fz * V.L * 0.4], [-fx * V.L * 0.4, -fz * V.L * 0.4], [-fz * V.Bm * 0.5, fx * V.Bm * 0.5]];
    q.forEach(([dx, dz], k) => { queryData[(V.probeBase + k) * 4] = V.pos.x + dx; queryData[(V.probeBase + k) * 4 + 1] = V.pos.z + dz; });
  }
  uwUpdateDye(dt);
});

// ---- GUI
uwGui.add(params, 'uwBuoy').name('Lifebuoy');
uwGui.add(params, 'uwRaft').name('Life raft');
uwGui.add(params, 'uwDebris').name('Debris');
uwGui.add(params, 'uwVessel').name('Rescue vessel (horizon)');
uwGui.add(params, 'uwDye').name('Sea-marker dye');
uwGui.add({ f: uwDyeRelease }, 'f').name('Release new dye');
uwGui.add({ f: uwRegroupGear }, 'f').name('Regroup gear near people');
