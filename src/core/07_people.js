// ============================================================================
// People overboard
// ============================================================================
// Floating posture in a lifejacket: body inclined ~50 deg back, head up, arms sculling near the surface,
// legs hanging down-forward. Directions are in the body frame at equilibrium (+z = facing, y = up).
const LEAN = 0.85;
const AIMS = {
  Head: [0, 0.94, -0.34], LeftArm: [0.82, -0.18, 0.38], LeftForeArm: [0.5, -0.12, 0.86], LeftHand: [0.45, -0.2, 0.87],
  LeftUpLeg: [0.13, -0.8, 0.58], LeftLeg: [0.06, -0.97, 0.22], LeftFoot: [0.0, -0.45, 0.89],
};
const MODEL_DEFS = [
  { file: 'readyplayer.me.glb', prefix: '', height: 1.8, mass: [80, 96], wet: { Wolf3D_Skin: 0.85, Wolf3D_Body: 0.85 } },
  // single material whose hair cards rely on the texture alpha; the asset also ships with metalness 0.5
  { file: 'Michelle.glb', prefix: 'mixamorig', height: 1.68, mass: [62, 76], wet: {}, alphaTest: 0.5, metalness: 0 },
];
const PFD_COLORS = [0xff5a00, 0xffc400, 0xff3d12];
const templates = [];
let modelsReady = false;

function capFraction(depth, r) {           // submerged fraction of a sphere whose centre is 'depth' below the surface
  const h = clamp(depth + r, 0, 2 * r);
  return h * h * (3 * r - h) / (4 * r ** 3);
}

// --- forward kinematics with aim targets, in the body frame
const _q = new THREE.Quaternion(), _q2 = new THREE.Quaternion(), _v = new THREE.Vector3(), _v2 = new THREE.Vector3();
const IDQ = new THREE.Quaternion();
function buildRig(model, prefix) {
  model.updateMatrixWorld(true);
  const bones = [];
  model.traverse(o => { if (o.isBone) bones.push(o); });
  const byName = n => bones.find(b => b.name === prefix + n);
  const info = new Map();
  for (const b of bones) {
    info.set(b, {
      restWQ: b.getWorldQuaternion(new THREE.Quaternion()), restWP: b.getWorldPosition(new THREE.Vector3()),
      parentBone: b.parent && b.parent.isBone ? b.parent : null, parentWQ: b.parent.getWorldQuaternion(new THREE.Quaternion()),
      restDir: null, accum: new THREE.Quaternion(), wq: new THREE.Quaternion(),
    });
  }
  const rig = { bones, info, byName, hips: byName('Hips'), aims: new Map() };
  const chain = (n, child) => {
    const b = byName(n), c = byName(child);
    if (b && c) info.get(b).restDir = info.get(c).restWP.clone().sub(info.get(b).restWP).normalize();
    return b;
  };
  for (const s of ['Left', 'Right']) {
    chain(s + 'Arm', s + 'ForeArm'); chain(s + 'ForeArm', s + 'Hand'); chain(s + 'Hand', s + 'HandMiddle1');
    chain(s + 'UpLeg', s + 'Leg'); chain(s + 'Leg', s + 'Foot'); chain(s + 'Foot', s + 'ToeBase');
  }
  chain('Head', 'HeadTop_End');
  return rig;
}
const leanQ = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -LEAN);
function solvePose(rig) {
  for (const b of rig.bones) {
    const I = rig.info.get(b);
    const pa = I.parentBone ? rig.info.get(I.parentBone).accum : IDQ;
    I.accum.copy(pa);
    const target = rig.aims.get(b);
    if (b === rig.hips) I.accum.premultiply(leanQ);
    else if (target && I.restDir) {
      _v.copy(I.restDir).applyQuaternion(pa);
      I.accum.premultiply(_q.setFromUnitVectors(_v, target));
    }
    I.wq.copy(I.accum).multiply(I.restWQ);
    const pwq = I.parentBone ? rig.info.get(I.parentBone).wq : I.parentWQ;
    b.quaternion.copy(pwq).invert().multiply(I.wq);
  }
}
function setAims(P, t) {
  // gentle sculling: arms sweep a little around the vertical, forearms lag behind
  for (const [side, sx] of [['Left', 1], ['Right', -1]]) {
    for (const key of ['Arm', 'ForeArm', 'Hand', 'UpLeg', 'Leg', 'Foot']) {
      const b = P.rig.byName(side + key);
      if (!b) continue;
      const a = AIMS['Left' + key];
      const v = P.aimVecs.get(b) || new THREE.Vector3();
      v.set(a[0] * sx, a[1], a[2]);
      if (key === 'Arm' || key === 'ForeArm' || key === 'Hand') {
        const ph = t * P.scullW + P.scullPh + (key === 'Arm' ? 0 : 0.9);
        v.applyAxisAngle(_v2.set(0, 1, 0), sx * 0.22 * Math.sin(ph) * (key === 'Arm' ? 1 : 1.4));
        v.y += 0.06 * Math.sin(ph * 2);
      } else if (key === 'UpLeg' || key === 'Leg') {
        v.z += 0.08 * Math.sin(t * 0.7 + P.scullPh + (sx > 0 ? 0 : 2.1));   // slow treading
      }
      v.normalize();
      P.aimVecs.set(b, v);
      P.rig.aims.set(b, v);
    }
  }
  const head = P.rig.byName('Head');
  if (head) P.rig.aims.set(head, new THREE.Vector3(...AIMS.Head).applyAxisAngle(_v2.set(0, 1, 0), P.headYaw).normalize());
}

// --- inflated lifejacket built around the posed torso
function buildJacket(rig, color) {
  const pos = n => { const b = rig.byName(n); return b ? b.getWorldPosition(new THREE.Vector3()) : null; };
  const neck = pos('Neck'), chest = pos('Spine2'), belly = pos('Spine1') || pos('Spine');
  const ls = pos('LeftArm'), rs = pos('RightArm');
  const l = ls.clone().sub(rs).normalize();
  const width = ls.distanceTo(rs);
  const up = neck.clone().sub(belly).normalize();
  const f = new THREE.Vector3().crossVectors(l, up).normalize();     // chest-forward
  const P = (base, a, b, c) => base.clone().addScaledVector(l, a).addScaledVector(f, b).addScaledVector(up, c);
  const group = new THREE.Group();
  const pfd = new THREE.MeshPhysicalMaterial({ color, roughness: 0.55, clearcoat: 0.6, clearcoatRoughness: 0.35 });
  const tape = new THREE.MeshStandardMaterial({ color: 0xd9dee2, roughness: 0.3, metalness: 0.6 });
  const strap = new THREE.MeshStandardMaterial({ color: 0x111316, roughness: 0.7 });
  const lobeR = 0.07, collarR = 0.06, sw = width * 0.24;
  // collar: tall horseshoe behind the head (it keeps the face up), running into the two chest lobes
  const collarPts = [
    P(chest, sw, 0.12, 0.1), P(neck, sw * 1.05, 0.07, -0.02), P(neck, sw * 0.95, 0.0, 0.04),
    P(neck, sw * 0.55, -0.08, 0.07), P(neck, 0, -0.1, 0.08), P(neck, -sw * 0.55, -0.08, 0.07),
    P(neck, -sw * 0.95, 0.0, 0.04), P(neck, -sw * 1.05, 0.07, -0.02), P(chest, -sw, 0.12, 0.1),
  ];
  const collar = new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(collarPts), 64, collarR, 16), pfd);
  group.add(collar);
  for (const sgn of [1, -1]) {
    // wide, flattened bladder down each side of the chest
    const top = P(chest, sgn * sw, 0.12, 0.1), bot = P(chest, sgn * sw * 0.85, 0.13, -0.2);
    const axis = top.clone().sub(bot).normalize();
    const side = new THREE.Vector3().crossVectors(axis, f).normalize();
    const lobe = new THREE.Mesh(new THREE.CapsuleGeometry(lobeR, top.distanceTo(bot), 8, 20), pfd);
    lobe.position.copy(top).add(bot).multiplyScalar(0.5);
    lobe.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(side, axis, new THREE.Vector3().crossVectors(side, axis)));
    lobe.scale.set(1.45, 1, 0.72);
    group.add(lobe);
    const strip = new THREE.Mesh(new THREE.BoxGeometry(0.11, 0.024, 0.01), tape);
    strip.position.copy(P(chest, sgn * sw, 0.182, -0.04));
    strip.quaternion.copy(lobe.quaternion);
    group.add(strip);
  }
  // waist belt
  const beltC = P(belly, 0, 0.0, -0.06);
  const belt = new THREE.Mesh(new THREE.TorusGeometry(1, 0.012, 6, 40), strap);
  belt.position.copy(beltC);
  const m = new THREE.Matrix4().makeBasis(l, f, up);
  belt.quaternion.setFromRotationMatrix(m);
  belt.scale.set(width * 0.5, 0.15, 1);
  group.add(belt);
  // SOLAS light on the left lobe
  const bulbMat = new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0xffffff, emissiveIntensity: 0 });
  const bulb = new THREE.Mesh(new THREE.SphereGeometry(0.018, 10, 8), bulbMat);
  bulb.position.copy(P(chest, sw, 0.17, 0.15));
  group.add(bulb);
  return { group, bulb, bulbMat, materials: [pfd, tape, strap, bulbMat] };
}

// --- wet look: darker, glossier below and just above the local waterline
// thermal classes (LWIR_MAT_*): 2 face, 3 hair, 4 hands, 5 wet clothing, 6 lifejacket; 7 = mixed texture (face near the head)
function makeWet(mat, wetU, dark, cls) {
  mat.onBeforeCompile = sh => {
    sh.uniforms.uWaterPlane = wetU.plane;
    sh.uniforms.uWetBase = wetU.base;
    sh.uniforms.uWetDark = { value: dark };
    sh.uniforms.uHeadPos = wetU.head;
    sh.uniforms.uMatClass = { value: cls };
    Object.assign(sh.uniforms, THERM);
    sh.vertexShader = 'varying vec3 vWetPos;\n' + sh.vertexShader.replace('#include <project_vertex>',
      '#include <project_vertex>\n  vWetPos = (modelMatrix * vec4(transformed, 1.0)).xyz;');
    sh.fragmentShader = 'varying vec3 vWetPos;\nuniform vec4 uWaterPlane;\nuniform float uWetBase;\nuniform float uWetDark;\n' +
      'uniform vec3 uHeadPos;\nuniform float uMatClass;\n' + THERMAL_UNIFORMS_GLSL + THERMAL_GLSL + sh.fragmentShader
      .replace('#include <dithering_fragment>', `#include <dithering_fragment>
  if (uThermal > 0.5) {
    float dHead = distance(vWetPos, uHeadPos);
    int cls = int(uMatClass + 0.5);
    if (cls == 7) cls = dHead < 0.13 ? LWIR_MAT_FACE : LWIR_MAT_WET_CLOTH;   // single-texture model
    float Tsurf = personSurfaceTemp(cls, wetH, dHead, uTwater, uTair, uWindSpd, uSunIrr);
    float em = materialEmissivity(cls);
    vec3 nW = normalize((vec4(normal, 0.0) * viewMatrix).xyz);
    vec3 rW = reflect(normalize(vWetPos - cameraPosition), nW);
    float Lsky = skyRadianceLWIR(max(rW.y, 0.0), uTair, uRH, uCloudFrac, uTcloud);
    float tau = pathTransmittanceLWIR(length(vViewPosition), uRH, uTair);
    gl_FragColor = vec4((em * planckBand(Tsurf) + (1.0 - em) * Lsky) * tau + planckBand(uTair) * (1.0 - tau), 0.0, 0.0, 1.0);
  }`)
      .replace('#include <map_fragment>', `#include <map_fragment>
  float wetH = dot(uWaterPlane.xyz, vWetPos) + uWaterPlane.w;
  float wetN = fract(sin(dot(floor(vWetPos.xz * 30.0 + vWetPos.y * 17.0), vec2(12.9898, 78.233))) * 43758.5453);
  float wetness = max(uWetBase, 1.0 - smoothstep(0.02, 0.14 + 0.1 * wetN, wetH));
  diffuseColor.rgb *= mix(1.0, uWetDark, wetness);`)
      .replace('#include <roughnessmap_fragment>', `#include <roughnessmap_fragment>
  roughnessFactor = mix(roughnessFactor, roughnessFactor * 0.3 + 0.05, wetness);`);
    for (const h of HOOKS.personShader) h(sh, mat, cls);   // feature modules: extra person-material code
  };
  mat.customProgramCacheKey = () => 'wet-thermal';
}

function prepareTemplate(gltf, def) {
  gltf.scene.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(gltf.scene);
  return { scene: gltf.scene, def, scale: def.height / (box.max.y - box.min.y) };
}
const GLOW_TEX = (() => {
  const c = document.createElement('canvas'); c.width = c.height = 64;
  const g = c.getContext('2d'), gr = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  gr.addColorStop(0, 'rgba(255,255,255,1)'); gr.addColorStop(0.2, 'rgba(255,255,255,0.6)'); gr.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = gr; g.fillRect(0, 0, 64, 64);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; return t;
})();
const PROBE_GEO = new THREE.SphereGeometry(0.04, 10, 8);

function createPerson(tpl) {
  const def = tpl.def;
  const root = new THREE.Group();          // physics body frame (origin = centre of mass)
  const poseRoot = new THREE.Group();      // body frame offset so the COM sits at the origin
  root.add(poseRoot);
  const model = SkeletonUtils.clone(tpl.scene);
  const holder = new THREE.Group();
  holder.scale.setScalar(tpl.scale);
  holder.add(model);
  poseRoot.add(holder);
  const wetU = { plane: { value: new THREE.Vector4(0, 1, 0, 0) }, base: { value: 0.45 }, head: { value: new THREE.Vector3() } };
  const mats = [];
  model.traverse(o => {
    if (!o.isMesh) return;
    if (o.name === 'Wolf3D_Headwear') { o.visible = false; return; }   // a fedora does not stay on in the water
    o.frustumCulled = false;
    o.layers.enable(PEOPLE_LAYER);
    o.material = [].concat(o.material).map(m => {
      const c = m.clone();
      const dark = def.wet[m.name] ?? (/Skin|Body|Ch03/.test(m.name) ? 0.8 : 0.5);
      if (/Outfit_Top/.test(m.name)) c.color.multiply(new THREE.Color(pick([0xffffff, 0x8899cc, 0x666666, 0xcc7777, 0x77aa88])));
      if (def.alphaTest !== undefined) c.alphaTest = def.alphaTest;
      if (def.metalness !== undefined) c.metalness = def.metalness;
      const cls = /Beard|Hair/.test(m.name) ? 3 : /Outfit/.test(m.name) ? 5 : /Wolf3D_Body/.test(m.name) ? 4 : /Skin|Eye|Teeth/.test(m.name) ? 2 : 7;
      makeWet(c, wetU, dark, cls);
      mats.push(c);
      return c;
    })[0];
  });
  const P = {
    root, poseRoot, model, mats, wetU, rig: buildRig(holder, def.prefix), aimVecs: new Map(),
    scullW: rand(1.6, 2.4), scullPh: Math.random() * 6.28, headYaw: rand(-0.4, 0.4), strobePh: Math.random(),
    mass: rand(def.mass[0], def.mass[1]),
    pos: new THREE.Vector3(), vel: new THREE.Vector3(), w: new THREE.Vector3(),
    q: new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.random() * Math.PI * 2),
  };
  P.headBones = [P.rig.byName('Head'), P.rig.byName('HeadTop_End') || P.rig.byName('Head')];
  setAims(P, 0);
  solvePose(P.rig);
  holder.updateMatrixWorld(true);

  // lifejacket
  const jk = buildJacket(P.rig, pick(PFD_COLORS));
  jk.group.traverse(o => { if (o.isMesh) o.layers.enable(PEOPLE_LAYER); });
  jk.materials.forEach((m, k) => { makeWet(m, wetU, 0.92, k === 2 ? 5 : 6); mats.push(m); });
  poseRoot.add(jk.group);
  P.bulbMat = jk.bulbMat;
  P.glow = new THREE.Sprite(new THREE.SpriteMaterial({ map: GLOW_TEX, color: 0xfff6e0, transparent: true, opacity: 0, depthWrite: false, blending: THREE.AdditiveBlending }));
  P.glow.position.copy(jk.bulb.position); P.glow.scale.setScalar(0.35);
  poseRoot.add(P.glow);
  mats.push(P.glow.material);

  // buoyancy probes from the posed skeleton (volumes scaled with body mass; ~150 N inflated jacket at the chest/collar)
  const bp = n => { const b = P.rig.byName(n); return b ? b.getWorldPosition(new THREE.Vector3()) : null; };
  const mid = (a, b) => bp(a).add(bp(b)).multiplyScalar(0.5);
  const headTop = bp('HeadTop_End') || bp('Head').add(new THREE.Vector3(0, 0.18, 0));
  const vs = P.mass / 88;
  const list = [
    [bp('Head').lerp(headTop, 0.45), 0.105, 0.0055 * vs], [bp('Neck'), 0.12, 0.011], [bp('Spine2'), 0.17, 0.030 * vs],
    [bp('Spine'), 0.15, 0.020 * vs], [bp('Hips'), 0.15, 0.017 * vs],
    [mid('LeftUpLeg', 'LeftLeg'), 0.085, 0.0055 * vs], [mid('RightUpLeg', 'RightLeg'), 0.085, 0.0055 * vs],
    [mid('LeftLeg', 'LeftFoot'), 0.06, 0.0035 * vs], [mid('RightLeg', 'RightFoot'), 0.06, 0.0035 * vs],
    [bp('LeftForeArm'), 0.045, 0.0012 * vs], [bp('RightForeArm'), 0.045, 0.0012 * vs],
  ];
  // equilibrium: water level where buoyancy = weight, then put the COM a little below the centre of buoyancy
  const buoy = y => list.reduce((s, [p, r, V]) => s + RHO * V * capFraction(y - p.y, r), 0);
  let lo = -2, hi = 2;
  for (let i = 0; i < 60; i++) { const m = (lo + hi) / 2; if (buoy(m) < P.mass) lo = m; else hi = m; }
  const yW = (lo + hi) / 2, cob = new THREE.Vector3();
  let vsum = 0;
  for (const [p, r, V] of list) { const v = V * capFraction(yW - p.y, r); cob.addScaledVector(p, v); vsum += v; }
  cob.divideScalar(vsum);
  const com = cob.clone().add(new THREE.Vector3(0, -0.09, 0));
  poseRoot.position.copy(com).negate();
  P.probes = list.map(([p, r, V]) => ({ local: p.clone().sub(com), r, V, area: Math.PI * r * r }));
  P.Vtot = list.reduce((s, l) => s + l[2], 0);
  P.depth = new Float64Array(P.probes.length); P.frac = new Float64Array(P.probes.length); P.sub = 0;
  P.ix = P.mass * 0.24; P.iy = P.mass * 0.05;       // rough principal inertias (kg m^2)
  P.probeGroup = new THREE.Group(); P.probeGroup.visible = params.showProbes; root.add(P.probeGroup);
  P.probeMats = P.probes.map(pr => {
    const m = new THREE.MeshBasicMaterial({ color: 0xff3030, depthTest: false, transparent: true });
    const s = new THREE.Mesh(PROBE_GEO, m); s.position.copy(pr.local); s.renderOrder = 10; P.probeGroup.add(s);
    mats.push(m); return m;
  });
  return P;
}

const people = [];
function groupCentroid(out) {
  out.set(0, 0, 0);
  if (!people.length) return out;
  people.forEach(P => out.add(P.pos));
  return out.divideScalar(people.length).setY(0);
}
// a spot around (cx, cz) at least 2.2 m from everyone already placed (ring widens if it is crowded)
function freeSpot(cx, cz, placed, out) {
  for (let k = 0; k < 60; k++) {
    const a = Math.random() * Math.PI * 2, r = placed.length ? rand(2.5, 10 + k * 0.2) : 0;
    out.set(cx + r * Math.cos(a), -0.4, cz + r * Math.sin(a));
    if (placed.every(q => Math.hypot(q.x - out.x, q.z - out.z) > 2.2)) break;
  }
  return out;
}
function addPerson() {
  if (!templates.length) return;
  const P = createPerson(templates[people.length % templates.length]);
  const c = groupCentroid(new THREE.Vector3());
  freeSpot(c.x, c.z, people.map(Q => Q.pos), P.pos);
  scene.add(P.root);
  people.push(P);
}
function removePerson() {
  const P = people.pop();
  scene.remove(P.root);
  P.mats.forEach(m => m.dispose());
}
function setPeopleCount(n) {
  n = clamp(Math.round(n), 0, MAX_PEOPLE);
  while (people.length < n && templates.length) addPerson();
  while (people.length > n) removePerson();
}
function respawn() {
  const placed = [];
  people.forEach(P => {
    freeSpot(0, 0, placed, P.pos); P.vel.set(0, 0, 0); P.w.set(0, 0, 0);
    placed.push(P.pos);
  });
  controls.target.set(0, 0, 0);
  placeCamera();
}

// --- rigid-body buoyancy using the GPU surface probes (sampled once per frame, extrapolated between substeps)
const _r = new THREE.Vector3(), _pv = new THREE.Vector3(), _F = new THREE.Vector3(), _T = new THREE.Vector3();
const _n = new THREE.Vector3(), _up = new THREE.Vector3(), _qi = new THREE.Quaternion(), _dq = new THREE.Quaternion();
const CD = 1.0, C_LIN = 45, C_ANG = 10, K_ALIGN = 30, ADDED = 0.45;
function writeQueries() {
  people.forEach((P, pi) => {
    const base = pi * PROBES_PER;
    P.probes.forEach((pr, i) => {
      _r.copy(pr.local).applyQuaternion(P.q).add(P.pos);
      queryData[(base + i) * 4] = _r.x; queryData[(base + i) * 4 + 1] = _r.z;
      pr.sx0 = _r.x; pr.sz0 = _r.z;
    });
    queryData[(base + PROBES_PER - 1) * 4] = P.pos.x; queryData[(base + PROBES_PER - 1) * 4 + 1] = P.pos.z;
  });
  queryData[CAM_PROBE * 4] = camera.position.x; queryData[CAM_PROBE * 4 + 1] = camera.position.z;
}
function stepPerson(P, pi, dt, t) {
  const m = P.mass, base = pi * PROBES_PER, kp = sea.kp, dts = t - probeTime;
  _F.set(0, -m * G, 0); _T.set(0, 0, 0);
  let Vsub = 0;
  P.probes.forEach((pr, i) => {
    const s = probe(base + i);
    _r.copy(pr.local).applyQuaternion(P.q);
    const px = P.pos.x + _r.x, py = P.pos.y + _r.y, pz = P.pos.z + _r.z;
    const h = s.h + s.vy * dts + s.sx * (px - pr.sx0) + s.sz * (pz - pr.sz0);
    const depth = h - py;
    P.depth[i] = Math.max(0, depth);
    const frac = capFraction(depth, pr.r);
    P.frac[i] = frac;
    const vol = pr.V * frac;
    Vsub += vol;
    _pv.crossVectors(P.w, _r).add(P.vel);
    let fx = 0, fy = RHO * G * vol, fz = 0;
    if (frac > 0) {
      const dec = Math.exp(-kp * P.depth[i]);
      const rx = _pv.x - (s.vx * dec + sea.current.x), ry = _pv.y - s.vy * dec, rz = _pv.z - (s.vz * dec + sea.current.z);
      const c = (0.5 * RHO * CD * pr.area * Math.hypot(rx, ry, rz) + C_LIN) * frac;
      fx -= c * rx; fy -= c * ry; fz -= c * rz;
    }
    if (frac < 1) {
      const ax = sea.wind.x - _pv.x, az = sea.wind.z - _pv.z;
      const c = 0.5 * RHO_AIR * 1.1 * pr.area * (1 - frac) * Math.hypot(ax, az);
      fx += c * ax; fz += c * az;
    }
    _F.x += fx; _F.y += fy; _F.z += fz;
    _T.x += _r.y * fz - _r.z * fy; _T.y += _r.z * fx - _r.x * fz; _T.z += _r.x * fy - _r.y * fx;
  });
  P.sub = Vsub / P.Vtot;
  // wave-slope pressure turns the body with the local surface normal
  const sc = probe(base + PROBES_PER - 1);
  _up.set(0, 1, 0).applyQuaternion(P.q);
  _n.set(-sc.sx, 1, -sc.sz).normalize();
  _T.addScaledVector(_n.crossVectors(_up, _n), K_ALIGN * P.sub);
  _T.addScaledVector(P.w, -C_ANG);
  _T.y += 0.8 * Math.sin(t * 0.13 + P.scullPh);
  // soft horizontal contact between people (bodies and jackets ~1.4 m apart at the closest)
  for (const Q of people) {
    if (Q === P) continue;
    const dx = P.pos.x - Q.pos.x, dz = P.pos.z - Q.pos.z, d = Math.hypot(dx, dz);
    if (d < 1.4 && d > 1e-4) { const f = (1.4 - d) * 600 / d; _F.x += dx * f; _F.z += dz * f; }
  }
  const madd = ADDED * RHO * Vsub, s = 1 + madd / m;
  P.vel.addScaledVector(_F, dt / (m + madd));
  if (P.vel.lengthSq() > 400) P.vel.setLength(20);
  P.pos.addScaledVector(P.vel, dt);
  _qi.copy(P.q).invert();
  _T.applyQuaternion(_qi);
  _pv.copy(P.w).applyQuaternion(_qi);
  _pv.x += _T.x / (P.ix * s) * dt; _pv.y += _T.y / (P.iy * s) * dt; _pv.z += _T.z / (P.ix * s) * dt;
  P.w.copy(_pv.applyQuaternion(P.q));
  if (P.w.lengthSq() > 100) P.w.setLength(10);
  _dq.set(P.w.x * dt * 0.5, P.w.y * dt * 0.5, P.w.z * dt * 0.5, 0).multiply(P.q);
  P.q.set(P.q.x + _dq.x, P.q.y + _dq.y, P.q.z + _dq.z, P.q.w + _dq.w).normalize();
}

