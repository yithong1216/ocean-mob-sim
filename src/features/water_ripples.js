// ============================================================================
// Interactive water: GPU ripple / wake height field around the people  (feature: water_)
// ============================================================================
// A linear, dispersive surface-wave simulation on a square grid that follows the group. The grid lives in the
// frame of the drifting surface water (Lagrangian FFT coordinates minus the integrated surface current), so the
// ripples ride on the swell and drift with the water; the grid only moves by whole texels (no numerical
// diffusion). Gravity-wave dispersion (omega^2 = g |k|) comes from a sparse multi-scale ring kernel fitted
// offline (non-negative weights => unconditionally positive semi-definite, phase speed within ~12 % of
// sqrt(g/k) from 2 to 64 texel wavelengths, isotropic to a few %). Sponge borders absorb outgoing waves.
// Forcing: body heave relative to the surface, drift relative to the current (dipole => wake), hands sculling
// near the surface, droplet impacts from the spray system, plus a static waterline ring around each torso.
// Output, packed in ONE RGBA texture: height (m), dh/dx, dh/dz, turbulence -> ocean vertex displacement, extra
// slope (normal + LEAN roughness) and turbulence micro-foam in the ocean shader.

Object.assign(params, {
  waterRipples: true, waterRippleGain: 1.0, waterWake: 1.0, waterMicroFoam: 1.0,
  waterSpray: true, waterSprayAmount: 1.0, waterWhitecapSpray: 1.0, waterSplashes: 1.0,
});

const WATER_RIP_Q = [
  { n: 384, texel: 0.125 },    // 48 m
  { n: 512, texel: 0.1 },      // 51 m
  { n: 768, texel: 0.07 },     // 54 m
  { n: 1024, texel: 0.06 },    // 61 m
];
const WATER_MAXS = 64;         // point sources per step (bodies + hands + slaps)

const WATER_RIP_SIM_FRAG = /* glsl */`
precision highp float;
precision highp int;
${NOISE_GLSL}
#define MAXS ${WATER_MAXS}
#define MAXB ${MAX_PEOPLE}
uniform sampler2D uState;      // r height, g vertical velocity, b turbulence, a static waterline shape
uniform sampler2D uImpact;     // r: downward velocity impulse from droplet impacts (m/s)
uniform vec2 uShift;           // whole-texel grid move since the last step
uniform float uRes, uTexel, uDt, uGrav, uDamp, uTurbTau, uDipole, uChurn;
uniform vec2 uSeed;          // random per step: decorrelates the churn forcing in time
uniform vec4 uSrcA[MAXS];      // texel x, y, radius (texels), object vertical velocity relative to the water (m/s)
uniform vec4 uSrcB[MAXS];      // relative horizontal velocity x, z (m/s), coupling rate (1/s), turbulence rate (1/s)
uniform int uSrcN;
uniform vec4 uBodA[MAXB];      // torso waterline capsule: end a (texels), end b (texels)
uniform vec4 uBodB[MAXB];      // radius (texels), lip height (m), trough depth (m), -
uniform int uBodN;
layout(location = 0) out vec4 fragColor;
ivec2 gShift;
int gRes;
vec4 st(ivec2 c) { return texelFetch(uState, clamp(c + gShift, ivec2(0), ivec2(gRes - 1)), 0); }
float hh(ivec2 c) { return texelFetch(uState, clamp(c + gShift, ivec2(0), ivec2(gRes - 1)), 0).r; }
// the four 90-degree rotations of offset (a, b)
float ring4(ivec2 p, int a, int b) { return hh(p + ivec2(a, b)) + hh(p + ivec2(-b, a)) + hh(p + ivec2(-a, -b)) + hh(p + ivec2(b, -a)); }
void main() {
  gRes = int(uRes);
  gShift = ivec2(uShift);
  ivec2 p = ivec2(gl_FragCoord.xy);
  ivec2 q = p + gShift;
  bool inside = q.x >= 0 && q.y >= 0 && q.x < gRes && q.y < gRes;
  vec4 c = inside ? texelFetch(uState, q, 0) : vec4(0.0);
  vec4 e = st(p + ivec2(1, 0)), w = st(p - ivec2(1, 0)), n = st(p + ivec2(0, 1)), s = st(p - ivec2(0, 1));
  float h0 = c.r, f4 = 4.0 * h0;
  // dispersive operator K ~ |k| (texel units); omega^2 = g K / texel
  float K = 0.35720 * (f4 - e.r - w.r - n.r - s.r)
          + 0.11951 * (f4 - ring4(p, 1, 1))
          + 0.01154 * (f4 - ring4(p, 2, 2))
          + 0.02398 * (f4 - ring4(p, 4, 0))
          + 0.02483 * (f4 - ring4(p, 3, 3))
          + 0.02069 * (f4 - ring4(p, 16, 0))
          + 0.01674 * (f4 - ring4(p, 11, 11));
  vec2 fp = gl_FragCoord.xy;
  vec2 eu = min(fp, vec2(uRes) - fp) / uRes;
  float sponge = 1.0 - smoothstep(0.0, 0.11, min(eu.x, eu.y));
  float beta = uDamp + 7.0 * sponge * sponge;
  float v = c.g, T = c.b;
  float acc = -uGrav * K / uTexel;
  float tIn = 0.0;
  for (int i = 0; i < MAXS; i++) {
    if (i >= uSrcN) break;
    vec4 A = uSrcA[i], B = uSrcB[i];
    vec2 d = (fp - A.xy) / A.z;
    float r2 = dot(d, d);
    if (r2 > 30.0) continue;
    float G = exp(-r2);
    // volume-conserving heave source (pushed down in the footprint, up in the ring around it), the body
    // holding the surface inside its footprint, and a bow/stern dipole for motion through the water
    float F = G - 0.25 * exp(-0.25 * r2);
    acc += B.z * (5.0 * F * A.w - G * v + uDipole * G * dot(B.xy, d));
    tIn += B.w * G;
  }
  // churned water boils: turbulent eddies push the surface about at random (short ripples spread from the
  // wake and from around hands); spatially smooth over ~3 texels, white in time
  acc += uChurn * min(T + tIn * 0.5, 2.0) * gnoise(fp * 0.33 + uSeed);
  v += acc * uDt;
  // tiny grid-scale smoothing (keeps impact splats from leaving 2-texel noise)
  v += 0.004 * (e.g + w.g + n.g + s.g - 4.0 * v);
  v -= texelFetch(uImpact, p, 0).r;
  v *= exp(-beta * uDt);
  float h = (h0 + v * uDt) * exp(-(0.08 + 2.0 * sponge) * uDt);
  T = T * exp(-uDt / uTurbTau) + tIn * uDt;
  T += 0.06 * (e.b + w.b + n.b + s.b - 4.0 * T);
  T = clamp(T * (1.0 - sponge), 0.0, 3.0);
  // static waterline shape around each torso: raised lip at the contact, broad shallow trough beyond it
  float a = 0.0;
  for (int i = 0; i < MAXB; i++) {
    if (i >= uBodN) break;
    vec4 A = uBodA[i], B = uBodB[i];
    vec2 ab = A.zw - A.xy;
    float t = clamp(dot(fp - A.xy, ab) / max(dot(ab, ab), 1e-4), 0.0, 1.0);
    float dd = (length(fp - A.xy - ab * t) - B.x) * uTexel;     // metres outside the waterline outline
    if (dd > 1.4) continue;
    float lip = dd + 0.02;
    float tr = dd - 0.22;
    a += B.y * exp(-lip * lip / 0.0036) - B.z * exp(-tr * tr / 0.045);
  }
  fragColor = vec4(inside ? h : 0.0, inside ? v : 0.0, T, a);
}`;

// height + static shape -> (height, slope x, slope z, turbulence), faded to zero at the domain edge
const WATER_RIP_COMPOSE_FRAG = /* glsl */`
precision highp float;
uniform sampler2D uState;
uniform float uRes, uTexel, uGain;
layout(location = 0) out vec4 fragColor;
float fadeAt(vec2 c) {
  vec2 f = (c + 0.5) / uRes;
  float e = min(min(f.x, 1.0 - f.x), min(f.y, 1.0 - f.y));
  return smoothstep(0.015, 0.13, e);
}
float H(ivec2 c) {
  c = clamp(c, ivec2(0), ivec2(int(uRes) - 1));
  vec4 s = texelFetch(uState, c, 0);
  return (s.r + s.a) * fadeAt(vec2(c)) * uGain;
}
void main() {
  ivec2 p = ivec2(gl_FragCoord.xy);
  float h = H(p);
  float sx = (H(p + ivec2(1, 0)) - H(p - ivec2(1, 0))) / (2.0 * uTexel);
  float sz = (H(p + ivec2(0, 1)) - H(p - ivec2(0, 1))) / (2.0 * uTexel);
  float T = texelFetch(uState, p, 0).b * fadeAt(vec2(p));
  fragColor = vec4(h, sx, sz, T);
}`;

// droplet impacts (from the spray system) splatted into the grid as small downward velocity impulses
const WATER_IMPACT_CLEAR = new THREE.Color(0, 0, 0);

const water = {
  q: -1, n: 0, texel: 0, size: 0,
  stateA: null, stateB: null, out: null, impact: null,
  center: new THREE.Vector2(),          // grid centre in the drifting-water frame (whole texels)
  drift: { x: 0, z: 0 },                // integrated surface current (m), float64
  shift: new THREE.Vector2(),
  ready: false, steps: 0,
  per: new Map(),                       // per-person state (previous hand positions, cooldowns ...)
  srcA: Array.from({ length: WATER_MAXS }, () => new THREE.Vector4()),
  srcB: Array.from({ length: WATER_MAXS }, () => new THREE.Vector4()),
  srcN: 0,
  events: [],                           // splash events for the spray system (filled by the ripple update)
};
const waterSimMat = passMat(WATER_RIP_SIM_FRAG, {
  uState: { value: null }, uImpact: { value: null }, uShift: { value: new THREE.Vector2() },
  uRes: { value: 1 }, uTexel: { value: 0.1 }, uDt: { value: 0 }, uGrav: { value: G }, uDamp: { value: 0.06 },
  uTurbTau: { value: 7 }, uDipole: { value: 4.5 }, uChurn: { value: 8 }, uSeed: { value: new THREE.Vector2() },
  uSrcA: { value: water.srcA }, uSrcB: { value: water.srcB }, uSrcN: { value: 0 },
  uBodA: { value: Array.from({ length: MAX_PEOPLE }, () => new THREE.Vector4()) },
  uBodB: { value: Array.from({ length: MAX_PEOPLE }, () => new THREE.Vector4()) }, uBodN: { value: 0 },
});
const waterComposeMat = passMat(WATER_RIP_COMPOSE_FRAG, { uState: { value: null }, uRes: { value: 1 }, uTexel: { value: 0.1 }, uGain: { value: 1 } });

function waterAllocRipples() {
  const q = clamp(Math.round(params.quality ?? 2), 0, 3);
  if (q === water.q) return;
  for (const k of ['stateA', 'stateB', 'out', 'impact']) if (water[k]) water[k].dispose();
  const { n, texel } = WATER_RIP_Q[q];
  Object.assign(water, { q, n, texel, size: n * texel });
  const st = { type: THREE.FloatType, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter };
  water.stateA = target(n, n, st);
  water.stateB = target(n, n, st);
  water.out = target(n, n, { minFilter: THREE.LinearMipmapLinearFilter, generateMipmaps: true, anisotropy: 4 });
  water.impact = target(n, n, { format: THREE.RedFormat, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter });
  const save = renderer.getClearColor(new THREE.Color()), saveA = renderer.getClearAlpha();
  renderer.setClearColor(WATER_IMPACT_CLEAR, 0);
  for (const rt of [water.stateA, water.stateB, water.out, water.impact]) { renderer.setRenderTarget(rt); renderer.clear(true, false, false); }
  renderer.setRenderTarget(null);
  renderer.setClearColor(save, saveA);
  waterSimMat.uniforms.uRes.value = waterComposeMat.uniforms.uRes.value = n;
  waterSimMat.uniforms.uTexel.value = waterComposeMat.uniforms.uTexel.value = texel;
  water.ready = false;
  if (typeof oceanU !== 'undefined' && oceanU.uWrTex) oceanU.uWrTex.value = water.out.texture;
}
waterAllocRipples();

// ---- ocean shader: ripple displacement (vertex), slope + roughness (fragment), turbulence micro-foam
// (the trailing '//' swallows the rest of the marker's comment line, whatever order the features inject in)
const waterInject = (hook, code, stage) => injectShader(ocean.material, hook, code + '\n//', stage);
Object.assign(oceanU, {
  uWrTex: { value: water.out.texture },
  uWrXf: { value: new THREE.Vector4(0, 0, water.size, water.texel) },   // lagrangian centre x, z, size, texel
  uWrOn: { value: 1 },
  uWrP: { value: new THREE.Vector4(1, 1, 1, 0) },                      // on, micro-foam, roughness, -
});
waterInject('ocean-vert-decl', 'uniform sampler2D uWrTex;\nuniform vec4 uWrXf;\nuniform float uWrOn;', 'vertex');
waterInject('ocean-vert-displace', /* glsl */`
  if (uWrOn > 0.5) {
    vec2 wrUv = (xz - uWrXf.xy) / uWrXf.z + 0.5;
    if (max(abs(wrUv.x - 0.5), abs(wrUv.y - 0.5)) < 0.5) disp.y += textureLod(uWrTex, wrUv, log2(max(spacing / uWrXf.w, 1.0))).r;
  }`, 'vertex');
waterInject('ocean-frag-decl', 'uniform sampler2D uWrTex;\nuniform vec4 uWrXf;\nuniform vec4 uWrP;');
waterInject('ocean-frag-slopes', /* glsl */`
  vec2 wrUv = (xz - uWrXf.xy) / uWrXf.z + 0.5;
  vec4 wrR = texture(uWrTex, wrUv) * (uWrP.x * step(max(abs(wrUv.x - 0.5), abs(wrUv.y - 0.5)), 0.5));
  s += wrR.yz;
  // churned water: short unresolved ripples raise the slope variance (wider, dimmer glitter)
  float wrRough = 0.006 * uWrP.z * clamp(wrR.w, 0.0, 2.0);
  cov.x += wrRough; cov.y += wrRough;`);
waterInject('ocean-frag-foam', /* glsl */`
  if (uWrP.y > 0.0) {
    // turbulence micro-foam: lacy bubble patches where hands and bodies churn the water, fine bubbles at the edges
    float wrT = clamp(wrR.w * uWrP.y, 0.0, 2.0);
    float wrLace = texture(uFoamTex, xz * 0.62 + vec2(0.71, 0.29) + uTime * vec2(0.0021, -0.0013)).r;
    float wrFine = texture(uFoamTex, xz * 1.9 + vec2(0.31, 0.17)).g;
    float wrPat = wrLace * 0.55 + wrFine * 0.45;
    float wrCv = clamp(wrT * 0.7, 0.0, 0.75);
    float wrFw = fwidth(wrPat) + 0.05;
    float wrM = smoothstep(1.0 - wrCv - wrFw, 1.0 - wrCv + wrFw, wrPat) * smoothstep(0.03, 0.35, wrT);
    foamMask = max(foamMask, wrM * (0.45 + 0.3 * smoothstep(0.4, 1.2, wrT)));
    cover = max(cover, wrT * 0.6);       // entrained bubbles brighten the water body below
  }`);

// ---- per-frame update
const _wv = new THREE.Vector3(), _wv2 = new THREE.Vector3();
const WATER_HANDS = ['LeftHand', 'RightHand'];
const WATER_FOREARM_PROBE = { LeftHand: 9, RightHand: 10 };
function waterPerson(P) {
  let s = water.per.get(P);
  if (!s) {
    s = { hands: WATER_HANDS.map(n => ({ bone: P.rig.byName(n), prev: new THREE.Vector3(), has: false, prevD: 0, drip: 0 })),
      slapCool: rand(1, 3), wRelF: 0, wMean: 0 };
    water.per.set(P, s);
  }
  return s;
}
function waterAddSource(tx, ty, rTex, wRel, ux, uz, couple, turb) {
  if (water.srcN >= WATER_MAXS) return;
  const i = water.srcN++;
  water.srcA[i].set(tx, ty, rTex, wRel);
  water.srcB[i].set(ux, uz, couple, turb);
}
// drifting-water-frame position (m) -> grid texel coordinates (continuous; texel centres at i + 0.5)
const waterTexX = x => (x - water.center.x) / water.texel + water.n / 2;
const waterTexZ = z => (z - water.center.y) / water.texel + water.n / 2;

function waterUpdate(dt) {
  waterAllocRipples();
  const on = params.waterRipples;
  oceanU.uWrOn.value = on ? 1 : 0;
  oceanU.uWrP.value.set(on ? 1 : 0, on ? params.waterMicroFoam : 0, 1, 0);
  water.events.length = 0;
  if (dt <= 0) return;
  // the drifting-water frame moves with the surface current
  water.drift.x += sea.current.x * dt; water.drift.z += sea.current.z * dt;

  // grid centre: lagrangian centroid of the people (neck probes), recentred in whole texels with hysteresis
  let cx = 0, cz = 0, cnt = 0;
  people.forEach((P, i) => { const pr = probe(i * PROBES_PER + 1); cx += pr.x0; cz += pr.z0; cnt++; });
  if (cnt) { cx /= cnt; cz /= cnt; } else { cx = controls.target.x; cz = controls.target.z; }
  cx -= water.drift.x; cz -= water.drift.z;
  water.shift.set(0, 0);
  if (!water.ready || Math.abs(cx - water.center.x) > 0.12 * water.size || Math.abs(cz - water.center.y) > 0.12 * water.size) {
    const nx = Math.round(cx / water.texel) * water.texel, nz = Math.round(cz / water.texel) * water.texel;
    if (water.ready) water.shift.set(Math.round((nx - water.center.x) / water.texel), Math.round((nz - water.center.y) / water.texel));
    else water.shift.set(1e5, 1e5);    // first step: start from a flat field
    water.center.set(nx, nz);
    water.ready = true;
  }
  if (!on) { water.ready = false; return; }

  // ---- forcing from the people
  water.srcN = 0;
  const bodA = waterSimMat.uniforms.uBodA.value, bodB = waterSimMat.uniforms.uBodB.value;
  const tx = water.texel, gain = params.waterRippleGain, wake = params.waterWake;
  let nb = 0;
  people.forEach((P, i) => {
    const S = waterPerson(P), base = i * PROBES_PER;
    const pN = probe(base + 1), pC = probe(base + 2), sc = probe(base + PROBES_PER - 1);
    // world -> drifting-water frame: subtract the local horizontal wave displacement and the drift
    const dX = P.probes[1].sx0 - pN.x0, dZ = P.probes[1].sz0 - pN.z0;
    const toTX = x => waterTexX(x - dX - water.drift.x), toTZ = z => waterTexZ(z - dZ - water.drift.z);
    const wRaw = P.vel.y - sc.vy;                                           // heave relative to the surface
    S.wMean += (wRaw - S.wMean) * (1 - Math.exp(-dt / 6));                  // (high-passed: no net pumping)
    const wRel = clamp(wRaw - S.wMean, -1.5, 1.5);
    S.wRelF += (wRel - S.wRelF) * (1 - Math.exp(-dt * 4));
    const ux = P.vel.x - sc.vx - sea.current.x, uz = P.vel.z - sc.vz - sea.current.z;   // drift through the water
    const sub = smoothstep(0.25, 0.6, P.sub);
    // torso: broad footprint at the waterline crossing (between the collar and chest probes)
    const ax = waterTexX(pN.x0 - water.drift.x), az = waterTexZ(pN.z0 - water.drift.z);
    const bx = waterTexX(pC.x0 - water.drift.x), bz = waterTexZ(pC.z0 - water.drift.z);
    waterAddSource((ax + bx) / 2, (az + bz) / 2, Math.max(0.26 / tx, 1.6), wRel * gain, ux * wake, uz * wake, 7 * sub,
      (0.15 + 1.0 * Math.abs(wRel) + 1.8 * Math.hypot(ux, uz)) * sub);
    // static waterline ring (trough deepens when the body rises out of the water, piles up when it sinks)
    if (nb < MAX_PEOPLE) {
      bodA[nb].set(ax, az, bx, bz);
      bodB[nb].set(0.17 / tx, 0.010 * sub * gain, clamp(0.012 + 0.05 * S.wRelF, -0.01, 0.04) * sub * gain, 0);
      nb++;
    }
    // hands sculling near the surface
    S.hands.forEach((H, k) => {
      if (!H.bone) return;
      H.bone.getWorldPosition(_wv);
      const fa = probe(base + WATER_FOREARM_PROBE[WATER_HANDS[k]]), fq = P.probes[WATER_FOREARM_PROBE[WATER_HANDS[k]]];
      const hs = fa.h + fa.sx * (_wv.x - fq.sx0) + fa.sz * (_wv.z - fq.sz0);
      const d = hs - _wv.y;                                                  // > 0: hand under water
      if (H.has && H.prev.distanceToSquared(_wv) < (6 * dt) ** 2) {         // (skip teleports / respawns)
        _wv2.copy(_wv).sub(H.prev).divideScalar(dt);                         // hand velocity (world)
        const rx = _wv2.x - fa.vx - sea.current.x, ry = _wv2.y - fa.vy, rz = _wv2.z - fa.vz - sea.current.z;
        const wgt = smoothstep(0.3, 0.0, d) * smoothstep(-0.06, 0.0, d);
        if (wgt > 0.01) {
          waterAddSource(toTX(_wv.x), toTZ(_wv.z), Math.max(0.085 / tx, 1.5), ry * gain, rx * gain, rz * gain, 9 * wgt,
            3.0 * Math.hypot(rx, ry, rz) * wgt);
        }
        // surface crossings and skimming -> splash events for the spray system
        const spd = Math.hypot(rx, ry, rz);
        if (H.prevD > 0 && d <= 0 && spd > 0.15) { water.events.push({ type: 'exit', pos: _wv.clone(), vel: new THREE.Vector3(rx, ry, rz), speed: spd }); H.drip = 1.2; }
        if (H.prevD < 0 && d >= 0 && spd > 0.25) water.events.push({ type: 'entry', pos: _wv.clone().setY(hs), vel: new THREE.Vector3(rx, ry, rz), speed: spd });
        if (Math.abs(d) < 0.05 && Math.hypot(rx, rz) > 0.3) water.events.push({ type: 'skim', pos: _wv.clone().setY(hs), vel: new THREE.Vector3(rx, ry, rz), speed: Math.hypot(rx, rz), dt });
        if (H.drip > 0 && d < -0.01) water.events.push({ type: 'drip', pos: _wv.clone(), vel: _wv2.clone(), level: H.drip, dt });
        H.drip = Math.max(0, H.drip - dt);
      }
      H.prev.copy(_wv); H.has = true; H.prevD = d;
    });
    // crest slapping the body: water rising fast past the body, steep or folding crest at the body
    S.slapCool -= dt;
    const J = probeBuf[(base + PROBES_PER - 1) * 4 + 3];
    const up = sc.vy - P.vel.y, hor = Math.hypot(sc.vx + sea.current.x - P.vel.x, sc.vz + sea.current.z - P.vel.z);
    const score = Math.max(up - 0.3, 0) * 2.2 + Math.max(hor - 0.45, 0) * 1.2 + Math.max(0.8 - J, 0) * 3;
    if (S.slapCool <= 0 && score > 0 && Math.random() < score * dt * 3) {
      S.slapCool = rand(1.2, 3.5);
      const n = probe(base + 1), q = P.probes[1];
      const pos = new THREE.Vector3(q.sx0, n.h + 0.02, q.sz0);
      const dir = new THREE.Vector3(sc.vx - P.vel.x, 0, sc.vz - P.vel.z);
      if (dir.lengthSq() < 1e-4) dir.set(sea.toward[0], 0, sea.toward[1]);
      dir.normalize();
      water.events.push({ type: 'slap', pos, dir, strength: clamp(0.4 + score, 0.4, 2.0) });
      waterAddSource((ax + bx) / 2, (az + bz) / 2, Math.max(0.35 / tx, 2), 0.25 * gain, 0, 0, 5, 2.5 * clamp(score, 0.3, 1.5));
    }
  });
  waterSimMat.uniforms.uBodN.value = nb;
  waterSimMat.uniforms.uSrcN.value = water.srcN;

  // ---- simulate (fixed substeps keep the dispersion stable at any time scale)
  const steps = Math.min(4, Math.max(1, Math.ceil(dt / 0.034)));
  const u = waterSimMat.uniforms;
  u.uDt.value = dt / steps;
  for (let k = 0; k < steps; k++) {
    u.uShift.value.copy(k === 0 ? water.shift : new THREE.Vector2(0, 0));
    u.uSeed.value.set(Math.random() * 512, Math.random() * 512);
    u.uState.value = water.stateA.texture;
    u.uImpact.value = water.impact.texture;
    runPass(waterSimMat, water.stateB);
    [water.stateA, water.stateB] = [water.stateB, water.stateA];
    if (k === 0) { renderer.setRenderTarget(water.impact); renderer.setClearColor(WATER_IMPACT_CLEAR, 0); renderer.clear(true, false, false); renderer.setClearColor(0x000000, 1); }
  }
  water.steps++;
  waterComposeMat.uniforms.uState.value = water.stateA.texture;
  waterComposeMat.uniforms.uGain.value = 1;
  runPass(waterComposeMat, water.out);
  renderer.setRenderTarget(null);
  oceanU.uWrTex.value = water.out.texture;
}
// ripple frame -> lagrangian (FFT) frame for the shaders
function waterSyncUniforms() {
  oceanU.uWrXf.value.set(water.center.x + water.drift.x, water.center.y + water.drift.z, water.size, water.texel);
}
HOOKS.update.push((dt) => { waterUpdate(dt); waterSyncUniforms(); });

// ---- GUI
const fWaterFx = gui.addFolder('Interactive water & spray');
fWaterFx.add(params, 'waterRipples').name('Ripples & wakes');
fWaterFx.add(params, 'waterRippleGain', 0, 3, 0.01).name('Ripple strength ×');
fWaterFx.add(params, 'waterWake', 0, 3, 0.01).name('Drift wake ×');
fWaterFx.add(params, 'waterMicroFoam', 0, 3, 0.01).name('Churn micro-foam ×');
HOOKS.ready.push(() => { window.sim.water = water; });
