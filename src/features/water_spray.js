// ============================================================================
// Spray, splashes and spindrift: GPU particles  (feature: water_)
// ============================================================================
// Particle state lives in three float textures (position+age, velocity+lifetime, kind/size/seed/height above
// the water) updated by one full-screen pass. Slots are handed out as a ring buffer: each frame the CPU lists
// emitters (hand strokes, crest slaps from water_ripples.js, and a whitecap emitter) and the next slots of the
// ring are re-initialised from them, so nothing is read back. The whitecap emitter searches the world foam map
// on the GPU for folding crests (J below the breaking threshold) and spawns blown spray, spindrift streaks and
// mist puffs there. Droplets fall under gravity with wind drag (log wind profile), die on the wave surface and
// splat a small impulse into the ripple field; mist puffs ride the wind and fade.
// Rendering: camera-facing motion-blurred streaks (droplets) and soft noisy billboards (mist), lit by the sun
// (forward-peaked droplet phase function, faint primary rainbow at 138 deg) and the sky, aerial perspective,
// soft contact with the bodies; band radiance in the LWIR sensor pass.

const WATER_SPRAY_Q = [[64, 64], [128, 64], [128, 128], [256, 128]];
const WATER_MAXE = 32;

const WATER_SPRAY_SIM_FRAG = CASC_SAMPLERS(['uDisp']) + /* glsl */`
precision highp float;
precision highp int;
#define NC ${NC}
#define MAXE ${WATER_MAXE}
${NOISE_GLSL}
uniform sampler2D uS0, uS1, uS2, uFoam, uRip;
uniform float uCascL[NC];
uniform vec2 uFoamC;
uniform float uFoamSize, uJThr, uWcR;
uniform vec4 uRipXf;           // ripple field: lagrangian centre x, z, size, texel
uniform float uRipOn;
uniform float uDt, uTime, uFrame, uTexW, uNP, uHead, uSpawnN;
uniform vec3 uWind;            // 10 m wind vector x, z, speed
uniform vec4 uEmA[MAXE];       // position xyz, cumulative slot end
uniform vec4 uEmB[MAXE];       // velocity xyz, velocity spread
uniform vec4 uEmC[MAXE];       // type (0 droplets, 1 mist, 2 whitecap search), size, lifetime, radius
uniform int uEmN;
layout(location = 0) out vec4 o0;   // position, age
layout(location = 1) out vec4 o1;   // velocity, lifetime (0 = dead)
layout(location = 2) out vec4 o2;   // kind (0 droplet, 1 mist, 2 spindrift), size, seed, height above water | impact
uint gSeed;
float rnd() { gSeed = pcgHash(gSeed); return float(gSeed >> 8u) / 16777216.0; }
vec3 rndDir() { float z = rnd() * 2.0 - 1.0, a = rnd() * 6.2831853, r = sqrt(max(1.0 - z * z, 0.0)); return vec3(r * cos(a), z, r * sin(a)); }
vec3 dispAt(vec2 x0) { vec3 d = vec3(0.0); ${forC(c => `d += textureLod(uDisp${c}, x0 / uCascL[${c}], 0.0).xyz;`)} return d; }
float ripAt(vec2 x0) {
  vec2 uv = (x0 - uRipXf.xy) / uRipXf.z + 0.5;
  return (uRipOn > 0.5 && max(abs(uv.x - 0.5), abs(uv.y - 0.5)) < 0.5) ? textureLod(uRip, uv, 0.0).r : 0.0;
}
// surface height under a world point: invert the horizontal (choppy) displacement first
float surfaceAt(vec2 p, out vec2 x0) {
  x0 = p;
  for (int i = 0; i < 3; i++) x0 = p - dispAt(x0).xz;
  return dispAt(x0).y + ripAt(x0);
}
void spawn(int e) {
  vec4 A = uEmA[e], B = uEmB[e], C = uEmC[e];
  int type = int(C.x + 0.5);
  vec3 p, v;
  float kind, size, life;
  if (type == 2) {
    // whitecap spray: look for a folding crest in the world foam map (b = Jacobian)
    vec2 x0 = vec2(0.0);
    float brk = 0.0;
    for (int t = 0; t < 14; t++) {
      float r = uWcR * sqrt(rnd()), a = 6.2831853 * rnd();
      vec2 c = uFoamC + r * vec2(cos(a), sin(a));
      vec4 f = textureLod(uFoam, (c - uFoamC) / uFoamSize + 0.5, 0.0);
      float b = clamp(max((uJThr - f.b) * 6.0, (f.g - 0.55) * 1.6), 0.0, 1.0);   // folding now, or freshly broken
      if (rnd() < b) { x0 = c; brk = b; break; }
    }
    if (brk <= 0.0) { o0 = vec4(0.0); o1 = vec4(0.0); o2 = vec4(0.0); return; }
    vec3 D = dispAt(x0);
    vec2 wd = normalize(uWind.xy + vec2(1e-4, 0.0));
    float U = uWind.z, r = rnd();
    float driftShare = smoothstep(11.0, 19.0, U);
    p = vec3(x0.x + D.x, D.y + 0.04 + 0.12 * rnd(), x0.y + D.z);
    vec3 w3 = vec3(wd.x, 0.0, wd.y);
    if (r < 0.28) {                               // mist blown off the crest
      kind = 1.0; size = 0.22 + 0.4 * rnd() * (0.6 + 0.4 * brk); life = 1.3 + 1.7 * rnd();
      v = w3 * U * (0.25 + 0.2 * rnd()) + vec3(0.0, 0.35 + 0.7 * rnd(), 0.0);
    } else if (r < 0.28 + 0.6 * driftShare) {     // spindrift: streaks torn off the crest, low and fast
      kind = 2.0; size = 0.01 + 0.012 * rnd(); life = 0.7 + 1.1 * rnd();
      v = w3 * U * (0.4 + 0.25 * rnd()) + vec3(0.0, 0.25 + 0.8 * rnd(), 0.0) + rndDir() * 0.3;
    } else {                                      // spray droplets thrown up by the breaking crest
      kind = 0.0; size = 0.007 + 0.01 * rnd(); life = 2.5;
      v = w3 * U * (0.12 + 0.15 * rnd()) + vec3(0.0, 0.8 + 2.4 * rnd() * brk, 0.0) + rndDir() * 0.6;
    }
  } else {
    vec3 o = rndDir() * C.w * pow(rnd(), 0.4);
    p = A.xyz + vec3(o.x, abs(o.y) * 0.3, o.z);
    v = B.xyz + rndDir() * B.w;
    kind = type == 1 ? 1.0 : 0.0;
    float sr = rnd();
    size = C.y * (type == 1 ? 0.6 + 0.8 * sr : 0.3 + 1.6 * sr * sr * sr);    // droplets: many small, a few big
    life = C.z * (0.6 + 0.8 * rnd());
  }
  o0 = vec4(p, 0.0);
  o1 = vec4(v, life);
  o2 = vec4(kind, size, rnd(), 0.05);
}
void main() {
  ivec2 pc = ivec2(gl_FragCoord.xy);
  float idx = float(pc.y) * uTexW + float(pc.x);
  gSeed = pcgHash(uint(idx) * 9781u + uint(uFrame) * 6271u + 17u);
  float k = mod(idx - uHead + uNP, uNP);
  if (k < uSpawnN) {
    int e = 0;
    for (int i = 0; i < MAXE; i++) { if (i >= uEmN) break; e = i; if (k < uEmA[i].w) break; }
    spawn(e);
    return;
  }
  vec4 s0 = texelFetch(uS0, pc, 0), s1 = texelFetch(uS1, pc, 0), s2 = texelFetch(uS2, pc, 0);
  if (s1.w <= 0.0) { o0 = s0; o1 = s1; o2 = vec4(s2.xyz, 0.0); return; }      // dead (impact reported once)
  vec3 p = s0.xyz, v = s1.xyz;
  float age = s0.w + uDt, life = s1.w, kind = s2.x;
  vec2 x0;
  float hs = surfaceAt(p.xz, x0);
  // log wind profile above the waves (z0 ~ 0.2 mm), plus small eddies for the mist
  float z = max(p.y - hs, 0.02);
  vec3 wind = vec3(uWind.x, 0.0, uWind.y) * clamp(log(z / 2e-4) / log(5e4), 0.3, 1.15);
  bool mist = kind > 0.5 && kind < 1.5;
  if (mist) wind += vec3(gnoise(p.xz * 0.9 + uTime * 0.7), 0.6 * gnoise(p.zx * 0.9 - uTime * 0.6), gnoise(p.xz * 0.9 + 31.0 + uTime * 0.5)) * (0.25 + 0.08 * uWind.z);
  float tau = kind < 0.5 ? 1.1 : (mist ? 0.12 : 0.35);       // drag relaxation time (s)
  float gEff = kind < 0.5 ? 9.81 : (mist ? 0.35 : 8.0);      // mist settles slowly
  v += (wind - v) * (1.0 - exp(-uDt / tau));
  v.y -= gEff * uDt;
  p += v * uDt;
  hs = surfaceAt(p.xz, x0);
  float hA = p.y - hs;
  if (age > life) { o0 = vec4(p, age); o1 = vec4(v, 0.0); o2 = vec4(s2.xyz, 0.0); return; }
  if (hA < 0.0 && (mist || v.y < 0.0)) {     // (still rising out of the water: not a hit yet)
    if (mist) { p.y = hs; v.y = max(v.y, 0.0); hA = 0.0; }
    else {
      // into the sea: report a ripple impulse at the lagrangian impact point
      o0 = vec4(x0.x, hs, x0.y, age);
      o1 = vec4(v, 0.0);
      o2 = vec4(s2.xyz, clamp(s2.y * 22.0 * max(-v.y, 0.5), 0.0, 0.25));
      return;
    }
  }
  o0 = vec4(p, age);
  o1 = vec4(v, life);
  o2 = vec4(s2.xyz, hA);
}`;

// impacts -> ripple field (points, additive)
const WATER_IMPACT_VERT = /* glsl */`
uniform sampler2D uS1, uS2, uS0;
uniform float uTexW;
uniform vec4 uRipXf;
out float vImp;
void main() {
  int W = int(uTexW);
  ivec2 pc = ivec2(gl_VertexID % W, gl_VertexID / W);
  vec4 s2 = texelFetch(uS2, pc, 0);
  vImp = 0.0;
  gl_Position = vec4(2.0, 2.0, 0.0, 1.0);
  gl_PointSize = 1.0;
  if (texelFetch(uS1, pc, 0).w > 0.0 || s2.w <= 0.0) return;
  vec2 x0 = texelFetch(uS0, pc, 0).xz;
  vec2 uv = (x0 - uRipXf.xy) / uRipXf.z + 0.5;
  if (max(abs(uv.x - 0.5), abs(uv.y - 0.5)) > 0.48) return;
  vImp = s2.w;
  gl_Position = vec4(uv * 2.0 - 1.0, 0.0, 1.0);
  gl_PointSize = 4.0;
}`;
const WATER_IMPACT_FRAG = /* glsl */`
in float vImp;
layout(location = 0) out vec4 fragColor;
void main() {
  vec2 d = gl_PointCoord * 2.0 - 1.0;
  fragColor = vec4(vImp * exp(-2.5 * dot(d, d)), 0.0, 0.0, 1.0);
}`;

const WATER_SPRAY_VERT = /* glsl */`
uniform sampler2D uS0, uS1, uS2;
uniform samplerCube uEnv;
uniform float uTexW, uExpo;
uniform vec3 uSunDir, uSunE;
uniform vec2 uResolution;
uniform vec4 uFogExt;
${THERMAL_UNIFORMS_GLSL}${THERMAL_GLSL}
out vec4 vCol;          // premultiplied radiance, opacity
out vec3 vShape;        // streak: along (px), across (px), length (px)
out vec2 vUv;           // mist: billboard coordinates
out float vHalfW, vKind, vSeed, vViewZ, vSoft;
float wrHG(float c, float g) { float g2 = g * g; return (1.0 - g2) / (12.566371 * pow(max(1.0 + g2 - 2.0 * g * c, 1e-4), 1.5)); }
void main() {
  int W = int(uTexW);
  ivec2 pc = ivec2(gl_InstanceID % W, gl_InstanceID / W);
  vec4 s0 = texelFetch(uS0, pc, 0), s1 = texelFetch(uS1, pc, 0), s2 = texelFetch(uS2, pc, 0);
  gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
  vCol = vec4(0.0); vShape = vec3(0.0); vUv = vec2(0.0); vHalfW = 0.0; vKind = s2.x; vSeed = s2.z; vViewZ = 1.0; vSoft = 1.0;
  if (s1.w <= 0.0) return;
  vec3 p = s0.xyz, v = s1.xyz;
  float t = clamp(s0.w / s1.w, 0.0, 1.0), kind = s2.x, size = s2.y, seed = s2.z;
  bool mist = kind > 0.5 && kind < 1.5;
  vec4 c0 = projectionMatrix * viewMatrix * vec4(p, 1.0);
  if (c0.w < 0.08) return;
  float pxPerM = projectionMatrix[1][1] * 0.5 * uResolution.y;
  vec2 sp0 = (c0.xy / c0.w * 0.5 + 0.5) * uResolution;
  vec2 spx;
  float cw = c0.w, cz = c0.z, alpha;
  if (mist) {
    float sz = size * (0.55 + 1.45 * sqrt(t));
    float rp = sz * pxPerM / c0.w, rc = max(rp, 1.5);
    alpha = (0.13 + 0.15 * fract(seed * 7.31)) * smoothstep(0.0, 0.1, t) * (1.0 - smoothstep(0.3, 1.0, t)) * min(1.0, rp * rp / (rc * rc));
    alpha *= 1.0 - smoothstep(220.0, 480.0, rp);                    // puffs right at the lens: skip (fill rate)
    // streamed out along the wind: stretch the billboard along its screen-space motion
    vec4 cv = projectionMatrix * viewMatrix * vec4(p + v * 0.15, 1.0);
    vec2 dv = cv.w > 0.08 ? (cv.xy / cv.w * 0.5 + 0.5) * uResolution - sp0 : vec2(0.0);
    float dl = length(dv);
    vec2 ax = dl > 1e-3 ? dv / dl : vec2(1.0, 0.0);
    float el = 1.0 + clamp(dl / rc * 0.7, 0.0, 2.2);
    spx = sp0 + ax * position.x * rc * el + vec2(-ax.y, ax.x) * position.y * rc;
    alpha /= sqrt(el);
    vUv = position.xy;
    vHalfW = rc;
    vSoft = smoothstep(-0.05, 0.4 * sz, s2.w);
  } else {
    // droplet: capsule smeared along its screen motion during the exposure
    vec4 c1 = projectionMatrix * viewMatrix * vec4(p - v * uExpo * (kind > 1.5 ? 3.0 : 1.0), 1.0);
    if (c1.w < 0.08) c1 = c0;
    vec2 sp1 = (c1.xy / c1.w * 0.5 + 0.5) * uResolution;
    vec2 dir = sp0 - sp1;
    float len = length(dir);
    dir = len > 1e-3 ? dir / len : vec2(1.0, 0.0);
    float wT = size * pxPerM / c0.w, w = max(wT, 1.4), hw = 0.5 * w + 0.7;
    alpha = (kind > 1.5 ? 0.6 : 1.0) * min(1.0, 1.6 * wT * wT / ((len + w) * w)) * (1.0 - smoothstep(0.8, 1.0, t));
    alpha *= 1.0 - smoothstep(18.0, 40.0, wT);                      // drops brushing the lens
    float along = position.x < 0.0 ? -hw : len + hw;
    spx = (position.x < 0.0 ? sp1 - dir * hw : sp0 + dir * hw) + vec2(-dir.y, dir.x) * position.y * hw;
    if (position.x < 0.0) { cw = c1.w; cz = c1.z; }
    vShape = vec3(along, position.y * hw, len);
    vHalfW = 0.5 * w;
  }
  if (alpha < 0.002) return;
  gl_Position = vec4((spx / uResolution * 2.0 - 1.0) * cw, cz, cw);
  vViewZ = c0.w;
  vec3 Vv = cameraPosition - p;
  float dist = length(Vv);
  vec3 V = Vv / dist;
  if (uThermal > 0.5) {
    // LWIR: near-blackbody droplets at about the sea temperature, cooling towards the wet bulb as they fly
    float em = mist ? 0.9 : 0.96;
    float Tp = mix(uTwater - 0.3, min(uTwater, uTair) - 1.5, t * 0.6);
    float Ls = skyRadianceLWIR(0.35, uTair, uRH, uCloudFrac, uTcloud);
    float tau = pathTransmittanceLWIR(dist, uRH, uTair);
    float L = (em * planckBand(Tp) + (1.0 - em) * Ls) * tau + planckBand(uTair) * (1.0 - tau);
    float aT = alpha * (mist ? 0.5 : 1.0);                      // fine mist is thinner at 10 um than in the visible
    vCol = vec4(L * aT, 0.0, 0.0, aT);
    return;
  }
  float cosT = dot(-uSunDir, V);                                // 1 = looking into the sun through the spray
  float sunUp = smoothstep(-0.03, 0.04, uSunDir.y);
  float ph = 0.78 * wrHG(cosT, 0.8) + 0.22 * wrHG(cosT, -0.25);
  float th = degrees(acos(clamp(cosT, -1.0, 1.0)));
  vec3 bowD = (vec3(th) - vec3(137.6, 138.4, 139.3)) / 0.75;
  vec3 bow = exp(-bowD * bowD);                                 // primary rainbow (red outside)
  vec3 skyUp = textureLod(uEnv, vec3(0.0, 1.0, 0.0), 8.0).rgb;
  vec3 skyH = 0.25 * (textureLod(uEnv, vec3(0.94, 0.34, 0.0), 7.0).rgb + textureLod(uEnv, vec3(-0.94, 0.34, 0.0), 7.0).rgb +
                      textureLod(uEnv, vec3(0.0, 0.34, 0.94), 7.0).rgb + textureLod(uEnv, vec3(0.0, 0.34, -0.94), 7.0).rgb);
  vec3 amb = skyUp * 0.45 + skyH * 0.55;
  vec3 col;
  if (mist) {
    // dense spray multiply-scatters like foam (albedo ~0.85) plus the forward-scattering single-scatter lobe
    col = uSunE * sunUp * (0.85 * 0.16 * clamp(0.4 + 2.4 * max(uSunDir.y, 0.0), 0.0, 1.0) + ph * 1.2 + bow * 0.06) + amb * 1.35;
  } else {
    // glints: each droplet throws a tiny image of the sun, twinkling as it wobbles
    float tw = fract(sin(seed * 913.7 + floor(s0.w * 24.0) * 17.13) * 43758.55);
    float glint = smoothstep(0.8, 1.0, tw) * (0.3 + 4.0 * ph);
    col = uSunE * sunUp * (ph * 1.4 + 0.07 + glint + bow * 0.05) + amb * 1.25;
  }
  vec3 T = exp(-max(uFogExt.rgb, vec3(uFogExt.w)) * dist);
  vec3 fogCol = textureLod(uEnv, normalize(vec3(-V.x, max(abs(V.y), 0.002), -V.z)), 1.0).rgb;
  col = col * T + fogCol * (1.0 - T);
  vCol = vec4(col * alpha, alpha);
}`;
const WATER_SPRAY_FRAG = /* glsl */`
uniform sampler2D uFoamTex, uRefrDepth;
uniform vec2 uResolution;
uniform float uNear, uFar, uThermal;
in vec4 vCol;
in vec3 vShape;
in vec2 vUv;
in float vHalfW, vKind, vSeed, vViewZ, vSoft;
layout(location = 0) out vec4 fragColor;
void main() {
  if (vCol.a <= 0.0) discard;
  float a;
  if (vKind > 0.5 && vKind < 1.5) {
    float r = length(vUv);
    if (r > 1.0) discard;
    float ang = vSeed * 6.2831853;
    vec2 tuv = mat2(cos(ang), sin(ang), -sin(ang), cos(ang)) * vUv * 0.2 + vec2(fract(vSeed * 17.0), fract(vSeed * 29.0));
    float n1 = texture(uFoamTex, tuv).b, n2 = texture(uFoamTex, tuv * 2.3 + 0.37).r;
    a = smoothstep(1.0, 0.2, r) * smoothstep(0.2, 0.8, n1 * 0.75 + n2 * 0.45 - r * 0.3);
    a *= vSoft;
  } else {
    float da = max(max(-vShape.x, vShape.x - vShape.z), 0.0);
    float dd = length(vec2(da, vShape.y));
    a = clamp(vHalfW + 0.5 - dd, 0.0, 1.0);
    // resolved drops are clear beads: thin in the middle, bright Fresnel rim, a pin-point highlight
    float rr = clamp(dd / max(vHalfW, 1e-3), 0.0, 1.0);
    float big = smoothstep(1.5, 4.0, vHalfW);
    a *= mix(1.0, 0.3 + 0.7 * rr * rr + 1.5 * exp(-dot(vec2(vShape.x - vShape.z * 0.8, vShape.y + 0.35 * vHalfW), vec2(vShape.x - vShape.z * 0.8, vShape.y + 0.35 * vHalfW)) / max(0.04 * vHalfW * vHalfW, 0.5)), big);
  }
  // soft contact with the people (depth of the refraction pre-pass); not in the LWIR pass, which has none
  if (uThermal < 0.5) {
    float sd = texture(uRefrDepth, gl_FragCoord.xy / uResolution).r;
    if (sd < 1.0) {
      float sceneZ = uNear * uFar / (uFar - sd * (uFar - uNear));
      a *= clamp((sceneZ - vViewZ) / 0.12, 0.0, 1.0);
    }
  }
  if (a <= 0.001) discard;
  fragColor = vCol * a;
}`;

const spray = {
  q: -1, W: 0, H: 0, NP: 0, src: null, dst: null, head: 0, frame: 0, spawned: 0,
  emA: Array.from({ length: WATER_MAXE }, () => new THREE.Vector4()),
  emB: Array.from({ length: WATER_MAXE }, () => new THREE.Vector4()),
  emC: Array.from({ length: WATER_MAXE }, () => new THREE.Vector4()),
};
const waterSpraySimMat = passMat(WATER_SPRAY_SIM_FRAG, {
  uS0: { value: null }, uS1: { value: null }, uS2: { value: null }, uFoam: { value: null }, uRip: { value: null },
  uCascL: { value: CASC_L }, uFoamC: { value: new THREE.Vector2() }, uFoamSize: { value: FOAM_SIZE }, uJThr: { value: 0.6 },
  uWcR: { value: FOAM_SIZE * 0.36 }, uRipXf: { value: oceanU.uWrXf.value }, uRipOn: { value: 1 },
  uDt: { value: 0 }, uTime: { value: 0 }, uFrame: { value: 0 }, uTexW: { value: 1 }, uNP: { value: 1 }, uHead: { value: 0 }, uSpawnN: { value: 0 },
  uWind: { value: new THREE.Vector3() },
  uEmA: { value: spray.emA }, uEmB: { value: spray.emB }, uEmC: { value: spray.emC }, uEmN: { value: 0 },
  ...Object.fromEntries(CASC_L.map((_, c) => [`uDisp${c}`, { value: DISP[c] }])),
});
const waterImpactMat = new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3, vertexShader: WATER_IMPACT_VERT, fragmentShader: WATER_IMPACT_FRAG,
  uniforms: { uS0: { value: null }, uS1: { value: null }, uS2: { value: null }, uTexW: { value: 1 }, uRipXf: { value: oceanU.uWrXf.value } },
  blending: THREE.AdditiveBlending, depthTest: false, depthWrite: false, transparent: true,
});
const waterImpactScene = new THREE.Scene();
let waterImpactPts = null;

const waterSprayMat = new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3, vertexShader: WATER_SPRAY_VERT, fragmentShader: WATER_SPRAY_FRAG,
  uniforms: {
    uS0: { value: null }, uS1: { value: null }, uS2: { value: null }, uTexW: { value: 1 }, uExpo: { value: 1 / 60 },
    uEnv: oceanU.uEnv, uSunDir: oceanU.uSunDir, uSunE: oceanU.uSunE, uResolution: oceanU.uResolution, uFogExt: oceanU.uFogExt,
    uFoamTex: oceanU.uFoamTex, uRefrDepth: oceanU.uRefrDepth, uNear: oceanU.uNear, uFar: oceanU.uFar,
    ...THERM,
  },
  transparent: true, depthWrite: false, depthTest: true,
  blending: THREE.CustomBlending, blendEquation: THREE.AddEquation,
  blendSrc: THREE.OneFactor, blendDst: THREE.OneMinusSrcAlphaFactor,
  blendSrcAlpha: THREE.OneFactor, blendDstAlpha: THREE.OneMinusSrcAlphaFactor,
});
const waterSprayGeo = new THREE.InstancedBufferGeometry();
waterSprayGeo.setAttribute('position', new THREE.Float32BufferAttribute([-1, -1, 0, 1, -1, 0, 1, 1, 0, -1, 1, 0], 3));
waterSprayGeo.setIndex([0, 1, 2, 0, 2, 3]);
const waterSprayMesh = new THREE.Mesh(waterSprayGeo, waterSprayMat);
waterSprayMesh.frustumCulled = false;
waterSprayMesh.renderOrder = 2;
scene.add(waterSprayMesh);

function waterAllocSpray() {
  const q = clamp(Math.round(params.quality ?? 2), 0, 3);
  if (q === spray.q) return;
  if (spray.src) { spray.src.dispose(); spray.dst.dispose(); }
  const [W, H] = WATER_SPRAY_Q[q];
  const mk = () => new THREE.WebGLRenderTarget(W, H, {
    count: 3, type: THREE.FloatType, depthBuffer: false, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter, generateMipmaps: false,
  });
  Object.assign(spray, { q, W, H, NP: W * H, src: mk(), dst: mk(), head: 0 });
  const save = renderer.getClearColor(new THREE.Color()), saveA = renderer.getClearAlpha();
  renderer.setClearColor(0x000000, 0);
  for (const rt of [spray.src, spray.dst]) { renderer.setRenderTarget(rt); renderer.clear(true, false, false); }
  renderer.setRenderTarget(null);
  renderer.setClearColor(save, saveA);
  waterSpraySimMat.uniforms.uTexW.value = waterSprayMat.uniforms.uTexW.value = waterImpactMat.uniforms.uTexW.value = W;
  waterSpraySimMat.uniforms.uNP.value = W * H;
  waterSprayGeo.instanceCount = W * H;
  if (waterImpactPts) { waterImpactScene.remove(waterImpactPts); waterImpactPts.geometry.dispose(); }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(W * H * 3), 3));
  waterImpactPts = new THREE.Points(g, waterImpactMat);
  waterImpactPts.frustumCulled = false;
  waterImpactScene.add(waterImpactPts);
}
waterAllocSpray();

// ---- emitters
let waterEmN = 0, waterEmCount = 0;
const waterPois = x => Math.floor(x + Math.random());
// type 0 droplets / 1 mist; pos, vel: THREE.Vector3-like
function waterEmit(type, count, pos, vel, spread, size, life, radius) {
  count = Math.round(count);
  if (count <= 0 || waterEmN >= WATER_MAXE) return;
  const i = waterEmN++;
  waterEmCount += count;
  spray.emA[i].set(pos.x, pos.y, pos.z, waterEmCount);
  spray.emB[i].set(vel.x, vel.y, vel.z, spread);
  spray.emC[i].set(type, size, life, radius);
}
const _wsUp = new THREE.Vector3(0, 1, 0), _wsV = new THREE.Vector3(), _wsP = new THREE.Vector3();
function waterSprayEmitters(dt) {
  waterEmN = 0; waterEmCount = 0;
  const amt = params.waterSprayAmount, sp = params.waterSplashes * amt;
  for (const ev of water.events) {
    switch (ev.type) {
      case 'exit':     // hand lifted out: a sheet of water breaks into drops that follow the hand
        waterEmit(0, clamp((ev.speed - 0.1) * 70, 2, 30) * sp, ev.pos, _wsV.copy(ev.vel).multiplyScalar(0.7).addScaledVector(_wsUp, 0.3), 0.3, 0.009, 1.6, 0.05);
        break;
      case 'entry':    // hand slapping in: crown of drops, a little mist when hard
        waterEmit(0, clamp(ev.speed * 45, 3, 30) * sp, ev.pos, _wsV.set(ev.vel.x * 0.3, 0.6 + ev.speed * 1.4, ev.vel.z * 0.3), 0.35 + 0.4 * ev.speed, 0.009, 1.5, 0.06);
        if (ev.speed > 0.6) waterEmit(1, 1 * sp, ev.pos, _wsV.set(0, 0.3, 0), 0.2, 0.12, 1.0, 0.05);
        break;
      case 'skim':     // hand sculling through the surface: small flicks of spray ahead of it
        waterEmit(0, waterPois(ev.speed * ev.dt * 60 * sp), ev.pos, _wsV.copy(ev.vel).multiplyScalar(0.8).addScaledVector(_wsUp, 0.45), 0.3, 0.008, 1.2, 0.04);
        break;
      case 'drip':     // water running off the raised hand
        waterEmit(0, waterPois(16 * ev.level * ev.dt * sp), ev.pos, _wsV.copy(ev.vel).multiplyScalar(0.9), 0.04, 0.007, 1.2, 0.03);
        break;
      case 'slap': {   // crest breaking against the body: a burst of drops and a puff of mist up the face
        const s = ev.strength;
        _wsV.copy(ev.dir).multiplyScalar(0.6 + 0.8 * s).addScaledVector(_wsUp, 1.1 + 1.4 * s);
        waterEmit(0, (20 + 55 * s) * sp, ev.pos, _wsV, 0.5 + 0.4 * s, 0.010, 2.0, 0.22);
        waterEmit(1, (2 + 4 * s) * sp, _wsP.copy(ev.pos).addScaledVector(_wsUp, 0.08), _wsV.copy(ev.dir).multiplyScalar(0.5).addScaledVector(_wsUp, 0.5 + 0.4 * s), 0.3, 0.2 + 0.1 * s, 1.8, 0.2);
        break;
      }
    }
  }
  // breaking whitecaps in strong wind: attempts scale with wind; acceptance scales with the breaking area
  const U = params.windSpeed;
  const windF = smoothstep(7.5, 14, U) * (1 + 2 * smoothstep(14, 22, U));
  const rate = 3200 * windF * amt * params.waterWhitecapSpray * (0.5 + 0.25 * spray.q);
  if (rate > 0) waterEmit(2, waterPois(rate * dt), _wsP.set(0, 0, 0), _wsV.set(0, 0, 0), 0, 0, 0, 0);
  // the ring must not overwrite live particles: cap at the slots that free up per frame
  const cap = Math.floor(spray.NP * dt / 3) + 8;
  if (waterEmCount > cap) {
    const f = cap / waterEmCount;
    let acc = 0, run = 0;
    for (let i = 0; i < waterEmN; i++) {
      const n = spray.emA[i].w - run; run = spray.emA[i].w;
      acc += Math.floor(n * f);
      spray.emA[i].w = acc;
    }
    waterEmCount = acc;
  }
}

function waterSprayUpdate(dt) {
  waterAllocSpray();
  waterSprayMesh.visible = params.waterSpray;
  if (!params.waterSpray || dt <= 0) return;
  waterSprayEmitters(dt);
  const u = waterSpraySimMat.uniforms;
  u.uS0.value = spray.src.textures[0]; u.uS1.value = spray.src.textures[1]; u.uS2.value = spray.src.textures[2];
  u.uFoam.value = foamSrc.texture;
  u.uFoamC.value.copy(foamUpdMat.uniforms.uCenter.value);
  u.uJThr.value = foamUpdMat.uniforms.uJThr.value;
  u.uRip.value = water.out.texture;
  u.uRipOn.value = params.waterRipples ? 1 : 0;
  u.uDt.value = Math.min(dt, 0.05);
  u.uTime.value = simTime;
  u.uFrame.value = (spray.frame++) % 1e6;
  u.uHead.value = spray.head;
  u.uSpawnN.value = waterEmCount;
  u.uEmN.value = waterEmN;
  u.uWind.value.set(sea.toward[0] * params.windSpeed, sea.toward[1] * params.windSpeed, params.windSpeed);
  runPass(waterSpraySimMat, spray.dst);
  [spray.src, spray.dst] = [spray.dst, spray.src];
  spray.head = (spray.head + waterEmCount) % spray.NP;
  spray.spawned += waterEmCount;
  const r = spray.src.textures;
  waterSprayMat.uniforms.uS0.value = r[0]; waterSprayMat.uniforms.uS1.value = r[1]; waterSprayMat.uniforms.uS2.value = r[2];
  // droplets that hit the sea this step -> ripple impulses (consumed by the next ripple step)
  if (params.waterRipples && water.ready) {
    waterImpactMat.uniforms.uS0.value = r[0]; waterImpactMat.uniforms.uS1.value = r[1]; waterImpactMat.uniforms.uS2.value = r[2];
    const ac = renderer.autoClear;
    renderer.autoClear = false;
    renderer.setRenderTarget(water.impact);
    renderer.render(waterImpactScene, fsCam);
    renderer.autoClear = ac;
  }
  renderer.setRenderTarget(null);
}
HOOKS.update.push((dt) => waterSprayUpdate(dt));
HOOKS.ready.push(() => { window.sim.spray = spray; });

fWaterFx.add(params, 'waterSpray').name('Spray & splashes');
fWaterFx.add(params, 'waterSprayAmount', 0, 3, 0.01).name('Spray amount ×');
fWaterFx.add(params, 'waterWhitecapSpray', 0, 3, 0.01).name('Whitecap spray / spindrift ×');
fWaterFx.add(params, 'waterSplashes', 0, 3, 0.01).name('Splashes (strokes, slaps) ×');
