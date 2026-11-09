// ============================================================================
// FFT ocean simulation (GPU)
// ============================================================================
const SIM_COMMON = /* glsl */`
#define NFFT ${N}
const float PI = 3.14159265358979;
const float GRAV = 9.81;
const float SURF = 7.4e-5;
vec2 waveVec(ivec2 id, float L) {
  int nx = id.x >= NFFT / 2 ? id.x - NFFT : id.x;
  int ny = id.y >= NFFT / 2 ? id.y - NFFT : id.y;
  return vec2(float(nx), float(ny)) * (2.0 * PI / L);
}
float omegaOf(float k) { return sqrt(GRAV * k + SURF * k * k * k); }
float dOmegaOf(float k) { return (GRAV + 3.0 * SURF * k * k) / (2.0 * omegaOf(k)); }
vec2 cmul(vec2 a, vec2 b) { return vec2(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x); }
vec2 mulI(vec2 a) { return vec2(-a.y, a.x); }
`;

// initial spectrum h0(k) and conj(h0(-k)) for one cascade
const INIT_FRAG = SIM_COMMON + NOISE_GLSL + /* glsl */`
uniform float uL, uKMin, uKMax, uCasc, uGain, uSpread, uFineGain;
uniform vec4 uWind;    // U10, direction (rad), fetch (m), gamma
uniform vec4 uSwell;   // Hs, direction, peak omega, spreading exponent
uniform vec4 uMisc;    // -, short-wave cut (m), -, seed
layout(location = 0) out vec4 fragColor;
float rand01(uint s) { return (float(pcgHash(s) >> 8u) + 0.5) / 16777216.0; }
vec2 gauss2(uint seed) {
  float u1 = rand01(seed * 2u + 17u);
  float u2 = rand01(seed * 2u + 18u + pcgHash(seed));
  float r = sqrt(-2.0 * log(u1));
  float a = 2.0 * PI * u2;
  return r * vec2(cos(a), sin(a));
}
float wrapAngle(float a) { return a - 2.0 * PI * floor((a + PI) / (2.0 * PI)); }
float jonswap(float w, float wp, float alpha, float gamma) {
  float sigma = w <= wp ? 0.07 : 0.09;
  float d = (w - wp) / (sigma * wp);
  float q = wp / w;
  float q2 = q * q;
  return alpha * GRAV * GRAV / (w * w * w * w * w) * exp(-1.25 * q2 * q2) * pow(gamma, exp(-0.5 * d * d));
}
float donelanBanner(float theta, float w, float wp) {
  float ratio = w / wp;
  float beta;
  if (ratio < 0.95) beta = 2.61 * pow(ratio, 1.3);
  else if (ratio < 1.6) beta = 2.28 * pow(ratio, -1.3);
  else beta = pow(10.0, -0.4 + 0.8393 * exp(-0.567 * log(ratio * ratio)));
  beta /= uSpread;                               // choppiness widens the directional spread
  float sh = 1.0 / cosh(clamp(beta * theta, -20.0, 20.0));
  return beta / (2.0 * tanh(beta * PI)) * sh * sh;
}
float cosSpread(float theta, float s) {
  float q = sqrt(s + 0.25) / (2.0 * sqrt(PI));
  return q * pow(abs(cos(theta * 0.5)), 2.0 * s);
}
float spectrum(vec2 k) {
  float kl = length(k);
  float w = omegaOf(kl);
  float theta = atan(k.y, k.x);
  float U = uWind.x, F = uWind.z;
  float Sw = 0.0;
  if (U > 0.1) {
    float alpha = 0.076 * pow(U * U / (F * GRAV), 0.22);
    float wp = 22.0 * pow(GRAV * GRAV / (U * F), 1.0 / 3.0);
    Sw = jonswap(w, wp, alpha, uWind.w) * donelanBanner(wrapAngle(theta - uWind.y), w, wp);
  }
  if (uSwell.x > 0.001) {
    float ws = uSwell.z, gs = 6.0, hs4 = uSwell.x * 0.25;
    float alphaS = hs4 * hs4 * 5.0 * ws * ws * ws * ws / (GRAV * GRAV) * (1.0 - 0.287 * log(gs));
    Sw += jonswap(w, ws, alphaS, gs) * cosSpread(wrapAngle(theta - uSwell.y), uSwell.w);
  }
  Sw *= exp(-kl * kl * uMisc.y * uMisc.y);
  return Sw * dOmegaOf(kl) / kl * uGain;
}
float amplitude(vec2 k) {
  float kl = length(k);
  if (kl < 1e-5 || kl < uKMin || kl >= uKMax) return 0.0;
  float dk = 2.0 * PI / uL;
  float g = uCasc >= 4.0 ? 0.85 * uFineGain : (uCasc >= 3.0 ? mix(1.0, uFineGain, 0.5) : 1.0);
  return sqrt(max(spectrum(k), 0.0)) * dk * 0.5 * g;
}
void main() {
  ivec2 id = ivec2(gl_FragCoord.xy);
  vec2 k = waveVec(id, uL);
  int mx = (NFFT - id.x) % NFFT;
  int my = (NFFT - id.y) % NFFT;
  uint seed = uint(uMisc.w) * 7919u + uint(uCasc) * 1000003u;
  vec2 a = gauss2(seed + uint(id.y * NFFT + id.x)) * amplitude(k);
  vec2 b = gauss2(seed + uint(my * NFFT + mx)) * amplitude(-k);
  fragColor = vec4(a, b.x, -b.y);
}`;

// time evolution -> six complex fields packed in three RGBA targets
const EVOLVE_FRAG = SIM_COMMON + /* glsl */`
uniform sampler2D uH0;
uniform float uL, uTime, uChoppy;
layout(location = 0) out vec4 oA;
layout(location = 1) out vec4 oB;
layout(location = 2) out vec4 oC;
void main() {
  ivec2 id = ivec2(gl_FragCoord.xy);
  vec2 k = waveVec(id, uL);
  float kl = length(k);
  if (kl < 1e-5) { oA = vec4(0.0); oB = vec4(0.0); oC = vec4(0.0); return; }
  float w0 = 2.0 * PI / ${LOOP}.0;
  float w = floor(omegaOf(kl) / w0) * w0;
  float ph = w * uTime;
  vec2 e = vec2(cos(ph), sin(ph));
  vec4 H0 = texelFetch(uH0, id, 0);
  vec2 hp = cmul(H0.xy, vec2(e.x, -e.y));
  vec2 hm = cmul(H0.zw, e);
  vec2 h = hp + hm;
  vec2 ht = mulI(hm - hp) * w;                  // dh/dt
  vec2 ih = mulI(h);
  float lam = uChoppy, ik = 1.0 / kl;
  vec2 Dx = ih * (lam * k.x * ik), Dz = ih * (lam * k.y * ik);
  vec2 jxz = h * (-lam * k.x * k.y * ik);
  vec2 sx = ih * k.x, sz = ih * k.y;
  vec2 jxx = h * (-lam * k.x * k.x * ik), jzz = h * (-lam * k.y * k.y * ik);
  vec2 iht = mulI(ht);                          // horizontal orbital velocity = i k^ dh/dt
  oA = vec4(Dx + mulI(Dz), h + mulI(jxz));
  oB = vec4(sx + mulI(sz), jxx + mulI(jzz));
  oC = vec4(iht * (k.x * ik) + mulI(iht * (k.y * ik)), ht);
}`;

// one Stockham radix-2 pass of the inverse FFT (natural order in, natural order out)
const FFT_BODY = /* glsl */`
uniform sampler2D uIn0, uIn1, uIn2;
uniform float uSize;
uniform bool uHorizontal;
void butterfly(out vec4 A, out vec4 B, out vec4 C) {
  ivec2 p = ivec2(gl_FragCoord.xy);
  int idx = uHorizontal ? p.x : p.y;
  int size = int(uSize), hs = size / 2;
  int ev = (idx / size) * hs + (idx % hs);
  int od = ev + NFFT / 2;
  ivec2 pe = uHorizontal ? ivec2(ev, p.y) : ivec2(p.x, ev);
  ivec2 po = uHorizontal ? ivec2(od, p.y) : ivec2(p.x, od);
  float arg = 2.0 * PI * float(idx) / uSize;
  vec2 w = vec2(cos(arg), sin(arg));
  vec4 e0 = texelFetch(uIn0, pe, 0), o0 = texelFetch(uIn0, po, 0);
  vec4 e1 = texelFetch(uIn1, pe, 0), o1 = texelFetch(uIn1, po, 0);
  vec4 e2 = texelFetch(uIn2, pe, 0), o2 = texelFetch(uIn2, po, 0);
  A = vec4(e0.xy + cmul(w, o0.xy), e0.zw + cmul(w, o0.zw));
  B = vec4(e1.xy + cmul(w, o1.xy), e1.zw + cmul(w, o1.zw));
  C = vec4(e2.xy + cmul(w, o2.xy), e2.zw + cmul(w, o2.zw));
}`;
const FFT_FRAG = SIM_COMMON + FFT_BODY + /* glsl */`
layout(location = 0) out vec4 oA;
layout(location = 1) out vec4 oB;
layout(location = 2) out vec4 oC;
void main() { butterfly(oA, oB, oC); }`;
// last column pass writes the filterable, mip-mapped textures used for rendering
const FINAL_FRAG = SIM_COMMON + FFT_BODY + /* glsl */`
layout(location = 0) out vec4 oDisp;    // Dx, h, Dz, vertical velocity
layout(location = 1) out vec4 oDeriv;   // slope x, slope z, dDx/dx, dDz/dz
layout(location = 2) out vec4 oMom;     // sx^2, sz^2, sx*sz, dDx/dz  (LEAN moments)
layout(location = 3) out vec4 oVel;     // horizontal orbital velocity
void main() {
  vec4 A, B, C;
  butterfly(A, B, C);
  oDisp = vec4(A.x, A.z, A.y, C.z);
  oDeriv = B;
  oMom = vec4(B.x * B.x, B.y * B.y, B.x * B.y, A.w);
  oVel = vec4(C.x, C.y, 0.0, 0.0);
}`;

const simRT = (count, o = {}) => new THREE.WebGLRenderTarget(N, N, {
  count, type: THREE.FloatType, depthBuffer: false, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter, generateMipmaps: false, ...o,
});
const h0RT = CASC_L.map(() => simRT(1));
const pingRT = simRT(3), pongRT = simRT(3);
const finalRT = CASC_L.map(() => simRT(4, {
  type: THREE.HalfFloatType, minFilter: THREE.LinearMipmapLinearFilter, magFilter: THREE.LinearFilter,
  wrapS: THREE.RepeatWrapping, wrapT: THREE.RepeatWrapping, generateMipmaps: true,
}));
const DISP = finalRT.map(r => r.textures[0]), DERIV = finalRT.map(r => r.textures[1]);
const MOM = finalRT.map(r => r.textures[2]), VEL = finalRT.map(r => r.textures[3]);

const initMat = passMat(INIT_FRAG, {
  uL: { value: 1 }, uKMin: { value: 0 }, uKMax: { value: 1 }, uCasc: { value: 0 }, uGain: { value: 1 },
  uSpread: { value: 1 }, uFineGain: { value: 1 },
  uWind: { value: new THREE.Vector4() }, uSwell: { value: new THREE.Vector4() }, uMisc: { value: new THREE.Vector4(0, 0.0012, LOOP, 1) },
});
const evolveMat = passMat(EVOLVE_FRAG, { uH0: { value: null }, uL: { value: 1 }, uTime: { value: 0 }, uChoppy: { value: 1.25 } });
const fftUniforms = () => ({ uIn0: { value: null }, uIn1: { value: null }, uIn2: { value: null }, uSize: { value: 2 }, uHorizontal: { value: true } });
const fftMat = passMat(FFT_FRAG, fftUniforms());
const finalMat = passMat(FINAL_FRAG, fftUniforms());
let needSpectrum = true;

const sea = { wp: 1, kp: 0.1, Hs: 0, Tp: 0, lp: 0, toward: [1, 0], current: { x: 0, z: 0 }, wind: { x: 0, z: 0 } };
function jonswapCPU(w, wp, alpha, gamma) {
  const sigma = w <= wp ? 0.07 : 0.09, d = (w - wp) / (sigma * wp), q = wp / w;
  return alpha * G * G / w ** 5 * Math.exp(-1.25 * q ** 4) * gamma ** Math.exp(-0.5 * d * d);
}
function initSpectrum() {
  const U = Math.max(params.windSpeed, 0.2), F = params.fetchKm * 1000;
  const [tx, tz] = bearingVec(params.windFrom + 180);
  const [sx, sz] = bearingVec(params.swellFrom + 180);
  const u = initMat.uniforms;
  u.uWind.value.set(U, Math.atan2(tz, tx), F, 3.3);
  u.uSwell.value.set(params.swellHeight, Math.atan2(sz, sx), Math.sqrt(G * 2 * Math.PI / params.swellLength), 48);
  u.uGain.value = params.waveHeight ** 2;
  u.uSpread.value = 0.5 + params.choppiness;
  u.uFineGain.value = 0.6 + 0.8 * params.choppiness;
  for (let c = 0; c < NC; c++) {
    const L = CASC_L[c];
    const kMax = c < NC - 1 ? 2 * Math.PI * 6 / CASC_L[c + 1] : Math.PI * N / L * 0.98;
    const kMin = c === 0 ? Math.PI / L : 2 * Math.PI * 6 / CASC_L[c];
    u.uL.value = L; u.uKMin.value = kMin; u.uKMax.value = kMax; u.uCasc.value = c;
    runPass(initMat, h0RT[c]);
  }
  evolveMat.uniforms.uChoppy.value = params.steepness;
  // sea-state numbers for the HUD and the body physics
  const alpha = 0.076 * Math.pow(U * U / (F * G), 0.22), wp = 22 * Math.pow(G * G / (U * F), 1 / 3);
  let m0 = 0;
  for (let i = 1; i <= 800; i++) m0 += jonswapCPU(i * 0.01, wp, alpha, 3.3) * 0.01;
  const HsW = 4 * Math.sqrt(m0) * params.waveHeight, HsS = params.swellHeight * params.waveHeight;
  Object.assign(sea, { wp, kp: wp * wp / G, Hs: Math.hypot(HsW, HsS), HsW, HsS, Tp: 2 * Math.PI / wp, lp: 2 * Math.PI * G / (wp * wp), toward: [tx, tz] });
  // surface drift: Stokes drift (~1.25 % U) + wind-driven current (~1.5 % U), the classic ~3 % rule
  sea.current.x = 0.0275 * U * tx; sea.current.z = 0.0275 * U * tz;
  sea.wind.x = 0.8 * U * tx; sea.wind.z = 0.8 * U * tz;
  needSpectrum = false;
}

function simulateWaves(t) {
  if (needSpectrum) initSpectrum();
  for (let c = 0; c < NC; c++) {
    evolveMat.uniforms.uH0.value = h0RT[c].texture;
    evolveMat.uniforms.uL.value = CASC_L[c];
    evolveMat.uniforms.uTime.value = t % LOOP;
    runPass(evolveMat, pingRT);
    let src = pingRT, dst = pongRT;
    for (let pass = 0; pass < 16; pass++) {
      const last = pass === 15;
      const m = last ? finalMat : fftMat;
      m.uniforms.uIn0.value = src.textures[0]; m.uniforms.uIn1.value = src.textures[1]; m.uniforms.uIn2.value = src.textures[2];
      m.uniforms.uSize.value = 2 << (pass % 8);
      m.uniforms.uHorizontal.value = pass < 8;
      runPass(m, last ? finalRT[c] : dst);
      [src, dst] = [dst, src];
    }
  }
}

// ---- procedural foam micro-structure (tileable): R lace, G fine lace, B soft density, A streaks
const FOAMTEX_FRAG = NOISE_GLSL + /* glsl */`
layout(location = 0) out vec4 fragColor;
vec2 h2(ivec2 i, uint salt) {
  uint h = pcgHash((uint(i.x) * 73856093u) ^ pcgHash(uint(i.y) * 19349663u + salt));
  uint hv = pcgHash(h + 0x9e3779b9u);
  return vec2(float(h >> 8u), float(hv >> 8u)) / 16777216.0;
}
float bubbles(vec2 uv, int period, uint salt, float rmin, float rmax) {
  vec2 w = vec2(pfbm(uv * float(period) * 0.5, period / 2, 3), pfbm(uv * float(period) * 0.5 + 17.0, period / 2, 3));
  vec2 p = uv * float(period) + w * 0.9;
  ivec2 ip = ivec2(floor(p));
  vec2 fp = p - floor(p);
  float v = 9.0;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      ivec2 o = ivec2(i, j);
      ivec2 cell = ivec2(imod(ip.x + i, period), imod(ip.y + j, period));
      vec2 r = vec2(o) + h2(cell, salt) * 0.9 + 0.05 - fp;
      float rad = mix(rmin, rmax, h2(cell, salt + 7u).x);
      v = min(v, length(r) / rad);
    }
  }
  return smoothstep(0.8, 1.25, v);
}
void main() {
  vec2 uv = gl_FragCoord.xy / 512.0;
  vec2 w1 = vec2(pfbm(uv * 5.0, 5, 3), pfbm(uv * 5.0 + vec2(3.7, 9.1), 5, 3));
  float m1 = smoothstep(-0.14, 0.06, pfbm(uv * 10.0 + w1 * 1.4, 10, 5));
  vec2 w2 = vec2(pfbm(uv * 12.0 + 1.3, 12, 2), pfbm(uv * 12.0 + 7.9, 12, 2));
  float m2 = smoothstep(-0.16, 0.04, pfbm(uv * 27.0 + w2 * 0.9, 27, 4));
  float b3 = bubbles(uv, 29, 37u, 0.06, 0.45);
  float b4 = bubbles(uv, 61, 53u, 0.05, 0.42);
  float dens = pfbm(uv * 4.0, 4, 5) * 0.5 + 0.5;
  float fineD = pfbm(uv * 16.0, 16, 3) * 0.5 + 0.5;
  float big = m1 * (0.55 + 0.45 * m2) * (0.75 + 0.25 * b3);
  float fine = 0.45 + 0.55 * b3 * (0.7 + 0.3 * b4);
  float streak = pfbm(uv * vec2(4.0, 16.0), 4, 5) * 0.5 + 0.5;
  fragColor = vec4(clamp(big * 0.6 + dens * 0.4, 0.0, 1.0), clamp(fine * 0.8 + fineD * 0.2, 0.0, 1.0), dens, streak);
}`;
const foamTexRT = target(512, 512, {
  type: THREE.UnsignedByteType, wrapS: THREE.RepeatWrapping, wrapT: THREE.RepeatWrapping,
  minFilter: THREE.LinearMipmapLinearFilter, generateMipmaps: true,
});
runPass(passMat(FOAMTEX_FRAG, {}), foamTexRT);
const FOAM_TEX = foamTexRT.texture;

// ---- world-space foam map: injected where the surface folds (and around bodies), decays and drifts downwind
const CASC_SAMPLERS = (names) => names.map(n => Array.from({ length: NC }, (_, c) => `uniform sampler2D ${n}${c};`).join('\n')).join('\n');
// GLSL ES has no token pasting, so per-cascade code is unrolled here
const forC = f => CASC_L.map((_, c) => f(c)).join('\n');
const FOAMUPD_FRAG = CASC_SAMPLERS(['uDeriv', 'uMom']) + /* glsl */`
#define NC ${NC}
#define MAXP ${MAX_PEOPLE}
uniform sampler2D uPrev;
uniform float uCascL[NC];
uniform vec2 uCenter, uPrevCenter, uDrift;
uniform float uSize, uDt, uDecay, uJThr, uGain, uGrowth;
uniform vec4 uPeople[MAXP];   // lagrangian x, z, radius, strength
uniform int uPeopleCount;
layout(location = 0) out vec4 fragColor;
float hash(vec2 p) { vec3 p3 = fract(vec3(p.xyx) * 0.1031); p3 += dot(p3, p3.yzx + 33.33); return fract((p3.x + p3.y) * p3.z); }
void main() {
  float ts = uSize / ${FOAM_RES}.0;
  vec2 p = uCenter + (gl_FragCoord.xy / ${FOAM_RES}.0 - 0.5) * uSize;
  float jxx = 0.0, jzz = 0.0, jxz = 0.0;
  ${forC(c => `{ float lod = log2(max(ts / (uCascL[${c}] / ${N}.0), 1.0));
    if (lod < 7.5) { vec2 uv = p / uCascL[${c}]; vec4 d = textureLod(uDeriv${c}, uv, lod); jxx += d.z; jzz += d.w; jxz += textureLod(uMom${c}, uv, lod).w; } }`)}
  float J = (1.0 + jxx) * (1.0 + jzz) - jxz * jxz;
  vec2 back = (p - uDrift * uDt - uPrevCenter) / uSize + 0.5;
  vec4 prev = (all(greaterThan(back, vec2(0.0))) && all(lessThan(back, vec2(1.0)))) ? texture(uPrev, back) : vec4(0.0);
  float inject = clamp((uJThr - J) * uGain, 0.0, 1.0);
  // bodies breaking the surface: thin, broken ring of bubbles at the waterline (no build-up)
  float body = 0.0;
  for (int i = 0; i < MAXP; i++) {
    if (i >= uPeopleCount) break;
    vec4 b = uPeople[i];
    float d = length(p - b.xy);
    float ring = smoothstep(b.z + 0.16, b.z + 0.03, d) * smoothstep(b.z - 0.1, b.z, d);
    float n = hash(floor(p * 11.0) + vec2(float(i) * 13.0, 0.0));
    body = max(body, b.w * ring * step(0.5, n));
  }
  float f = prev.r * exp(-uDt * uDecay);
  f = max(max(f, inject), body);
  f = min(f + inject * uDt * uGrowth, 1.6);
  float fresh = max(prev.g * exp(-uDt * 1.6), inject);
  fragColor = vec4(f, fresh, J, 1.0);
}`;
const foamA = target(FOAM_RES, FOAM_RES), foamB = target(FOAM_RES, FOAM_RES);
let foamSrc = foamA, foamDst = foamB;
const foamUpdMat = passMat(FOAMUPD_FRAG, {
  uPrev: { value: null }, uCascL: { value: CASC_L }, uCenter: { value: new THREE.Vector2() }, uPrevCenter: { value: new THREE.Vector2(1e6, 1e6) },
  uDrift: { value: new THREE.Vector2() }, uSize: { value: FOAM_SIZE }, uDt: { value: 0 }, uDecay: { value: 0.22 },
  uJThr: { value: 0.6 }, uGain: { value: 2 }, uGrowth: { value: 1 },
  uPeople: { value: Array.from({ length: MAX_PEOPLE }, () => new THREE.Vector4()) }, uPeopleCount: { value: 0 },
  ...Object.fromEntries(CASC_L.flatMap((_, c) => [[`uDeriv${c}`, { value: DERIV[c] }], [`uMom${c}`, { value: MOM[c] }]])),
});
const foamCenter = new THREE.Vector2();
function updateFoam(dt) {
  const ts = FOAM_SIZE / FOAM_RES, u = foamUpdMat.uniforms;
  u.uPrevCenter.value.copy(u.uCenter.value);
  u.uCenter.value.set(Math.round(controls.target.x / ts) * ts, Math.round(controls.target.z / ts) * ts);
  u.uDt.value = dt;
  u.uJThr.value = 0.52 + 0.16 * params.foam;
  const drift = 0.03 * params.windSpeed;
  u.uDrift.value.set(sea.toward[0] * drift, sea.toward[1] * drift);
  u.uPrev.value = foamSrc.texture;
  runPass(foamUpdMat, foamDst);
  [foamSrc, foamDst] = [foamDst, foamSrc];
  foamCenter.copy(u.uCenter.value);
}

// ---- surface probes: exact height / slope / velocity at query points, read back for the buoyancy
const PROBE_FRAG = CASC_SAMPLERS(['uDisp', 'uDeriv', 'uVel']) + /* glsl */`
#define NC ${NC}
uniform sampler2D uQuery;
uniform float uCascL[NC];
layout(location = 0) out vec4 fragColor;
vec4 dispAt(vec2 xz) { vec4 d = vec4(0.0); ${forC(c => `d += textureLod(uDisp${c}, xz / uCascL[${c}], 0.0);`)} return d; }
void main() {
  ivec2 p = ivec2(gl_FragCoord.xy);
  vec2 goal = texelFetch(uQuery, ivec2(p.x, 0), 0).xy;
  vec2 x0 = goal;
  for (int i = 0; i < 5; i++) x0 = goal - dispAt(x0).xz;    // invert the horizontal displacement
  if (p.y == 0) {
    vec4 d = vec4(0.0);
    ${forC(c => `d += textureLod(uDeriv${c}, x0 / uCascL[${c}], 0.0);`)}
    float J = (1.0 + d.z) * (1.0 + d.w);
    fragColor = vec4(dispAt(x0).y, d.x / max(1.0 + d.z, 0.2), d.y / max(1.0 + d.w, 0.2), J);
  } else if (p.y == 1) {
    vec4 v = vec4(0.0);
    ${forC(c => `v += textureLod(uVel${c}, x0 / uCascL[${c}], 0.0);`)}
    fragColor = vec4(v.x, dispAt(x0).w, v.y, 0.0);
  } else {
    fragColor = vec4(x0, 0.0, 0.0);
  }
}`;
const queryData = new Float32Array(MAX_PROBES * 4);
const queryTex = new THREE.DataTexture(queryData, MAX_PROBES, 1, THREE.RGBAFormat, THREE.FloatType);
const probeRT = target(MAX_PROBES, 3, { type: THREE.FloatType, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter });
const probeMat = passMat(PROBE_FRAG, {
  uQuery: { value: queryTex }, uCascL: { value: CASC_L },
  ...Object.fromEntries(CASC_L.flatMap((_, c) => [[`uDisp${c}`, { value: DISP[c] }], [`uDeriv${c}`, { value: DERIV[c] }], [`uVel${c}`, { value: VEL[c] }]])),
});
const probeBuf = new Float32Array(MAX_PROBES * 3 * 4);
let probeTime = 0;
function runProbes(t) {
  queryTex.needsUpdate = true;
  runPass(probeMat, probeRT);
  renderer.readRenderTargetPixels(probeRT, 0, 0, MAX_PROBES, 3, probeBuf);
  probeTime = t;
}
// probe i: h, slope x, slope z, J | vx, vy, vz | lagrangian x0, z0
const probe = i => ({
  h: probeBuf[i * 4], sx: probeBuf[i * 4 + 1], sz: probeBuf[i * 4 + 2],
  vx: probeBuf[(MAX_PROBES + i) * 4], vy: probeBuf[(MAX_PROBES + i) * 4 + 1], vz: probeBuf[(MAX_PROBES + i) * 4 + 2],
  x0: probeBuf[(2 * MAX_PROBES + i) * 4], z0: probeBuf[(2 * MAX_PROBES + i) * 4 + 1],
});

