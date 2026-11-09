// ============================================================================
// LiDAR (feature module "sensor"): a time-of-flight laser scanner on the EO/IR payload. Its point cloud is drawn
// in a third window, to the right of the LWIR window.
// ============================================================================
// Scanning: the beam directions are generated analytically. A spinning multi-beam scanner has channels x columns
// per turn; a solid-state scanner draws a non-repetitive Risley-prism rosette, or (DJI L2-type, repetitive mode)
// six lines swept across a 70° x 3° field. Each beam samples a
// range / reflectance image of the real scene geometry rendered from the sensor: the displaced FFT ocean, the
// skinned people and the gear.
// Return physics, per shot:
//   - water: monostatic specular backscatter from the wave facets that face the sensor (Kirchhoff / geometric
//     optics with the LEAN slope covariance, Fresnel R(0) ~ 0.02), plus a tiny volume term. At grazing angles
//     almost nothing comes back. Near nadir the glints are strong and flicker from shot to shot (exponential
//     speckle);
//   - whitecaps and foam: Lambertian, NIR reflectance ~0.25 at 905 nm and ~0.08 at 1550 nm;
//   - people and gear: Lambertian reflectance per material at the laser wavelength:
//       skin ~0.45 / 0.07 (905 / 1550 nm), wet textiles darker, coated orange fabric ~0.65;
//       retroreflective SOLAS tape returns ~1000x a white card at normal incidence, saturates the receiver and
//       blooms into neighbouring shots;
//   - link budget: signal ~ rho_eff T^2 / R^2, normalised to the datasheet "range at 10 % reflectivity" (R10,
//     90 % detection probability), with daylight background noise, near-field overlap, a Weibull detection
//     probability and range noise that grows at low SNR;
//   - atmosphere: extinction from the sea-fog / haze visibility (Kim's wavelength exponent) and the rain rate;
//     fog and rain clutter near the sensor; rare solar false alarms.
// Multiple returns (up to 5 per pulse, as on the DJI Zenmuse L2: 240 k pulses/s -> up to 1.2 M points/s): the
// footprint is sampled at five points, each with its share of the pulse energy. Samples on one surface merge
// into one echo; separate surfaces give separate echoes, for example a head and the sea behind it, or a raft
// edge. Spray above breaking crests and fog or rain droplets add echoes in front of the water. Each echo is
// detected on its own and stored in range order (first return, second return ...).
// The points are stored in world coordinates on the GPU (no readback), so the cloud stays registered while the
// sensor moves. "Export point cloud" reads them back once and saves an ASCII .pcd in the sensor frame.
// kind 0 spinning, 1 non-repetitive (rosette), 2 repetitive (line sweep); fovX / fovY full field (deg);
// maxRet = returns the sensor can record per pulse, defRet = returns selected when the model is picked.
// DJI L2-type: 905 nm, 240 kHz pulses, up to 5 returns, 250 m at 10 % in 100 klx sunlight (R10 here is without
// sunlight; the daylight noise model brings it to ~240 m at noon), 450 m maximum, 3 m blind zone, 2 cm range
// noise, 4 x 12 cm spot at 100 m (0.4 x 1.2 mrad); fields 70° x 75° (non-repetitive) or 70° x 3° (repetitive).
const LIDAR_MODELS = [
  { name: 'Solid-state rosette 70° (240k pts/s)', short: 'ROSETTE 70°', kind: 1, fovX: 70.4, fovY: 70.4, rate: 240000, R10: 190, Rmax: 450, Rmin: 1.0, sigma: 0.02, div: 0.28, maxRet: 3 },
  { name: 'Solid-state long range 15° (240k pts/s)', short: 'ROSETTE 15°', kind: 1, fovX: 15, fovY: 15, rate: 240000, R10: 320, Rmax: 500, Rmin: 3.0, sigma: 0.02, div: 0.07, maxRet: 2 },
  { name: 'DJI L2-type, non-repetitive 70°×75° (up to 5 returns)', short: 'L2 NON-REP 70°×75°', kind: 1, fovX: 70, fovY: 75, rate: 240000, R10: 300, Rmax: 450, Rmin: 3.0, sigma: 0.02, div: 0.046, maxRet: 5, defRet: 5 },
  { name: 'DJI L2-type, repetitive 70°×3° (up to 5 returns)', short: 'L2 REP 70°×3°', kind: 2, fovX: 70, fovY: 3, rate: 240000, R10: 300, Rmax: 450, Rmin: 3.0, sigma: 0.02, div: 0.046, maxRet: 5, defRet: 5 },
  { name: 'Spinning 32-beam 360°', short: '32-CH 360°', kind: 0, ch: 32, cols: 1024, elMin: -16, elMax: 15, R10: 60, Rmax: 120, Rmin: 0.5, sigma: 0.03, div: 0.18, maxRet: 2 },
  { name: 'Spinning 64-beam 360°', short: '64-CH 360°', kind: 0, ch: 64, cols: 1024, elMin: -22.5, elMax: 22.5, R10: 80, Rmax: 150, Rmin: 0.5, sigma: 0.025, div: 0.18, maxRet: 2 },
  { name: 'Spinning 128-beam long range', short: '128-CH 360°', kind: 0, ch: 128, cols: 2048, elMin: -11.25, elMax: 11.25, R10: 150, Rmax: 240, Rmin: 0.8, sigma: 0.025, div: 0.1, maxRet: 2 },
];
Object.assign(params, {
  lidar: true, lidarModel: 0, lidarBand: 905, lidarR10: 190, lidarRmax: 450, lidarHz: 10, lidarInteg: 1.0, lidarTilt: 0, lidarLevel: false, lidarReturns: 1,
  lidarView: 0, lidarYaw: 0, lidarPitch: 35, lidarZoom: 1, lidarColor: 0, lidarPtSize: 2, lidarClutter: true, lidarBloom: true, lidarNoise: 1,
  lidarFrame: 0, lidarExportFmt: 'pcd',
});
const LIDAR_RT_W = 1024, LIDAR_RT_H = 512, LIDAR_MAXN = 1024 * 256 * 5;
const LIDAR_W1 = 2 * Math.PI * 121.3, LIDAR_W2 = -2 * Math.PI * 109.7;   // Risley prism rates (rad/s), counter-rotating
const LIDAR_U = { uLidar: { value: 0 }, uLidarBand: { value: 0 }, uLidarRetro: { value: 300 } };

// ---------------------------------------------------------------------------- ocean: range, reflectance, class
// output: R range (m), G effective reflectance rho_eff (= pi f_r cos, so a Lambertian card gives rho cos),
// B class (1 water ... 1.99 foam), A 1 = surface hit
Object.assign(oceanU, { uLidar: LIDAR_U.uLidar, uLidarBand: LIDAR_U.uLidarBand });
injectShader(ocean.material, 'ocean-frag-decl', 'uniform float uLidar, uLidarBand;');
injectShader(ocean.material, 'ocean-frag-normal', /* glsl */`
  if (uLidar > 0.5) {
    // ---- LiDAR (sensor_lidar.js): foam coverage, same model as the visible shading below
    float lw0 = 1.0 - smoothstep(0.22 * uFoamSize, 0.48 * uFoamSize, length(xz - uFoamCenter));
    vec4 lfw = texture(uFoamWorld, (xz - uFoamCenter) / uFoamSize + 0.5) * lw0;
    float lfresh = clamp((uFoamP.x - J) * uFoamP.y, 0.0, 1.0);
    float lcover = clamp((max(lfw.r, lfresh * (1.0 - lw0 * 0.7)) + lfw.g * 0.6) * uFoamP.z, 0.0, 1.5);
    vec2 lws = vec2(dot(xz, uWind.xy), dot(xz, vec2(-uWind.y, uWind.x)));
    float lstreak = texture(uFoamTex, lws * vec2(0.035, 0.11)).a;
    lcover *= mix(smoothstep(0.25, 0.75, lstreak) * 1.5, 1.0, clamp(lfw.g * 2.0 + lfresh, 0.0, 1.0));
    float lpat = foamPattern(xz, uTime);
    float lfm = smoothstep(1.0 - lcover - 0.06, 1.0 - lcover + 0.06, lpat) * smoothstep(0.0, 0.5, lcover);
    float lfoam = clamp(max(lfm, smoothstep(0.35, 0.9, lfw.g) * smoothstep(0.25, 0.6, lpat + 0.25)), 0.0, 1.0);
    // monostatic specular backscatter: the facets whose normal points back at the sensor (h = v), Gaussian slope
    // pdf around the resolved slope with the unresolved covariance; shadowing = masking for coincident rays
    float lvy = max(V.y, 0.02);
    vec2 lds = -V.xz / lvy - sl;
    float ldet = max(cov.x * cov.y - cov.z * cov.z, 1e-10);
    float le = (lds.x * lds.x * cov.y + lds.y * lds.y * cov.x - 2.0 * lds.x * lds.y * cov.z) / ldet;
    float lpdf = exp(-0.5 * le) / (2.0 * PI * sqrt(ldet));
    float lG = 1.0 / (1.0 + lambdaSmith(V, cov));
    float lF = mix(0.0200, 0.0190, uLidarBand);                 // R(0), n = 1.328 (905 nm) / 1.318 (1550 nm)
    float lspec = min(PI * lF * lpdf * lG / (4.0 * lvy * lvy * lvy * lvy * lvy), 60.0);
    float lbody = mix(5e-4, 5e-5, uLidarBand) * lvy;              // water-leaving (volume) part: NIR Rrs ~1e-4 /sr; absorption ~7 /m (905), ~1000 /m (1550)
    float lfoamR = mix(0.25, 0.08, uLidarBand) * max(dot(nrm, V), 0.0);
    fragColor = vec4(dist, mix(lspec + lbody, lfoamR, lfoam), 1.0 + 0.99 * lfoam, 1.0);
    return;
  }`);

// ---------------------------------------------------------------------------- people and gear (wet materials)
// reflectance at 905 / 1550 nm, retroreflector flag, how much wetting darkens it
function lidarMatProps(mat, cls) {
  const c = (mat.color || new THREE.Color(1, 1, 1)).clone().convertLinearToSRGB();
  const mx = Math.max(c.r, c.g, c.b);
  if (cls === 6 && !mat.isMeshPhysicalMaterial && mat.metalness >= 0.55) return [0.6, 0.5, 1, 0];     // retroreflective tape
  if (mat.emissive && mat.emissive.getHex() !== 0) return [0.25, 0.2, 0, 0];                            // strobe / lamp lens
  if (cls === 2 || cls === 4) return [0.45, 0.07, 0, 0.25];                                              // skin
  if (cls === 3) return [0.12, 0.05, 0, 0.5];                                                            // hair
  if (cls === 5) return [0.12 + 0.45 * mx, 0.04 + 0.1 * mx, 0, 1];                                       // textiles, straps
  if (cls === 7) return [-1, -1, 0, 1];                                                                  // single-texture model
  return [clamp(0.06 + 0.62 * mx, 0.05, 0.75), clamp(0.04 + 0.4 * mx, 0.03, 0.5), 0, 0.5];              // coated fabric, plastic, wood
}
HOOKS.personShader.push((sh, mat, cls) => {
  const isPerson = people.some(P => P.mats.includes(mat));
  Object.assign(sh.uniforms, LIDAR_U, {
    uLidarMat: { value: new THREE.Vector4(...lidarMatProps(mat, cls)) }, uLidarCls: { value: isPerson ? 3 : 5 },
  });
  sh.fragmentShader = 'uniform float uLidar, uLidarBand, uLidarRetro, uLidarCls;\nuniform vec4 uLidarMat;\n' +
    sh.fragmentShader.replace('#include <dithering_fragment>', `#include <dithering_fragment>
  if (uLidar > 0.5) {
    float lR = length(vViewPosition);
    float lC = clamp(abs(dot(normalize(normal), vViewPosition / lR)), 0.0, 1.0);
    vec4 lm = uLidarMat;
    float lRho = mix(lm.x, lm.y, uLidarBand);
    if (lm.x < 0.0) {                     // one texture for skin and clothes: skin around the head
      float lMx = pow(max(max(diffuseColor.r, diffuseColor.g), diffuseColor.b), 0.4545);
      lRho = distance(vWetPos, uHeadPos) < 0.13 ? mix(0.45, 0.07, uLidarBand) : mix(0.12 + 0.45 * lMx, 0.04 + 0.1 * lMx, uLidarBand);
    }
    lRho *= 1.0 - lm.w * wetness * mix(0.4, 0.55, uLidarBand);
    float lEff = lRho * lC, lCls = uLidarCls;
    if (lm.z > 0.5) { lEff += uLidarRetro * lC * lC; lCls = 4.0; }
    gl_FragColor = vec4(lR, lEff, lCls, 1.0);
  }`);
});

// solid objects that are not in the water (the rescue vessel): painted surfaces
const lidarSolidCache = new Map();
function lidarSolidMat(src) {
  let m = lidarSolidCache.get(src);
  if (m) return m;
  const c = (src.color || new THREE.Color(1, 1, 1)).clone().convertLinearToSRGB();
  const mx = Math.max(c.r, c.g, c.b);
  m = new THREE.ShaderMaterial({
    glslVersion: THREE.GLSL3, side: src.side,
    uniforms: { uRho: { value: new THREE.Vector2(clamp(0.08 + 0.6 * mx, 0.05, 0.8), clamp(0.05 + 0.4 * mx, 0.04, 0.6)) }, uLidarBand: LIDAR_U.uLidarBand },
    vertexShader: 'out vec3 vVP; out vec3 vN;\nvoid main() { vec4 mv = modelViewMatrix * vec4(position, 1.0); vVP = -mv.xyz; vN = normalMatrix * normal; gl_Position = projectionMatrix * mv; }',
    fragmentShader: 'uniform vec2 uRho; uniform float uLidarBand; in vec3 vVP; in vec3 vN; layout(location = 0) out vec4 o;\n' +
      'void main() { float r = length(vVP); o = vec4(r, mix(uRho.x, uRho.y, uLidarBand) * abs(dot(normalize(vN), vVP / r)), 5.0, 1.0); }',
  });
  lidarSolidCache.set(src, m);
  return m;
}

// ---------------------------------------------------------------------------- render targets and passes
const lidarRT = new THREE.WebGLRenderTarget(LIDAR_RT_W, LIDAR_RT_H, {
  type: THREE.FloatType, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter, generateMipmaps: false,
});
lidarRT.scissorTest = true;
const lidarCam = new THREE.PerspectiveCamera(70, 1, 0.2, 400);

const LIDAR_BEAM_FRAG = /* glsl */`
precision highp float;
precision highp int;
uniform sampler2D tL;          // range / reflectance image
uniform vec2 uLRes;            // used part of it (px)
uniform mat4 uVP;              // world -> clip of the camera that rendered it
uniform vec3 uPos;             // sensor origin (world)
uniform mat3 uRot;             // sensor frame -> world
uniform vec2 uBuf;             // point buffer size (texels)
uniform float uKind;           // 0 spinning, 1 non-repetitive (Risley rosette), 2 repetitive (6-line sweep)
uniform float uRet;            // returns recorded per pulse; a pulse owns uRet consecutive texels (rows when spinning)
uniform float uIdx0, uCount;   // solid-state: first pulse slot written by this pass, number of pulses
uniform vec2 uPh, uW;          // solid-state: scanner phases at the first pulse (rad), rates (rad/s)
uniform float uRate;           // pulses per second
uniform vec2 uFov;             // solid-state: half field of view, horizontal / vertical (rad)
uniform vec2 uEl;              // spinning: lowest / highest beam elevation (rad)
uniform vec4 uRange;           // min range, max range, R10, range noise sigma (m)
uniform vec4 uAtm;             // extinction (1/m), daylight SNR factor, fog clutter p, rain clutter p
uniform vec4 uMisc;            // frame seed, beam divergence (rad), render pixel angle (rad), blooming on
uniform vec2 uMisc2;           // solar false alarms per pulse, sea-spray probability per foam-covered hit
layout(location = 0) out vec4 outP;

uint lh(uint x) { x ^= x >> 16u; x *= 0x7feb352du; x ^= x >> 15u; x *= 0x846ca68bu; x ^= x >> 16u; return x; }
float lu(inout uint s) { s = lh(s); return (float(s >> 8u) + 0.5) * (1.0 / 16777216.0); }
float lg(inout uint s) { float a = lu(s), b = lu(s); return sqrt(-2.0 * log(a)) * cos(6.2831853 * b); }

vec4 sampleL(vec3 d, out bool inside) {
  vec4 c = uVP * vec4(uPos + d, 1.0);
  vec2 uv = c.xy / max(c.w, 1e-6) * 0.5 + 0.5;
  inside = c.w > 0.0 && all(greaterThanEqual(uv, vec2(0.0))) && all(lessThan(uv, vec2(1.0)));
  return inside ? texelFetch(tL, clamp(ivec2(uv * uLRes), ivec2(0), ivec2(uLRes) - 1), 0) : vec4(0.0);
}
// normalised SNR: 1 = a 10 % Lambertian target at R10, detected 90 % of the time
float snr(float rho, float R) {
  float T2 = exp(-2.0 * uAtm.x * R);
  float overlap = smoothstep(uRange.x * 0.5, uRange.x * 2.0, R);    // transmitter / receiver overlap near the sensor
  return rho * T2 * overlap * uRange.z * uRange.z / (0.1 * R * R) * uAtm.y;
}
float pod(float s) { float q = s / 0.659; return 1.0 - exp(-q * q); }

#define MAXE 10
void main() {
  ivec2 px = ivec2(gl_FragCoord.xy);
  int R = int(uRet + 0.5), pulse, ret;
  vec3 dirL;
  if (uKind > 0.5) {
    int t = px.y * int(uBuf.x) + px.x;
    pulse = t / R; ret = t - pulse * R;
    int k = pulse - int(uIdx0);
    if (k < 0 || k >= int(uCount)) discard;
    float tt = float(k) / uRate;
    vec2 d;
    if (uKind < 1.5) {
      // two counter-rotating prisms: a rose whose petals precess, filling the field more each cycle
      vec2 a = uPh + uW * tt;
      d = 0.5 * uFov * (vec2(cos(a.x), sin(a.x)) + vec2(cos(a.y), sin(a.y)));
    } else {
      // repetitive: six channels stacked across the narrow vertical field, swept back and forth horizontally
      float ch = float((int(uIdx0) + k) % 6);
      d = vec2(uFov.x * sin(uPh.x + uW.x * tt), uFov.y * ((ch + 0.5) / 3.0 - 1.0) + 0.0012 * sin(uPh.y + uW.y * tt));
    }
    float th = length(d), ph = atan(d.y, d.x + 1e-9);
    dirL = vec3(sin(th) * cos(ph), sin(th) * sin(ph), -cos(th));
  } else {
    int ch = px.y / R;
    ret = px.y - ch * R; pulse = ch * int(uBuf.x) + px.x;
    float az = (float(px.x) + 0.5) / uBuf.x * 6.2831853;
    float el = mix(uEl.x, uEl.y, (float(ch) + 0.5) / (uBuf.y / float(R)));
    dirL = vec3(sin(az) * cos(el), sin(el), -cos(az) * cos(el));
  }
  // every texel of one pulse draws the same random numbers, so they agree on the echo list
  uint seed = lh(uint(pulse) * 747796405u + lh(uint(uMisc.x)));
  vec3 dW = normalize(uRot * dirL);
  vec3 t1 = normalize(cross(dW, abs(dW.y) < 0.99 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0)));
  vec3 t2 = cross(dW, t1);

  float eR[MAXE], eS[MAXE], eF[MAXE], eC[MAXE];        // echo range, SNR, apparent reflectance, class
  for (int i = 0; i < MAXE; i++) { eR[i] = -1.0; eS[i] = 0.0; eF[i] = 0.0; eC[i] = 0.0; }
  int nE = 0;
  // 1. surfaces inside the beam footprint: one sample (single return) or five (centre + four on the footprint
  //    edge). Each carries its share of the pulse energy; samples on one surface merge into one echo
  int K = R > 1 ? 5 : 1;
  vec2 jit = vec2(lg(seed), lg(seed)) * 0.4;
  float waterR = -1.0, waterFoam = 0.0;
  for (int i = 0; i < 5; i++) {
    if (i >= K) break;
    float ai = float(i) * 1.5707963 - 0.7853982;
    vec2 o = i == 0 ? jit : vec2(cos(ai), sin(ai)) * 0.5;
    bool ins;
    vec4 S = sampleL(normalize(dW + (t1 * o.x + t2 * o.y) * uMisc.y), ins);
    if (!ins || S.a < 0.5 || S.r < uRange.x || S.r > uRange.y) continue;
    float cl = S.b < 3.0 ? (S.b > 1.5 ? 2.0 : 1.0) : floor(S.b + 0.5);
    float s = snr(S.g, S.r) / float(K);
    if (cl < 2.5) { waterR = S.r; waterFoam = S.b - 1.0; }
    // closer than the pulse can resolve (~0.6 m), or the same kind of surface stretched over neighbouring pixels
    // at grazing incidence: one echo
    int j = 0;
    for (; j < nE; j++) {
      float dR = abs(eR[j] - S.r);
      if (dR < 0.6 || (abs(eC[j] - cl) < 0.5 && dR < 0.6 + 4.0 * S.r * uMisc.z)) break;
    }
    if (j < nE) {
      eR[j] = (eR[j] * eS[j] + S.r * s) / (eS[j] + s);
      eS[j] += s; eF[j] += S.g / float(K);
      if (cl == 4.0) eC[j] = 4.0;
    } else if (nE < MAXE) {
      eR[nE] = S.r; eS[nE] = s; eF[nE] = S.g / float(K); eC[nE] = cl; nE++;
    }
  }
  // 2. retroreflector blooming: a saturating echo leaks into neighbouring shots (optical crosstalk, receiver recovery)
  if (uMisc.w > 0.5) {
    for (int i = 0; i < 8; i++) {
      float ang = (float(i) + lu(seed)) * 0.7853982;
      float rr = uMisc.z * (1.0 + 4.0 * lu(seed));
      bool ins;
      vec4 B = sampleL(normalize(dW + (t1 * cos(ang) + t2 * sin(ang)) * rr), ins);
      if (!ins || B.b < 3.5 || B.b > 4.5 || B.r > uRange.y) continue;
      bool dup = false;
      for (int j = 0; j < MAXE; j++) if (j < nE && abs(eR[j] - B.r) < 0.6) dup = true;
      if (!dup && nE < MAXE) { eR[nE] = B.r; eS[nE] = snr(B.g, B.r) * 1e-3 * exp(-rr / 0.0044); eF[nE] = 2.0; eC[nE] = 7.0; nE++; }
      break;
    }
  }
  // 3. sea spray blown off breaking crests, in front of the water echo (rough seas)
  if (waterR > 0.0 && lu(seed) < waterFoam * uMisc2.y) {
    float rs = waterR - (0.4 + 2.5 * lu(seed));
    if (rs > uRange.x && nE < MAXE) { eR[nE] = rs; eS[nE] = snr(0.15, rs) * (0.3 + lu(seed)); eF[nE] = 0.05 + 0.15 * lu(seed); eC[nE] = 6.0; nE++; }
  }
  // 4. fog droplets and raindrops near the sensor
  if (lu(seed) < uAtm.z && nE < MAXE) { eR[nE] = uRange.x + 0.5 + 3.0 * (-log(lu(seed))); eS[nE] = 50.0; eF[nE] = 0.02 + 0.06 * lu(seed); eC[nE] = 6.0; nE++; }
  if (lu(seed) < uAtm.w && nE < MAXE) { eR[nE] = uRange.x + 0.3 + 5.0 * (-log(lu(seed))); eS[nE] = 50.0; eF[nE] = 0.05 + 0.2 * lu(seed); eC[nE] = 6.0; nE++; }

  // detection, speckle and range noise, echo by echo
  int nD = 0;
  for (int i = 0; i < MAXE; i++) {
    if (i >= nE) break;
    // sea-surface glints are a few facets (exponential speckle); diffuse targets average out
    float spk = eC[i] < 1.5 ? -log(lu(seed)) : max(0.0, 1.0 + 0.12 * lg(seed));
    float s = eS[i] * spk;
    if (lu(seed) >= pod(s)) { eR[i] = -1.0; continue; }
    eR[i] += uRange.w * clamp(inversesqrt(max(s, 1e-4)), 1.0, 6.0) * lg(seed);
    if (eC[i] < 5.5) eF[i] *= spk;
    nD++;
  }
  if (nD == 0 && lu(seed) < uMisc2.x) { eR[0] = mix(uRange.x, uRange.y, lu(seed)); eC[0] = 6.0; eF[0] = 0.01; nE = max(nE, 1); }

  // this texel holds the ret-th detected echo in range order (0 = first return)
  float lastR = 0.0;
  int found = -1;
  for (int r = 0; r < 5; r++) {
    if (r > ret) break;
    float bestR = 1e9;
    int bi = -1;
    for (int i = 0; i < MAXE; i++) {
      if (i >= nE) break;
      if (eR[i] > lastR && eR[i] < bestR) { bestR = eR[i]; bi = i; }
    }
    found = bi;
    if (bi < 0) break;
    lastR = bestR;
  }
  if (found < 0 || eR[found] > uRange.y) { outP = vec4(0.0); return; }
  outP = vec4(uPos + dW * eR[found], (float(ret) * 16.0 + eC[found]) * 1000.0 + clamp(eF[found], 0.0, 999.0));
}`;
const lidarBeamMat = new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3, depthTest: false, depthWrite: false,
  vertexShader: 'void main() { gl_Position = vec4(position.xy, 0.0, 1.0); }', fragmentShader: LIDAR_BEAM_FRAG,
  uniforms: {
    tL: { value: lidarRT.texture }, uLRes: { value: new THREE.Vector2() }, uVP: { value: new THREE.Matrix4() },
    uPos: { value: new THREE.Vector3() }, uRot: { value: new THREE.Matrix3() }, uBuf: { value: new THREE.Vector2(1, 1) },
    uKind: { value: 1 }, uRet: { value: 1 }, uIdx0: { value: 0 }, uCount: { value: 0 }, uPh: { value: new THREE.Vector2() },
    uW: { value: new THREE.Vector2(LIDAR_W1, LIDAR_W2) }, uRate: { value: 1e5 }, uFov: { value: new THREE.Vector2(0.6, 0.6) },
    uEl: { value: new THREE.Vector2() }, uRange: { value: new THREE.Vector4() }, uAtm: { value: new THREE.Vector4() },
    uMisc: { value: new THREE.Vector4() }, uMisc2: { value: new THREE.Vector2() },
  },
});

// ---- point cloud display: one GL point per buffer texel, positions fetched in the vertex shader
const LIDAR_CMAP_GLSL = /* glsl */`
vec3 lcmap(float t) {
  t = clamp(t, 0.0, 1.0) * 4.0;
  const vec3 c0 = vec3(0.10, 0.12, 0.45), c1 = vec3(0.05, 0.45, 0.95), c2 = vec3(0.15, 0.85, 0.65),
             c3 = vec3(0.98, 0.85, 0.15), c4 = vec3(0.95, 0.25, 0.10);
  if (t < 1.0) return mix(c0, c1, t);
  if (t < 2.0) return mix(c1, c2, t - 1.0);
  if (t < 3.0) return mix(c2, c3, t - 2.0);
  return mix(c3, c4, t - 3.0);
}`;
const LIDAR_CMAP_CSS = 'linear-gradient(90deg,rgb(26,31,115),rgb(13,115,242),rgb(38,217,166),rgb(250,217,38),rgb(242,64,26))';
const LIDAR_SEM = [['water', '#2673f2'], ['foam', '#b3e6ff'], ['person', '#ff7319'], ['retro', '#ffffff'], ['object', '#ffd933'], ['clutter', '#8c8c99'], ['bloom', '#e64de6']];
const LIDAR_RET_COL = ['#4db3ff', '#4dff73', '#ffe633', '#ff8c26', '#ff3359'];
const lidarPtsGeo = new THREE.BufferGeometry();
lidarPtsGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(LIDAR_MAXN), 1));
// positions live in a texture: give the renderer's depth sort a fixed bound instead of computing one from the dummy attribute
lidarPtsGeo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 1e5);
lidarPtsGeo.boundingBox = new THREE.Box3(new THREE.Vector3(-1e5, -1e5, -1e5), new THREE.Vector3(1e5, 1e5, 1e5));
const lidarPtsMat = new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3,
  uniforms: {
    uPts: { value: null }, uBuf: { value: new THREE.Vector2(1, 1) }, uHead: { value: 0 }, uShow: { value: 0 }, uKind: { value: 1 },
    uRet: { value: 1 }, uSlots: { value: 1 },
    uSize: { value: 2 }, uPR: { value: 1 }, uColor: { value: 0 }, uRmax: { value: 100 }, uSensor: { value: new THREE.Vector3() }, uPersp: { value: 1 },
    uRGB: { value: rgbRT.texture }, uCamVP: { value: new THREE.Matrix4() }, uExpo: { value: 1 },
  },
  vertexShader: LIDAR_CMAP_GLSL + /* glsl */`
uniform sampler2D uPts, uRGB;
uniform vec2 uBuf;
uniform float uHead, uShow, uKind, uRet, uSlots, uSize, uPR, uColor, uRmax, uPersp, uExpo;
uniform vec3 uSensor;
uniform mat4 uCamVP;
out vec3 vCol;
void main() {
  int W = int(uBuf.x), Np = int(uSlots), id = gl_VertexID;
  vec4 p = texelFetch(uPts, ivec2(id % W, id / W), 0);
  bool ok = p.w > 0.5;
  // solid-state scanners: only pulses inside the integration window
  if (uKind > 0.5) ok = ok && float((int(uHead) - 1 - id / int(uRet + 0.5) + Np) % Np) < uShow;
  if (!ok) { gl_Position = vec4(2.0, 2.0, 2.0, 1.0); gl_PointSize = 0.0; vCol = vec3(0.0); return; }
  float q = floor(p.w / 1000.0), refl = p.w - q * 1000.0, cls = mod(q, 16.0), ret = floor(q / 16.0);
  int mode = int(uColor + 0.5);
  vec3 c;
  if (mode == 0) c = (cls == 4.0 || cls == 7.0 || refl > 1.5) ? vec3(1.0, 0.97, 0.95) : lcmap(log(1.0 + 50.0 * refl) / log(51.0));
  else if (mode == 1) c = lcmap(distance(p.xyz, uSensor) / uRmax);
  else if (mode == 2) c = lcmap((p.y + 1.5) / 4.0);
  else if (mode == 3) c = cls == 1.0 ? vec3(0.15, 0.45, 0.95) : cls == 2.0 ? vec3(0.7, 0.9, 1.0) : cls == 3.0 ? vec3(1.0, 0.45, 0.1)
         : cls == 4.0 ? vec3(1.0) : cls == 5.0 ? vec3(1.0, 0.85, 0.2) : cls == 6.0 ? vec3(0.55, 0.55, 0.6) : vec3(0.9, 0.3, 0.9);
  else if (mode == 4) c = ret < 0.5 ? vec3(0.3, 0.7, 1.0) : ret < 1.5 ? vec3(0.3, 1.0, 0.45) : ret < 2.5 ? vec3(1.0, 0.9, 0.2)
         : ret < 3.5 ? vec3(1.0, 0.55, 0.15) : vec3(1.0, 0.2, 0.35);
  else {
    // true colour: the point projected into the co-registered EO camera frame (as an L2 colours its cloud)
    vec4 cp = uCamVP * vec4(p.xyz, 1.0);
    vec2 uv = cp.xy / max(cp.w, 1e-6) * 0.5 + 0.5;
    c = vec3(0.3);
    if (cp.w > 0.0 && all(greaterThan(uv, vec2(0.0))) && all(lessThan(uv, vec2(1.0)))) {
      vec3 hdr = textureLod(uRGB, uv, 0.0).rgb * uExpo;
      c = pow(1.0 - exp(-hdr), vec3(1.0 / 2.2));
    }
  }
  vCol = c;
  vec4 mv = modelViewMatrix * vec4(p.xyz, 1.0);
  gl_Position = projectionMatrix * mv;
  gl_PointSize = uSize * uPR * (uPersp > 0.5 ? clamp(30.0 / max(-mv.z, 1e-3), 0.7, 1.4) : 1.0) + position.x * 0.0;
}`,
  fragmentShader: /* glsl */`
in vec3 vCol;
layout(location = 0) out vec4 outC;
void main() { vec2 d = gl_PointCoord - 0.5; if (dot(d, d) > 0.25) discard; outC = vec4(vCol, 1.0); }`,
});
const lidarPoints = new THREE.Points(lidarPtsGeo, lidarPtsMat);
lidarPoints.frustumCulled = false;
const lidarScene = new THREE.Scene();
lidarScene.add(lidarPoints);
const lidarViewCam = new THREE.PerspectiveCamera(50, 1.25, 0.1, 6000);
const lidarBevCam = new THREE.OrthographicCamera(-50, 50, 40, -40, 0.1, 3000);

// ---- statistics for the window text: 256 partial sums (all, person, retro, later returns) read back twice a second
const lidarStatRT = new THREE.WebGLRenderTarget(256, 1, { type: THREE.FloatType, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter, depthBuffer: false });
const lidarStatMat = new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3, depthTest: false, depthWrite: false,
  uniforms: {
    uPts: lidarPtsMat.uniforms.uPts, uBuf: lidarPtsMat.uniforms.uBuf, uHead: lidarPtsMat.uniforms.uHead, uShow: lidarPtsMat.uniforms.uShow,
    uKind: lidarPtsMat.uniforms.uKind, uRet: lidarPtsMat.uniforms.uRet, uSlots: lidarPtsMat.uniforms.uSlots,
  },
  vertexShader: 'void main() { gl_Position = vec4(position.xy, 0.0, 1.0); }',
  fragmentShader: /* glsl */`
precision highp float;
precision highp int;
uniform sampler2D uPts;
uniform vec2 uBuf;
uniform float uHead, uShow, uKind, uRet, uSlots;
layout(location = 0) out vec4 o;
void main() {
  int W = int(uBuf.x), N = W * int(uBuf.y), Np = int(uSlots), R = int(uRet + 0.5);
  int per = (N + 255) / 256, base = int(gl_FragCoord.x) * per;
  vec4 acc = vec4(0.0);
  for (int k = 0; k < 8192; k++) {
    int id = base + k;
    if (k >= per || id >= N) break;
    vec4 p = texelFetch(uPts, ivec2(id % W, id / W), 0);
    if (p.w < 0.5) continue;
    if (uKind > 0.5 && float((int(uHead) - 1 - id / R + Np) % Np) >= uShow) continue;
    float q = floor(p.w / 1000.0), cls = mod(q, 16.0);
    acc += vec4(1.0, cls == 3.0 ? 1.0 : 0.0, cls == 4.0 ? 1.0 : 0.0, q >= 16.0 ? 1.0 : 0.0);
  }
  o = acc;
}`,
});
const lidarStatBuf = new Float32Array(256 * 4);

// ---------------------------------------------------------------------------- state, model switching, helpers
const L = {
  model: null, W: 1, H: 1, N: 1, R: 1, head: 0, acc: 0, t: 0, col: 0, colFrac: 0, frame: 0, statT: 0, stats: [0, 0, 0, 0],
  pos: new THREE.Vector3(), q: new THREE.Quaternion(), pitch: 0, mat: new THREE.Matrix4(), yaw: 0, fH: new THREE.Vector3(0, 0, -1),
  center: new THREE.Vector3(), viewR: 50, gridR: 0, marker: null, grid: null,
};
let lidarPtsRT = null;
const _lq = new THREE.Quaternion(), _lv = new THREE.Vector3(), _lv2 = new THREE.Vector3(), _lE = new THREE.Euler(), _lY = new THREE.Vector3(0, 1, 0);
const _lHide = [], _lSwap = [], _lKind = new WeakMap(), _lCC = new THREE.Color();
const lidarFloatBlend = renderer.extensions.has('EXT_float_blend');

function lidarLineMat(color, opacity = 1) {
  return new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity, depthWrite: opacity >= 1 });
}
function lidarBuildMarker() {
  if (L.marker) { lidarScene.remove(L.marker); L.marker.geometry.dispose(); }
  const M = L.model, v = [];
  if (M.kind >= 1) {
    const D = 6, rx = D * Math.tan(M.fovX / 2 * D2R), ry = D * Math.tan(M.fovY / 2 * D2R);
    if (M.kind === 1) {                         // elliptical field of the rosette
      for (let i = 0; i < 48; i++) {
        const a = i / 48 * 2 * Math.PI, b = (i + 1) / 48 * 2 * Math.PI;
        v.push(rx * Math.cos(a), ry * Math.sin(a), -D, rx * Math.cos(b), ry * Math.sin(b), -D);
      }
      for (let i = 0; i < 4; i++) { const a = i * Math.PI / 2; v.push(0, 0, 0, rx * Math.cos(a), ry * Math.sin(a), -D); }
    } else {                                    // rectangular field of the line sweep
      const c = [[-rx, -ry], [rx, -ry], [rx, ry], [-rx, ry]];
      for (let i = 0; i < 4; i++) { const p = c[i], q = c[(i + 1) % 4]; v.push(p[0], p[1], -D, q[0], q[1], -D, 0, 0, 0, p[0], p[1], -D); }
    }
  } else {
    for (let i = 0; i < 32; i++) {
      const a = i / 32 * 2 * Math.PI, b = (i + 1) / 32 * 2 * Math.PI;
      v.push(Math.sin(a), 0, -Math.cos(a), Math.sin(b), 0, -Math.cos(b));
    }
    for (const el of [M.elMin, M.elMax]) {
      for (let i = 0; i < 8; i++) {
        const a = i / 8 * 2 * Math.PI, e = el * D2R;
        v.push(0, 0, 0, 2.5 * Math.sin(a) * Math.cos(e), 2.5 * Math.sin(e), -2.5 * Math.cos(a) * Math.cos(e));
      }
    }
    v.push(0, 0, 0, 0, 0, -3);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(v, 3));
  L.marker = new THREE.LineSegments(g, lidarLineMat(0x7fd4ff, 0.8));
  L.marker.matrixAutoUpdate = false;
  L.marker.frustumCulled = false;
  lidarScene.add(L.marker);
}
function lidarBuildGrid(R) {
  if (L.grid) { lidarScene.remove(L.grid); L.grid.geometry.dispose(); L.grid.material.dispose(); }
  const step = R <= 120 ? 10 : 25, rings = Math.max(1, Math.round(R / step));
  L.grid = new THREE.PolarGridHelper(rings * step, 12, rings, 96, 0x1e4552, 0x15323c);
  L.grid.material.transparent = true; L.grid.material.opacity = 0.7; L.grid.material.depthWrite = false;
  L.grid.frustumCulled = false;
  L.gridR = R;
  lidarScene.add(L.grid);
}
function lidarClearPoints() {
  const ca = renderer.getClearAlpha(); renderer.getClearColor(_lCC);
  renderer.setRenderTarget(lidarPtsRT); renderer.setClearColor(0x000000, 0); renderer.clear();
  renderer.setClearColor(_lCC, ca); renderer.setRenderTarget(null);
}
function lidarSetModel(fromGui) {
  const M = LIDAR_MODELS[params.lidarModel];
  L.model = M;
  if (fromGui) { params.lidarR10 = M.R10; params.lidarRmax = M.Rmax; params.lidarReturns = M.defRet || 1; }
  // each pulse owns R texels (one per return): consecutive texels (solid-state) or consecutive rows (spinning)
  L.R = params.lidarReturns = Math.min(params.lidarReturns, M.maxRet);
  L.N = M.kind >= 1 ? 1024 * 256 : M.cols * M.ch;                  // pulse slots
  [L.W, L.H] = M.kind >= 1 ? [1024, 256 * L.R] : [M.cols, M.ch * L.R];
  L.head = 0; L.col = 0; L.colFrac = 0; L.acc = 0;
  if (lidarPtsRT) lidarPtsRT.dispose();
  lidarPtsRT = new THREE.WebGLRenderTarget(L.W, L.H, {
    type: THREE.FloatType, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter, depthBuffer: false, generateMipmaps: false,
  });
  lidarPtsRT.scissorTest = true;
  lidarClearPoints();
  lidarBeamMat.uniforms.uBuf.value.set(L.W, L.H);
  lidarPtsMat.uniforms.uPts.value = lidarPtsRT.texture;
  lidarPtsMat.uniforms.uBuf.value.set(L.W, L.H);
  lidarPtsMat.uniforms.uKind.value = M.kind;
  lidarPtsMat.uniforms.uRet.value = lidarBeamMat.uniforms.uRet.value = L.R;
  lidarPtsMat.uniforms.uSlots.value = L.N;
  lidarPtsGeo.setDrawRange(0, L.W * L.H);
  lidarBuildMarker();
  gui.controllersRecursive().forEach(c => c.updateDisplay());
}

// hide everything without a LiDAR response (sky, clouds, rain, spray, glows, markers); give solids a LiDAR material
function lidarKind(m) {
  let k = _lKind.get(m);
  if (k === undefined) {
    const key = m.customProgramCacheKey ? m.customProgramCacheKey() : '';
    k = key === 'wet-thermal' ? (m.transparent && !lidarFloatBlend ? 0 : 1) : key === 'uw-solid' ? 2 : 0;
    _lKind.set(m, k);
  }
  return k;
}
function lidarSceneBegin() {
  scene.traverse(o => {
    if (!o.visible || o === ocean || !(o.isMesh || o.isPoints || o.isLine || o.isSprite)) return;
    if (o.isMesh && !o.isInstancedMesh) {
      if (Array.isArray(o.material) ? o.material.every(m => lidarKind(m) === 1) : lidarKind(o.material) === 1) return;
      if (!Array.isArray(o.material) && lidarKind(o.material) === 2) { _lSwap.push(o, o.material); o.material = lidarSolidMat(o.material); return; }
    }
    o.visible = false;
    _lHide.push(o);
  });
}
function lidarSceneEnd() {
  for (const o of _lHide) o.visible = true;
  for (let i = 0; i < _lSwap.length; i += 2) _lSwap[i].material = _lSwap[i + 1];
  _lHide.length = 0; _lSwap.length = 0;
}

// atmosphere at the laser wavelength
function lidarAtmos() {
  const lam = params.lidarBand;
  const sv = Math.max(oceanU.uFogExt.value.w, 1e-6);           // visible extinction (1/m): sea fog or the haze floor
  const V = 3.912 / sv / 1000;                                  // Koschmieder visibility (km)
  const q = V > 50 ? 1.6 : V > 6 ? 1.3 : V > 1 ? 0.16 * V + 0.34 : V > 0.5 ? V - 0.5 : 0;   // Kim et al. 2001
  const mmh = 25 * (params.rain || 0);                          // weather 'rain' 0..1 -> mm/h
  const a = sv * Math.pow(lam / 550, -q) + (mmh > 0 ? 0.365 * Math.pow(mmh, 0.63) / 1000 : 0);
  const fog = params.lidarClutter ? clamp(1 - Math.exp(-(sv - 0.003) * 25), 0, 0.6) : 0;
  const rain = params.lidarClutter ? clamp(mmh * 0.0015, 0, 0.3) : 0;
  const amb = 1 / (1 + THERM.uSunIrr.value / 1000 * (lam === 905 ? 0.6 : 0.3));   // solar background noise
  return { a, fog, rain, amb, V };
}

// ---------------------------------------------------------------------------- scanning
function lidarRenderView(w, h) {
  lidarRT.viewport.set(0, 0, w, h);
  lidarRT.scissor.set(0, 0, w, h);
  renderView(lidarCam, lidarRT, null);
  const u = lidarBeamMat.uniforms;
  u.uLRes.value.set(w, h);
  u.uVP.value.multiplyMatrices(lidarCam.projectionMatrix, lidarCam.matrixWorldInverse);
  u.uMisc.value.z = 2 * Math.tan(lidarCam.fov * D2R / 2) / h;
}
function lidarBeamPass(x, y, w, h) {
  lidarPtsRT.viewport.set(0, 0, L.W, L.H);
  lidarPtsRT.scissor.set(x, y, w, h);
  renderer.autoClear = false;
  runPass(lidarBeamMat, lidarPtsRT);
  renderer.autoClear = true;
  lidarBeamMat.uniforms.uMisc.value.x = ++L.frame;
}
function lidarScanSolid(dt) {
  const M = L.model, u = lidarBeamMat.uniforms, q = params.quality ?? 2;
  L.acc += M.rate * dt;
  const n = Math.min(Math.floor(L.acc), L.N >> 1);
  L.acc -= Math.floor(L.acc);
  if (n <= 0) return;
  // a frustum just larger than the field of view, pixels roughly square in angle
  const hx = (M.fovX / 2 + 2) * D2R, hy = (M.fovY / 2 + 2) * D2R, tx = Math.tan(hx), ty = Math.tan(hy), tm = Math.max(tx, ty);
  const res = ([320, 384, 512, 512][q] ?? 512) * (M.kind === 2 ? 2 : 1);
  const w = clamp(Math.round(res * tx / tm), 32, LIDAR_RT_W), h = clamp(Math.round(res * ty / tm), 32, LIDAR_RT_H);
  lidarCam.fov = 2 * hy * R2D; lidarCam.aspect = tx / ty; lidarCam.updateProjectionMatrix();
  lidarCam.position.copy(L.pos); lidarCam.quaternion.copy(L.q); lidarCam.updateMatrixWorld();
  lidarRenderView(w, h);
  u.uKind.value = M.kind; u.uRate.value = M.rate; u.uFov.value.set(M.fovX / 2 * D2R, M.fovY / 2 * D2R);
  if (M.kind === 2) u.uW.value.set(2 * Math.PI * 25, 2 * Math.PI * 3.1);    // 25 Hz sweep, slow vertical dither
  else u.uW.value.set(LIDAR_W1, LIDAR_W2);
  let i0 = L.head, left = n;
  while (left > 0) {
    const cnt = Math.min(left, L.N - i0);
    u.uIdx0.value = i0; u.uCount.value = cnt;
    u.uPh.value.set((u.uW.value.x * L.t) % (2 * Math.PI), (u.uW.value.y * L.t) % (2 * Math.PI));
    const r0 = Math.floor(i0 * L.R / L.W), r1 = Math.floor(((i0 + cnt) * L.R - 1) / L.W);
    lidarBeamPass(0, r0, L.W, r1 - r0 + 1);
    i0 = (i0 + cnt) % L.N; L.t += cnt / M.rate; left -= cnt;
  }
  L.head = i0;
}
function lidarScanSpin(dt) {
  const M = L.model, u = lidarBeamMat.uniforms, cols = M.cols, q = params.quality ?? 2;
  const adv = L.colFrac + params.lidarHz * dt * cols;
  let n = Math.min(Math.floor(adv), cols);
  L.colFrac = adv - Math.floor(adv);
  u.uKind.value = 0; u.uEl.value.set(M.elMin * D2R, M.elMax * D2R);
  const ppd = [3.5, 4.5, 6, 7][q] ?? 6, maxChunk = Math.floor(cols * 100 / 360);
  const elMax = (Math.max(Math.abs(M.elMin), Math.abs(M.elMax)) + 1.5) * D2R;
  let c = L.col;
  while (n > 0) {
    const len = Math.min(n, cols - c, maxChunk);
    const az0 = c / cols * 2 * Math.PI, az1 = (c + len) / cols * 2 * Math.PI;
    const halfW = (az1 - az0) / 2 + 2 * D2R, tanH = Math.tan(halfW), tanV = Math.tan(elMax) / Math.cos(halfW);
    const k = Math.min(1, LIDAR_RT_W / (2 * halfW * R2D * ppd), LIDAR_RT_H / (2 * Math.atan(tanV) * R2D * ppd));
    const w = Math.max(16, Math.ceil(2 * halfW * R2D * ppd * k)), h = Math.max(16, Math.ceil(2 * Math.atan(tanV) * R2D * ppd * k));
    lidarCam.fov = 2 * Math.atan(tanV) * R2D; lidarCam.aspect = tanH / tanV; lidarCam.updateProjectionMatrix();
    lidarCam.position.copy(L.pos);
    lidarCam.quaternion.copy(L.q).multiply(_lq.setFromAxisAngle(_lY, -(az0 + az1) / 2));
    lidarCam.updateMatrixWorld();
    lidarRenderView(Math.min(w, LIDAR_RT_W), Math.min(h, LIDAR_RT_H));
    lidarBeamPass(c, 0, len, L.H);
    c = (c + len) % cols; n -= len;
  }
  L.col = c;
}
function lidarPose() {
  const M = L.model;
  L.pos.copy(sensorCam.position);
  _lv.set(0, 0, -1).applyQuaternion(sensorCam.quaternion);
  if (Math.hypot(_lv.x, _lv.z) > 1e-3) L.yaw = Math.atan2(-_lv.x, -_lv.z);
  L.fH.set(-Math.sin(L.yaw), 0, -Math.cos(L.yaw));
  if (M.kind >= 1) L.q.copy(sensorCam.quaternion);       // boresighted with the EO / IR cameras
  else {
    // on the gimbal the spin axis tilts with the camera (beam fan centred on the line of sight ahead); levelled: horizon
    L.pitch = (params.lidarLevel ? 0 : Math.asin(clamp(_lv.y, -1, 1))) - params.lidarTilt * D2R;
    L.q.setFromEuler(_lE.set(L.pitch, L.yaw, 0, 'YXZ'));
  }
  L.mat.compose(L.pos, L.q, _lv2.set(1, 1, 1));
}
function lidarScan(dt) {
  const M = L.model, u = lidarBeamMat.uniforms, A = lidarAtmos();
  u.uPos.value.copy(L.pos);
  u.uRot.value.setFromMatrix4(L.mat);
  u.uRange.value.set(M.Rmin, params.lidarRmax, params.lidarR10, M.sigma * params.lidarNoise);
  u.uAtm.value.set(A.a, A.amb, A.fog, A.rain);
  u.uMisc.value.y = M.div * D2R; u.uMisc.value.w = params.lidarBloom ? 1 : 0;
  // solar false alarms; spray off breaking crests once the wind passes ~8 m/s (sea state 4-5 and up)
  u.uMisc2.value.set(1e-5 + 4e-4 * (1 - A.amb), clamp((params.windSpeed - 8) / 15, 0, 1) * 0.6);
  u.uRet.value = L.R;
  LIDAR_U.uLidarBand.value = params.lidarBand === 1550 ? 1 : 0;
  lidarCam.far = Math.min(params.lidarRmax * 1.3 + 10, 2000); lidarCam.near = 0.2;
  const ca = renderer.getClearAlpha(); renderer.getClearColor(_lCC);
  renderer.setClearColor(0x000000, 0);
  lidarSceneBegin();
  LIDAR_U.uLidar.value = 1;
  try {
    if (M.kind >= 1) lidarScanSolid(dt); else lidarScanSpin(dt);
  } finally {
    LIDAR_U.uLidar.value = 0;
    lidarSceneEnd();
    renderer.setClearColor(_lCC, ca);
    renderer.setRenderTarget(null);
  }
  L.atm = A;
}

// ---------------------------------------------------------------------------- window
const winLidar = document.createElement('div');
winLidar.className = 'sensor';
winLidar.id = 'winLidar';
winLidar.innerHTML = '<div class="t"><span id="osdLidarT"></span><span id="osdLidarV"></span></div>' +
  '<div class="b"><span id="osdLidarN"></span><span id="osdLidarLg"></span></div>';
document.body.appendChild(winLidar);
const lidarStyle = document.createElement('style');
lidarStyle.textContent = '#winLidar .b{flex-wrap:wrap;column-gap:10px}#winLidar .b span:last-child{margin-left:auto}.lgbar{display:inline-block;width:64px;height:7px;margin:0 5px;vertical-align:middle;border:1px solid rgba(220,235,240,.5)}' +
  '.lgsem i{display:inline-block;width:7px;height:7px;margin:0 2px 0 6px;vertical-align:middle;border-radius:50%}';
document.head.appendChild(lidarStyle);
function lidarRect() {
  const b = sensorRects()[1];
  return { x: b.x + b.w + 10, y: b.y, w: b.w, h: b.h };
}
let lidarShown = null;
function lidarLayout() {
  const on = !!(params.sensors && params.lidar), r = lidarRect();
  winLidar.style.display = on ? 'block' : 'none';
  winLidar.style.left = r.x + 'px'; winLidar.style.width = r.w + 'px'; winLidar.style.height = r.h + 'px'; winLidar.style.bottom = r.y + 'px';
  lidarShown = on;
}
window.addEventListener('resize', lidarLayout);
// the window takes the mouse: drag = orbit (3D view) or rotate (bird's-eye), wheel = zoom, double-click = reset
winLidar.style.pointerEvents = 'auto';
winLidar.style.cursor = 'grab';
winLidar.title = i18n('Drag to orbit · wheel to zoom · double-click to reset');
{
  let drag = null;
  winLidar.addEventListener('pointerdown', e => { drag = { x: e.clientX, y: e.clientY }; winLidar.setPointerCapture(e.pointerId); winLidar.style.cursor = 'grabbing'; e.preventDefault(); });
  winLidar.addEventListener('pointermove', e => {
    if (!drag) return;
    params.lidarYaw = ((params.lidarYaw - (e.clientX - drag.x) * 0.4 + 540) % 360) - 180;
    params.lidarPitch = clamp(params.lidarPitch + (e.clientY - drag.y) * 0.3, 3, 89);
    drag = { x: e.clientX, y: e.clientY };
  });
  const end = () => { if (drag) { drag = null; winLidar.style.cursor = 'grab'; gui.controllersRecursive().forEach(c => c.updateDisplay()); } };
  winLidar.addEventListener('pointerup', end);
  winLidar.addEventListener('pointercancel', end);
  winLidar.addEventListener('wheel', e => { params.lidarZoom = clamp(params.lidarZoom * Math.exp(e.deltaY * 0.001), 0.15, 6); e.preventDefault(); e.stopPropagation(); }, { passive: false });
  winLidar.addEventListener('dblclick', () => { Object.assign(params, { lidarYaw: 0, lidarPitch: 35, lidarZoom: 1 }); gui.controllersRecursive().forEach(c => c.updateDisplay()); });
}
const LIDAR_VIEWS = ['3D CHASE', 'SENSOR POV', 'BIRD\'S-EYE'];
function lidarLegend() {
  const c = params.lidarColor, el = document.getElementById('osdLidarLg');
  if (c === 3) { el.innerHTML = '<span class="lgsem">' + LIDAR_SEM.slice(0, 6).map(([n, col]) => `<i style="background:${col}"></i>${i18n(n, 'sem')}`).join('') + '</span>'; return; }
  if (c === 4) {
    el.innerHTML = `${i18n('RETURN')}<span class="lgsem">` + LIDAR_RET_COL.slice(0, L.R).map((col, i) => `<i style="background:${col}"></i>${i18n(['1st', '2nd', '3rd', '4th', '5th'][i], 'ret')}`).join('') + '</span>';
    return;
  }
  if (c === 5) { el.textContent = i18n('RGB from the EO camera · grey outside its view'); return; }
  const [a, b, name] = c === 0 ? ['0', '100% · retro white', 'REFL'] : c === 1 ? ['0', Math.min(params.lidarRmax, params.lidarR10 * 1.5).toFixed(0) + ' m', 'RANGE'] : ['−1.5', '+2.5 m', 'HEIGHT'];
  el.innerHTML = `${i18n(name)} ${a}<span class="lgbar" style="background:${LIDAR_CMAP_CSS}"></span>${i18n(b)}`;
}
function lidarOSD() {
  const M = L.model, [n, pp, rr, mr] = L.stats, zh = LANG === 'zh';
  document.getElementById('osdLidarT').textContent = `${i18n('LiDAR')} ${params.lidarBand} nm · ${i18n(M.short)}` +
    (M.kind === 0 ? ' · ' + params.lidarHz + ' Hz' : '') + (L.R > 1 ? ` · ${L.R} ${zh ? '回波' : 'returns'}` : '');
  document.getElementById('osdLidarV').textContent = i18n(LIDAR_VIEWS[params.lidarView]);
  const k = n >= 1000 ? (n / 1000).toFixed(1) + 'k' : n.toFixed(0);
  document.getElementById('osdLidarN').textContent = (zh ? `${k} 点 · 人员 ${pp.toFixed(0)} · 反光 ${rr.toFixed(0)}` : `${k} pts · person ${pp.toFixed(0)} · retro ${rr.toFixed(0)}`) +
    (L.R > 1 ? (zh ? ` · 后续回波 ${mr.toFixed(0)}` : ` · later returns ${mr.toFixed(0)}`) : '') +
    (L.atm && L.atm.V < 10 ? `${zh ? ' · 能见度 ' : ' · vis '}${L.atm.V < 1 ? (L.atm.V * 1000).toFixed(0) + ' m' : L.atm.V.toFixed(1) + ' km'}` : '');
}
function lidarStats() {
  runPass(lidarStatMat, lidarStatRT);
  renderer.readRenderTargetPixels(lidarStatRT, 0, 0, 256, 1, lidarStatBuf);
  const s = [0, 0, 0, 0];
  for (let i = 0; i < 256; i++) for (let j = 0; j < 4; j++) s[j] += lidarStatBuf[i * 4 + j];
  L.stats = s;
  renderer.setRenderTarget(null);
}
// viewer: chase camera behind / above the sensor (follows the main view's heading), sensor POV, or heading-up BEV
function lidarViewer(r) {
  const M = L.model, p = L.pos;
  const yawQ = _lq.setFromAxisAngle(_lY, params.lidarYaw * D2R);
  if (M.kind >= 1) {
    const d = _lv.set(0, 0, -1).applyQuaternion(L.q);
    const tHit = d.y < -0.02 ? p.y / -d.y : Infinity;
    const D = clamp(Math.min(tHit, params.lidarR10 * 0.9), 12, params.lidarRmax * 0.8);
    L.center.copy(p).addScaledVector(d, D);
    if (tHit < Infinity) L.center.y = 0;
    L.viewR = D;
  } else {
    // a level scanner sees the sea out to roughly where its lowest beams land (grazing returns are weak beyond)
    const lim = Math.min(params.lidarR10, params.lidarRmax);
    const reach = clamp(p.y / Math.tan(Math.max(-L.model.elMin * D2R - L.pitch, 3 * D2R)) * 2.5, 12, lim);
    L.center.set(p.x, 0, p.z).addScaledVector(L.fH, -L.pitch > 3 * D2R ? Math.min(p.y / Math.tan(-L.pitch), reach) : reach * 0.2);
    L.viewR = reach;
  }
  const u = lidarPtsMat.uniforms;
  if (params.lidarView === 2) {
    const c = _lv2.set(p.x, 0, p.z).lerp(L.center.clone().setY(0), 0.5), e = Math.max(L.viewR * 0.75, 15) * params.lidarZoom, a = r.w / r.h;
    Object.assign(lidarBevCam, { left: -e * a, right: e * a, top: e, bottom: -e });
    lidarBevCam.position.set(c.x, p.y + 500, c.z);
    lidarBevCam.up.copy(L.fH).applyQuaternion(yawQ);
    lidarBevCam.lookAt(c.x, 0, c.z);
    lidarBevCam.far = p.y + 1000;
    lidarBevCam.updateProjectionMatrix();
    u.uPersp.value = 0;
    return lidarBevCam;
  }
  const cam = lidarViewCam;
  cam.aspect = r.w / r.h;
  if (params.lidarView === 1) {
    cam.position.copy(sensorCam.position); cam.quaternion.copy(sensorCam.quaternion); cam.fov = sensorCam.fov;
  } else {
    // orbit around the scan centre, starting from behind the sensor (drag in the window to orbit, wheel to zoom)
    const dist = clamp(L.viewR * (M.kind >= 1 ? 0.75 : 1.1), 12, 200) * params.lidarZoom, el = params.lidarPitch * D2R;
    const back = _lv2.copy(L.fH).negate().applyQuaternion(yawQ);
    cam.position.copy(L.center).addScaledVector(back, dist * Math.cos(el)).add(_lv.set(0, dist * Math.sin(el), 0));
    cam.up.set(0, 1, 0);
    cam.lookAt(L.center);
    cam.fov = 50;
  }
  cam.updateProjectionMatrix();
  u.uPersp.value = 1;
  return cam;
}
function lidarDraw() {
  const r = lidarRect(), u = lidarPtsMat.uniforms;
  u.uHead.value = L.head;
  u.uShow.value = L.model.kind >= 1 ? Math.min(L.N, Math.round(L.model.rate * params.lidarInteg)) : L.N;
  u.uCamVP.value.multiplyMatrices(sensorCam.projectionMatrix, sensorCam.matrixWorldInverse);
  u.uExpo.value = compositePass.uniforms.uExposure.value;
  u.uSize.value = params.lidarPtSize; u.uPR.value = renderer.getPixelRatio(); u.uColor.value = params.lidarColor;
  u.uRmax.value = Math.min(params.lidarRmax, params.lidarR10 * 1.5); u.uSensor.value.copy(L.pos);
  const cam = lidarViewer(r);
  if (L.gridR !== params.lidarRmax) lidarBuildGrid(params.lidarRmax);
  L.grid.position.set(L.pos.x, 0, L.pos.z);
  L.marker.matrix.copy(L.mat); L.marker.matrixWorldNeedsUpdate = true;
  L.marker.visible = params.lidarView !== 1;
  const ca = renderer.getClearAlpha(); renderer.getClearColor(_lCC);
  renderer.setRenderTarget(null);
  renderer.setScissorTest(true);
  renderer.setViewport(r.x, r.y, r.w, r.h);
  renderer.setScissor(r.x, r.y, r.w, r.h);
  renderer.setClearColor(0x05090d, 1);
  renderer.clear(true, true, false);
  renderer.autoClear = false;
  renderer.render(lidarScene, cam);
  renderer.autoClear = true;
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, window.innerWidth, window.innerHeight);
  renderer.setClearColor(_lCC, ca);
}

// ---------------------------------------------------------------------------- export (PCD / LAS / LAZ; sensor_lidar_las.js)
// the integration window in the sensor frame as [x, y, z, reflectance, label, return] (scripting / tests)
function lidarReadPoints() {
  const f = lidarGatherWindow(false), out = [];
  for (let i = 0; i < f.n; i++) out.push([f.x[i], f.y[i], f.z[i], f.refl[i], f.label[i], f.ret[i]]);
  return out;
}
async function lidarExport() {
  const fmt = params.lidarExportFmt, world = params.lidarFrame === 1;
  const f = lidarGatherWindow(world);
  const desc = `Ocean MOB simulator LiDAR, ${L.model.name}, ${params.lidarBand} nm, ` +
    (world ? 'world frame x east y north z up (m)' : 'sensor frame x forward y left z up (m)') +
    ', intensity = apparent reflectance, label 1 water 2 foam 3 person 4 retro 5 object 6 clutter 7 blooming';
  const bytes = await lidarEncode(f, fmt, desc, 0, true);
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([bytes], { type: fmt === 'pcd' ? 'text/plain' : 'application/octet-stream' }));
  a.download = `lidar_${L.model.short.replace(/[^A-Za-z0-9]+/g, '_').replace(/_+$/, '').toLowerCase()}_${new Date().toISOString().replace(/[:.]/g, '-')}.${fmt}`;
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 2000);
}

// ---------------------------------------------------------------------------- GUI and frame hook
const fLidar = gui.addFolder('LiDAR');
fLidar.add(params, 'lidar').name('Show LiDAR window').onChange(lidarLayout);
fLidar.add(params, 'lidarModel', Object.fromEntries(LIDAR_MODELS.map((m, i) => [m.name, i]))).name('Sensor').onChange(() => { lidarSetModel(true); lidarLegend(); lidarOSD(); });
fLidar.add(params, 'lidarBand', { '905 nm (silicon APD)': 905, '1550 nm (InGaAs)': 1550 }).name('Wavelength');
fLidar.add(params, 'lidarReturns', { '1 (single)': 1, '2 (dual)': 2, '3 (triple)': 3, '5 (penta)': 5 }).name('Returns per pulse')
  .onChange(() => { lidarSetModel(false); lidarLegend(); lidarOSD(); });
fLidar.add(params, 'lidarView', { '3D chase': 0, 'Sensor POV (as cameras)': 1, 'Bird\'s-eye': 2 }).name('View');
fLidar.add(params, 'lidarColor', { 'Reflectivity': 0, 'Range': 1, 'Height': 2, 'Return type (ground truth)': 3, 'Return number': 4, 'True colour (RGB camera)': 5 })
  .name('Colour').onChange(lidarLegend);
fLidar.add(params, 'lidarYaw', -180, 180, 1).name('View yaw (°)');
fLidar.add(params, 'lidarPitch', 3, 89, 1).name('View elevation (°)');
fLidar.add(params, 'lidarZoom', 0.15, 6, 0.05).name('View distance ×');
fLidar.add(params, 'lidarPtSize', 1, 5, 0.5).name('Point size (px)');
fLidar.add(params, 'lidarInteg', 0.1, 1.0, 0.05).name('Integration (s, solid-state)');
fLidar.add(params, 'lidarHz', { '5 Hz': 5, '10 Hz': 10, '20 Hz': 20 }).name('Spin rate (360° models)');
fLidar.add(params, 'lidarLevel').name('360°: level spin axis');
fLidar.add(params, 'lidarTilt', -30, 60, 1).name('360°: extra down-tilt (°)');
fLidar.add(params, 'lidarR10', 10, 400, 1).name('Range @10% refl. (m)').onChange(lidarLegend);
fLidar.add(params, 'lidarRmax', 20, 600, 1).name('Max range (m)').onChange(lidarLegend);
fLidar.add(params, 'lidarNoise', 0, 3, 0.1).name('Range noise ×');
fLidar.add(params, 'lidarClutter').name('Fog / rain clutter');
fLidar.add(params, 'lidarBloom').name('Retroreflector blooming');
fLidar.add(params, 'lidarFrame', { 'Sensor (x fwd, y left, z up)': 0, 'World (x east, y north, z up)': 1 }).name('LiDAR coordinates').listen();
fLidar.add(params, 'lidarExportFmt', { 'PCD (text)': 'pcd', 'LAS 1.2': 'las', 'LAZ (LASzip)': 'laz' }).name('Export format');
fLidar.add({ f: lidarExport }, 'f').name('Export point cloud');

lidarSetModel(false);
lidarLegend();
LANG_HOOKS.push(() => { lidarLegend(); lidarOSD(); winLidar.title = i18n('Drag to orbit · wheel to zoom · double-click to reset'); });
lidarLayout();
HOOKS.afterRender.push(() => {
  const on = !!(params.sensors && params.lidar);
  if (on !== lidarShown) lidarLayout();
  if (!on) return;
  lidarPose();
  lidarScan(Math.min(lastDtReal, 0.1));
  L.statT -= lastDtReal;
  if (L.statT <= 0) { lidarStats(); lidarOSD(); L.statT = 0.5; }
  lidarDraw();
});
HOOKS.ready.push(() => {
  window.sim.lidar = { L, models: LIDAR_MODELS, rangeRT: lidarRT, get pointsRT() { return lidarPtsRT; }, beamMat: lidarBeamMat, readPoints: lidarReadPoints, gather: lidarGatherWindow, encode: lidarEncode, export: lidarExport, exportPCD: lidarExport, setModel: i => { params.lidarModel = i; lidarSetModel(true); lidarLegend(); } };
});
