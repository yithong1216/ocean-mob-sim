// ============================================================================
// Drone EO/IR gimbal: co-registered RGB and LWIR views, shown in two windows at the bottom
// ============================================================================
const SENSOR_W = 640, SENSOR_H = 512;                       // 5:4, like a 640x512 microbolometer
const sensorCam = new THREE.PerspectiveCamera(params.sensorFov, SENSOR_W / SENSOR_H, 0.5, 60000);
const rgbRT = new THREE.WebGLRenderTarget(SENSOR_W, SENSOR_H, { type: THREE.HalfFloatType, samples: 4 });
const sensorRefrRT = new THREE.WebGLRenderTarget(SENSOR_W, SENSOR_H, {
  type: THREE.HalfFloatType, depthTexture: new THREE.DepthTexture(SENSOR_W, SENSOR_H),
});
const irRT = new THREE.WebGLRenderTarget(SENSOR_W, SENSOR_H, { type: THREE.HalfFloatType, samples: 4 });
const PIP_VERT = 'varying vec2 vUv; void main() { vUv = uv; gl_Position = vec4(position.xy, 0.0, 1.0); }';
const rgbOutMat = new THREE.ShaderMaterial({
  uniforms: { tDiffuse: { value: rgbRT.texture }, uExposure: { value: 1 }, uFrame: { value: 0 }, uAspect: { value: SENSOR_W / SENSOR_H } },
  vertexShader: PIP_VERT, fragmentShader: compositePass.material.fragmentShader, depthTest: false, depthWrite: false,
});
// ---- LWIR microbolometer camera model (written for this file by a helper agent; inlined module) ----
// thermal_sensor.js — uncooled LWIR microbolometer camera model (Boson/Tau-class 640x512 @ 30 Hz)
// for a picture-in-picture "IR camera" window. three.js r169, WebGL2, GLSL ES 3.00.
//
// Pipeline (per frame):
//   1. Render the scene's in-band LWIR radiance (R channel) into a full-res render target `rad`.
//   2. Downsample pass (THERMAL_DOWNSAMPLE_FRAG) -> small FloatType target `small` (e.g. 80x64):
//        R = apparent temperature (K, box-averaged radiance), G = mean input value, B = max temp (K).
//   3. renderer.readRenderTargetPixels(small, 0, 0, 80, 64, buf) -> agc.update(buf, dt) -> agc.applyTo(u)
//      (readback stalls ~0.1-0.5 ms; doing it every 2nd-4th frame is fine given the ~0.7 s AGC time constant).
//   4. Sensor pass (THERMAL_SENSOR_FRAG) into the PIP sub-rectangle via setViewport/setScissor.
//
// Material setup (both passes):
//   new THREE.ShaderMaterial({ glslVersion: THREE.GLSL3, vertexShader: THERMAL_FULLSCREEN_VERT,
//     fragmentShader: BAND_TEMP_GLSL + THERMAL_SENSOR_FRAG, uniforms, depthTest: false, depthWrite: false })
//   on a Mesh(new THREE.PlaneGeometry(2, 2)) with frustumCulled = false.
//   If the R channel already holds kelvin, set material.defines = { INPUT_IS_KELVIN: '' } on BOTH passes
//   (then bandTemp is not referenced and need not be concatenated).
//
// Precision notes: a HalfFloat radiance target is fine (~0.04 K quantisation at 300 K, dithered by NETD),
// but kelvin stored in HalfFloat quantises to 0.25 K — use FloatType when INPUT_IS_KELVIN.
// The small target must be FloatType for Float32Array readback; the shader reads it with texelFetch,
// so it works with or without OES_texture_float_linear.


// ---------------------------------------------------------------------------------------------
// Shared GLSL: input conversion + sensor-FOV crop (sensor aspect is centre-cropped out of the
// radiance target; uZoom > 1 is digital zoom).
// ---------------------------------------------------------------------------------------------
const COMMON_GLSL = /* glsl */ `
uniform sampler2D uRadiance;   // R: in-band radiance (W m^-2 sr^-1) or kelvin with INPUT_IS_KELVIN
uniform vec2  uSensorRes;      // native focal-plane array size, e.g. 640x512
uniform float uZoom;           // digital zoom (1 = full sensor FOV)

#ifdef INPUT_IS_KELVIN
#define TS_TO_K(x) (x)
#else
#define TS_TO_K(x) bandTemp(x)
#endif

// scale from sensor uv to radiance-target uv (centre crop to the sensor aspect)
vec2 ts_srcScale() {
  vec2 src = vec2(textureSize(uRadiance, 0));
  float aSrc = src.x / src.y;
  float aSen = uSensorRes.x / uSensorRes.y;
  vec2 k = (aSrc > aSen) ? vec2(aSen / aSrc, 1.0) : vec2(1.0, aSrc / aSen);
  return k / max(uZoom, 1e-3);
}
vec2 ts_srcUV(vec2 s, vec2 k) { return 0.5 + (s - 0.5) * k; }
`;

/** Pass-through vertex shader for a PlaneGeometry(2,2) quad (ignores the camera). */
const THERMAL_FULLSCREEN_VERT = /* glsl */ `
void main() { gl_Position = vec4(position.xy, 0.0, 1.0); }
`;

// ---------------------------------------------------------------------------------------------
// Downsample for AGC statistics: each output texel box-filters its cell of the sensor FOV with
// 8x8 bilinear taps, averages *radiance* (physically correct "apparent temperature"), then converts.
// ---------------------------------------------------------------------------------------------
const THERMAL_DOWNSAMPLE_FRAG = COMMON_GLSL + /* glsl */ `
layout(location = 0) out vec4 fragColor;
uniform vec2 uOutRes;          // size of this render target, e.g. 80x64

void main() {
  vec2 k = ts_srcScale();
  vec2 cell = 1.0 / uOutRes;
  vec2 cell0 = floor(gl_FragCoord.xy) * cell;           // sensor-uv corner of this cell
  float sumL = 0.0;
  float maxL = -1e30;
  for (int j = 0; j < 8; j++) {
    for (int i = 0; i < 8; i++) {
      vec2 s = cell0 + (vec2(float(i), float(j)) + 0.5) * 0.125 * cell;
      float L = texture(uRadiance, ts_srcUV(s, k)).r;
      sumL += L;
      maxL = max(maxL, L);
    }
  }
  float Lm = sumL * (1.0 / 64.0);
  fragColor = vec4(TS_TO_K(Lm), Lm, TS_TO_K(maxL), 1.0);   // R: apparent K, G: mean input, B: max K
}
`;

// ---------------------------------------------------------------------------------------------
// Sensor model
// ---------------------------------------------------------------------------------------------
const THERMAL_SENSOR_FRAG = COMMON_GLSL + /* glsl */ `
layout(location = 0) out vec4 fragColor;

uniform vec4  uViewport;   // PIP rect in drawing-buffer px (x, y from bottom-left, w, h) = CSS px * pixelRatio
uniform float uFill;       // 0 = letterbox the whole sensor image, 1 = fill window (crop)
uniform sampler2D uSmall;  // downsample target (G = mean input), used for the wide scatter halo; optional
uniform float uFrame;      // frame counter (temporal noise seed)
uniform float uLow;        // AGC window low  (K)
uniform float uHigh;       // AGC window high (K)
uniform float uCurve[17];  // AGC transfer curve knots (plateau equalisation), identity by default
uniform float uCurveMix;   // 0 = linear AGC, 1 = full curve
uniform float uNETD;       // temporal noise, K rms at 300 K scene
uniform float uFPN;        // fixed-pattern residual strength (1 = typical after NUC)
uniform float uFFC;        // 0 right after flat-field correction ... 1 = fully drifted
uniform float uTref;       // shutter temperature at last FFC (gain residual pivots here), K
uniform float uVignette;   // radial non-uniformity, K at the corners (positive = cooler edges)
uniform float uBlur;       // optics/pixel MTF tap spacing in sensor px (0 = none)
uniform float uGlow;       // fraction of energy scattered into a wide halo
uniform float uDDE;        // local-contrast (digital detail enhancement) gain
uniform int   uPalette;    // 0 white-hot, 1 black-hot, 2 ironbow, 3 rainbow-HC
uniform float uOSD;        // 1 = draw crosshair + palette bar

// ---- stable hashing / Gaussian noise ----
uint ts_hash(uint x) {
  x ^= x >> 16u; x *= 0x7feb352du;
  x ^= x >> 15u; x *= 0x846ca68bu;
  x ^= x >> 16u; return x;
}
float ts_u01(uint h) { return (float(h >> 8u) + 0.5) * (1.0 / 16777216.0); }   // (0,1)
float ts_gauss(uint h) {                                                         // N(0,1), Box-Muller
  float a = ts_u01(ts_hash(h));
  float b = ts_u01(ts_hash(h ^ 0x9e3779b9u));
  return sqrt(-2.0 * log(a)) * cos(6.2831853 * b);
}
uint ts_seed(ivec2 p, uint salt) { return ts_hash(uint(p.x) ^ ts_hash(uint(p.y) ^ ts_hash(salt))); }

// NETD grows for colder scenes: noise is constant in radiance, so dT = dL / (dL/dT).
// Planck dB/dT at 10 um ~ x^2 e^x / (e^x - 1)^2, x = c2/(lambda T); normalised to 300 K.
float ts_netdScale(float T) {
  float x = 1438.777 / clamp(T, 150.0, 600.0);
  float e = exp(x);
  float d = x * x * e / ((e - 1.0) * (e - 1.0));
  return clamp(0.193245 / d, 0.25, 4.0);
}

// manual bilinear fetch of the small target's G channel (works without float-linear filtering)
float ts_smallG(vec2 uv) {
  ivec2 sz = textureSize(uSmall, 0);
  ivec2 mx = sz - 1;
  vec2 p = uv * vec2(sz) - 0.5;
  vec2 fl = floor(p);
  vec2 f = p - fl;
  ivec2 i0 = ivec2(fl);
  float t00 = texelFetch(uSmall, clamp(i0,               ivec2(0), mx), 0).g;
  float t10 = texelFetch(uSmall, clamp(i0 + ivec2(1, 0), ivec2(0), mx), 0).g;
  float t01 = texelFetch(uSmall, clamp(i0 + ivec2(0, 1), ivec2(0), mx), 0).g;
  float t11 = texelFetch(uSmall, clamp(i0 + ivec2(1, 1), ivec2(0), mx), 0).g;
  return mix(mix(t00, t10, f.x), mix(t01, t11, f.x), f.y);
}

float ts_curve(float x) {                      // piecewise-linear AGC transfer curve
  float f = clamp(x, 0.0, 1.0) * 16.0;
  int i = min(int(f), 15);
  return mix(uCurve[i], uCurve[i + 1], f - float(i));
}

vec3 ts_hsv(float h, float s, float v) {
  vec3 k = clamp(abs(fract(h + vec3(0.0, 2.0 / 3.0, 1.0 / 3.0)) * 6.0 - 3.0) - 1.0, 0.0, 1.0);
  return v * mix(vec3(1.0), k, s);
}

vec3 ts_palette(float t) {
  t = clamp(t, 0.0, 1.0);
  if (uPalette == 1) return vec3(1.0 - t);                                  // black-hot
  if (uPalette == 2) {                                                      // ironbow: black-indigo-magenta-orange-yellow-white
    float r = smoothstep(0.05, 0.55, t);
    float g = pow(clamp((t - 0.35) / 0.6, 0.0, 1.0), 1.2);
    float b = 0.55 * sin(3.14159265 * clamp(t * 2.0, 0.0, 1.0)) + smoothstep(0.82, 1.0, t);
    return clamp(vec3(r, g, b), 0.0, 1.0);
  }
  if (uPalette == 3) {                                                      // rainbow-HC: violet->red, white top
    return ts_hsv(0.75 * (1.0 - min(t / 0.92, 1.0)), 1.0 - smoothstep(0.92, 1.0, t),
                  0.3 + 0.7 * smoothstep(0.0, 0.25, t));
  }
  return vec3(t);                                                           // white-hot
}

void main() {
  // ---- window -> sensor uv (letterbox or crop by aspect) ----
  vec2 win = (gl_FragCoord.xy - uViewport.xy) / uViewport.zw;
  float aWin = uViewport.z / uViewport.w;
  float aSen = uSensorRes.x / uSensorRes.y;
  vec2 fFit = (aWin > aSen) ? vec2(aWin / aSen, 1.0) : vec2(1.0, aSen / aWin);
  vec2 f = (uFill > 0.5) ? fFit / max(fFit.x, fFit.y) : fFit;
  vec2 s = 0.5 + (win - 0.5) * f;
  if (any(lessThan(s, vec2(0.0))) || any(greaterThan(s, vec2(1.0)))) {
    fragColor = vec4(0.015, 0.015, 0.015, 1.0);                             // letterbox bars
    return;
  }
  vec2 rectPx = uViewport.zw / f;                                           // on-screen size of full image

  // ---- pixelation: everything is evaluated at the detector pixel centre ----
  ivec2 px = clamp(ivec2(floor(s * uSensorRes)), ivec2(0), ivec2(uSensorRes) - 1);
  vec2 c = (vec2(px) + 0.5) / uSensorRes;
  vec2 k = ts_srcScale();
  vec2 dS = k / uSensorRes;                                                 // one sensor px in source uv
  vec2 base = ts_srcUV(c, k);

  // ---- optics MTF + pixel aperture: 3x3 binomial in radiance (linear in flux) ----
  float Lc = 0.0;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float w = (i == 0 ? 2.0 : 1.0) * (j == 0 ? 2.0 : 1.0);
      Lc += w * texture(uRadiance, base + vec2(float(i), float(j)) * uBlur * dS).r;
    }
  }
  Lc *= 1.0 / 16.0;

  // near ring (r = 2.5 px): local background for DDE and near-field scatter
  float Lr = 0.0;
  for (int i = 0; i < 8; i++) {
    float a = float(i) * 0.78539816 + 0.39269908;
    Lr += texture(uRadiance, base + vec2(cos(a), sin(a)) * 2.5 * dS).r;
  }
  Lr *= 0.125;

  // ---- stray-light halo: energy-conserving mix toward a wide blur; hot objects glow because the
  //      mix is in radiance (steep in T). Wide part comes from the 80x64 target (~8-16 px kernel). ----
  float Lo = Lc;
  if (uGlow > 0.0 && textureSize(uSmall, 0).x > 1) {
    Lo = Lc + uGlow * (0.35 * Lr + 0.65 * ts_smallG(c) - Lc);
  }
  float T  = TS_TO_K(Lo);                                                   // scene apparent temperature
  float Tb = TS_TO_K(Lo + 0.75 * (Lr - Lc));                                // local background (DDE)

  // ---- temporal noise (white per pixel + weak per-frame row noise), in K ----
  uint fr = uint(uFrame);
  float nT = uNETD * ts_netdScale(T) *
             (ts_gauss(ts_seed(px, fr * 2u + 1u)) + 0.2 * ts_gauss(ts_seed(ivec2(0, px.y), fr * 2u + 2u)));

  // ---- fixed pattern after NUC: column offsets, pixel offsets, pixel gain residual (pivots at the
  //      shutter temperature), plus offset drift accumulating since the last FFC ----
  float gCol  = ts_gauss(ts_seed(ivec2(px.x, 0), 101u));
  float gPix  = ts_gauss(ts_seed(px, 202u));
  float gGain = ts_gauss(ts_seed(px, 303u));
  float gDCol = ts_gauss(ts_seed(ivec2(px.x, 0), 404u));
  float gDPix = ts_gauss(ts_seed(px, 505u));
  float fpn = uFPN * (0.025 * gCol + 0.012 * gPix + 0.003 * gGain * (T - uTref)
                      + uFFC * (0.08 * gDCol + 0.04 * gDPix));

  // ---- low-frequency non-uniformity: vignetting / housing narcissus + drift gradient (K) ----
  vec2 q = (c - 0.5) * vec2(aSen, 1.0);
  float r2 = dot(q, q) / (0.25 * (aSen * aSen + 1.0));                      // 0 centre, 1 corner
  float lowF = -uVignette * (0.6 + 0.4 * uFFC) * r2
               + uFPN * uFFC * 0.15 * ((c.x - 0.5) + 0.5 * (c.y - 0.5));

  // ---- 14-bit core output (K), then DDE in the temperature domain (boosts noise/FPN like real DDE) ----
  float Ts = T + lowF + fpn + nT;
  float Tbg = Tb + lowF;
  float span = max(uHigh - uLow, 1e-3);
  float clipK = 0.2 * span;                                                 // limit detail -> no big halos
  float Td = Ts + uDDE * clamp(Ts - Tbg, -clipK, clipK);

  // ---- AGC + palette ----
  float x = (Td - uLow) / span;
  x = mix(x, ts_curve(x), uCurveMix);
  vec3 col = ts_palette(x);

  // ---- burned-in OSD: crosshair + palette bar (display px, scaled with the image) ----
  if (uOSD > 0.5) {
    vec2 vmin = max(vec2(0.0), 0.5 * (uViewport.zw - rectPx));             // visible image rect (window px)
    vec2 vmax = min(uViewport.zw, 0.5 * (uViewport.zw + rectPx));
    float sc = (vmax.y - vmin.y) / 512.0;
    vec2 wp = win * uViewport.zw;
    vec2 aq = abs(wp - 0.5 * uViewport.zw);                                 // from image centre
    float lw = max(0.6, 0.75 * sc);
    float gap = 6.0 * sc;
    float len = 22.0 * sc;
    bool hO = aq.y < lw + 1.0 && aq.x > gap - 1.0 && aq.x < len + 1.0;
    bool vO = aq.x < lw + 1.0 && aq.y > gap - 1.0 && aq.y < len + 1.0;
    bool hI = aq.y < lw && aq.x > gap && aq.x < len;
    bool vI = aq.x < lw && aq.y > gap && aq.y < len;
    if (hO || vO) col = mix(col, vec3(0.0), 0.6);                           // dark outline
    if (hI || vI) col = vec3(0.95);
    float bx1 = vmax.x - 10.0 * sc;
    float bx0 = bx1 - 8.0 * sc;
    float by0 = mix(vmin.y, vmax.y, 0.3);
    float by1 = mix(vmin.y, vmax.y, 0.7);
    if (wp.x > bx0 - 1.0 && wp.x < bx1 + 1.0 && wp.y > by0 - 1.0 && wp.y < by1 + 1.0) {
      bool inner = wp.x > bx0 && wp.x < bx1 && wp.y > by0 && wp.y < by1;
      col = inner ? ts_palette((wp.y - by0) / (by1 - by0)) : vec3(0.85);
    }
  }
  fragColor = vec4(col, 1.0);                                               // display-referred, no tone mapping
}
`;

function identityCurve() {
  const c = new Float32Array(17);
  for (let i = 0; i < 17; i++) c[i] = i / 16;
  return c;
}

/** Uniforms for THERMAL_SENSOR_FRAG (defaults ~ FLIR Boson 640, 50 mK NETD class). */
function createThermalSensorUniforms() {
  return {
    uRadiance: { value: null },                        // full-res radiance render target .texture
    uSmall:    { value: null },                        // downsample target .texture (enables halo)
    uViewport: { value: new THREE.Vector4(0, 0, 640, 512) },
    uFill:     { value: 0 },
    uSensorRes:{ value: new THREE.Vector2(640, 512) },
    uZoom:     { value: 1 },
    uFrame:    { value: 0 },
    uLow:      { value: 285 },
    uHigh:     { value: 300 },
    uCurve:    { value: identityCurve() },
    uCurveMix: { value: 0 },
    uNETD:     { value: 0.045 },
    uFPN:      { value: 1 },
    uFFC:      { value: 0 },
    uTref:     { value: 295 },
    uVignette: { value: 0.4 },
    uBlur:     { value: 0.5 },
    uGlow:     { value: 0.04 },
    uDDE:      { value: 0.5 },
    uPalette:  { value: 0 },
    uOSD:      { value: 1 },
  };
}

/** Uniforms for THERMAL_DOWNSAMPLE_FRAG; shares uRadiance/uSensorRes/uZoom objects with the sensor pass
 *  so both passes always see the same FOV. */
function createThermalDownsampleUniforms(sensorUniforms, outW = 80, outH = 64) {
  const u = sensorUniforms || createThermalSensorUniforms();
  return {
    uRadiance: u.uRadiance,
    uSensorRes: u.uSensorRes,
    uZoom: u.uZoom,
    uOutRes: { value: new THREE.Vector2(outW, outH) },
  };
}

// ---------------------------------------------------------------------------------------------
// CPU automatic gain control
// ---------------------------------------------------------------------------------------------
/**
 * Histogram AGC on the small apparent-temperature readback.
 *  - robust tail rejection (default 1 % / 99 %) from a 1024-bin histogram, interpolated within bins
 *  - small margin, minimum span (default 4 K), exponential smoothing of window centre and span
 *    (time constant tau, default 0.7 s; first frame snaps)
 *  - plateau equalisation curve (64 bins, bin counts clipped at `plateau` x mean) as 17 knots,
 *    blended in the shader by `plateauMix` (0 = pure linear AGC)
 */
class ThermalAGC {
  constructor(o = {}) {
    this.width = o.width ?? 80;
    this.height = o.height ?? 64;
    this.stride = o.stride ?? 4;            // RGBA readback
    this.channel = o.channel ?? 0;          // R = apparent K
    this.toKelvin = o.toKelvin ?? null;     // optional fn if the channel holds radiance
    this.tailLow = o.tailLow ?? 0.01;
    this.tailHigh = o.tailHigh ?? 0.99;
    this.margin = o.margin ?? 0.05;         // fraction of the percentile span added on each side
    this.minSpan = o.minSpan ?? 4;          // K
    this.tau = o.tau ?? 0.7;                // s
    this.bins = o.bins ?? 1024;
    this.plateau = o.plateau ?? 2.0;
    this.plateauMix = o.plateauMix ?? 0.0;
    this.roi = o.roi ?? null;               // {x0,y0,x1,y1} normalised (y up), stats region
    this.validMin = o.validMin ?? 100;
    this.validMax = o.validMax ?? 1000;
    this._vals = new Float32Array(this.width * this.height);
    this._hist = new Uint32Array(this.bins);
    this._eq = new Float64Array(64);
    this._target = identityCurve();
    this.reset();
  }

  reset() {
    this.center = NaN;
    this.span = NaN;
    this.curve = identityCurve();
    this.p1 = NaN;
    this.p99 = NaN;
  }

  get low() { return this.center - 0.5 * this.span; }
  get high() { return this.center + 0.5 * this.span; }

  _collect(data) {
    const { width: W, height: H, stride, channel, toKelvin } = this;
    let x0 = 0, x1 = W, y0 = 0, y1 = H;
    if (this.roi) {
      x0 = Math.max(0, Math.floor(this.roi.x0 * W)); x1 = Math.min(W, Math.ceil(this.roi.x1 * W));
      y0 = Math.max(0, Math.floor(this.roi.y0 * H)); y1 = Math.min(H, Math.ceil(this.roi.y1 * H));
    }
    let n = 0, mn = Infinity, mx = -Infinity;
    for (let y = y0; y < y1; y++) {
      for (let x = x0; x < x1; x++) {
        let v = data[(y * W + x) * stride + channel];
        if (toKelvin) v = toKelvin(v);
        if (!(v >= this.validMin && v <= this.validMax)) continue;   // rejects NaN too
        this._vals[n++] = v;
        if (v < mn) mn = v;
        if (v > mx) mx = v;
      }
    }
    return { n, mn, mx };
  }

  /** @param {Float32Array} data readback buffer  @param {number} dt seconds since last update */
  update(data, dt = 1 / 30) {
    const { n, mn, mx } = this._collect(data);
    if (n === 0) return this.result();

    // --- robust percentiles from a histogram over [mn, mx] ---
    const B = this.bins, hist = this._hist;
    hist.fill(0);
    const range = Math.max(mx - mn, 1e-6), bw = range / B;
    for (let i = 0; i < n; i++) hist[Math.min(B - 1, ((this._vals[i] - mn) / bw) | 0)]++;
    const pct = (p) => {
      const target = p * n;
      let cum = 0;
      for (let i = 0; i < B; i++) {
        const h = hist[i];
        if (h > 0 && cum + h >= target) return mn + (i + (target - cum) / h) * bw;
        cum += h;
      }
      return mx;
    };
    this.p1 = pct(this.tailLow);
    this.p99 = pct(this.tailHigh);

    // --- window: margin, minimum span ---
    const s0 = this.p99 - this.p1;
    let lo = this.p1 - this.margin * s0, hi = this.p99 + this.margin * s0;
    const cen = 0.5 * (lo + hi);
    const spn = Math.max(hi - lo, this.minSpan);

    // --- temporal smoothing (first frame snaps) ---
    const a = Number.isFinite(this.center) ? 1 - Math.exp(-Math.max(dt, 0) / this.tau) : 1;
    this.center = Number.isFinite(this.center) ? this.center + a * (cen - this.center) : cen;
    this.span = Number.isFinite(this.span) ? this.span + a * (spn - this.span) : spn;

    // --- plateau equalisation curve on the smoothed window ---
    const eq = this._eq, NB = eq.length, wlo = this.low, wsp = this.span;
    eq.fill(0);
    let nIn = 0;
    for (let i = 0; i < n; i++) {
      const t = (this._vals[i] - wlo) / wsp;
      if (t < 0 || t >= 1) continue;
      eq[(t * NB) | 0]++;
      nIn++;
    }
    const tgt = this._target;
    if (nIn > 0) {
      const clip = Math.max(1, this.plateau * nIn / NB);
      let tot = 0;
      for (let i = 0; i < NB; i++) { eq[i] = Math.min(eq[i], clip); tot += eq[i]; }
      let cum = 0;
      tgt[0] = 0;
      for (let i = 0; i < NB; i++) {
        cum += eq[i];
        if ((i + 1) % (NB / 16) === 0) tgt[(i + 1) / (NB / 16)] = cum / tot;
      }
    } else {
      tgt.set(identityCurve());
    }
    for (let j = 0; j < 17; j++) this.curve[j] += a * (tgt[j] - this.curve[j]);
    return this.result();
  }

  result() {
    return { low: this.low, high: this.high, curve: this.curve, curveMix: this.plateauMix, p1: this.p1, p99: this.p99 };
  }

  /** Copy the current window/curve into THERMAL_SENSOR_FRAG uniforms. */
  applyTo(u) {
    if (!Number.isFinite(this.center)) return;
    u.uLow.value = this.low;
    u.uHigh.value = this.high;
    u.uCurve.value.set(this.curve);
    u.uCurveMix.value = this.plateauMix;
  }
}

// microbolometer sensor model: radiance -> apparent temperature, optics, noise, FPN, AGC, palette
const irSensorU = createThermalSensorUniforms();
irSensorU.uRadiance.value = irRT.texture;
irSensorU.uSensorRes.value.set(SENSOR_W, SENSOR_H);
irSensorU.uFill.value = 1;
irSensorU.uOSD.value = 1;
const irOutMat = new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3, vertexShader: THERMAL_FULLSCREEN_VERT, fragmentShader: THERMAL_GLSL + THERMAL_SENSOR_FRAG,
  uniforms: irSensorU, depthTest: false, depthWrite: false,
});
// small apparent-temperature image read back for the automatic gain control
const agcRT = target(80, 64, { type: THREE.FloatType, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter });
const agcMat = new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3, vertexShader: THERMAL_FULLSCREEN_VERT, fragmentShader: THERMAL_GLSL + THERMAL_DOWNSAMPLE_FRAG,
  uniforms: createThermalDownsampleUniforms(irSensorU, 80, 64), depthTest: false, depthWrite: false,
});
irSensorU.uSmall.value = agcRT.texture;
const agcBuf = new Float32Array(80 * 64 * 4);
const agc = new ThermalAGC({ width: 80, height: 64, plateauMix: 0.35 });
function updateAGC(dt) {
  runPass(agcMat, agcRT);
  renderer.readRenderTargetPixels(agcRT, 0, 0, 80, 64, agcBuf);
  agc.update(agcBuf, dt);
  agc.applyTo(irSensorU);
}

const droneTarget = new THREE.Vector3();
let droneAz = 0;
function updateSensorPose(dt) {
  if (params.sensorSource === 0) {
    // follow the main (mouse-controlled) view: same pose and vertical field of view
    sensorCam.position.copy(camera.position);
    sensorCam.quaternion.copy(camera.quaternion);
    // the EO/IR payload stays above the sea when the main view dives (LWIR cannot see through water)
    sensorCam.position.y = Math.max(sensorCam.position.y, probeBuf[CAM_PROBE * 4] + 0.6);
    droneTarget.copy(controls.target);
    if (sensorCam.fov !== camera.fov) { sensorCam.fov = camera.fov; sensorCam.updateProjectionMatrix(); }
    sensorCam.updateMatrixWorld();
    return;
  }
  groupCentroid(droneTarget);
  droneAz += params.droneOrbit * D2R * dt;
  const ang = Math.atan2(lightDir.z, lightDir.x) + Math.PI + 0.9 + droneAz;
  const t = simTime;
  sensorCam.position.set(droneTarget.x + Math.cos(ang) * params.droneRange, params.droneAlt + 0.4 * Math.sin(t * 0.7),
    droneTarget.z + Math.sin(ang) * params.droneRange);
  // residual gimbal jitter (stabilised, ~0.05 deg)
  droneTarget.x += 0.04 * Math.sin(t * 2.3); droneTarget.z += 0.04 * Math.cos(t * 1.9);
  sensorCam.lookAt(droneTarget);
  if (sensorCam.fov !== params.sensorFov) { sensorCam.fov = params.sensorFov; sensorCam.updateProjectionMatrix(); }
  sensorCam.updateMatrixWorld();
}

// render the scene from another camera: swap the view-dependent uniforms, then restore them
const _saveC = new THREE.Vector2(), _saveR = new THREE.Vector2(), _saveXZ = new THREE.Vector2(), _saveDome = new THREE.Vector3();
function renderView(cam, rt, refr) {
  _saveC.copy(oceanU.uCenter.value); _saveR.copy(oceanU.uResolution.value); _saveXZ.copy(skyU.uCamXZ.value); _saveDome.copy(skyDome.position);
  const savePix = skyU.uPixAng.value, saveRefr = oceanU.uRefr.value, saveDepth = oceanU.uRefrDepth.value;
  oceanU.uCenter.value.set(cam.position.x, cam.position.z);
  oceanU.uResolution.value.set(rt.width, rt.height);
  oceanU.uNear.value = cam.near; oceanU.uFar.value = cam.far;
  skyU.uCamXZ.value.set(cam.position.x, cam.position.z);
  skyU.uPixAng.value = 2 * Math.tan(cam.fov * D2R / 2) / rt.height;
  skyDome.position.copy(cam.position);
  if (refr) {
    oceanU.uRefr.value = refr.texture; oceanU.uRefrDepth.value = refr.depthTexture;
    cam.layers.set(PEOPLE_LAYER);
    renderer.setRenderTarget(refr);
    renderer.setClearColor(0x000000, 0); renderer.clear();
    renderer.render(scene, cam);
    renderer.setClearColor(0x000000, 1);
    cam.layers.set(0);
  }
  renderer.setRenderTarget(rt);
  renderer.render(scene, cam);
  oceanU.uCenter.value.copy(_saveC); oceanU.uResolution.value.copy(_saveR); skyU.uCamXZ.value.copy(_saveXZ); skyDome.position.copy(_saveDome);
  skyU.uPixAng.value = savePix; oceanU.uRefr.value = saveRefr; oceanU.uRefrDepth.value = saveDepth;
  oceanU.uNear.value = camera.near; oceanU.uFar.value = camera.far;
}
const winRGB = document.getElementById('winRGB'), winIR = document.getElementById('winIR');
const PALETTES = ['WHITE-HOT', 'BLACK-HOT', 'IRONBOW', 'RAINBOW'];
function sensorRects() {
  const w = Math.round(Math.min(window.innerWidth * 0.27, 400)), h = Math.round(w * SENSOR_H / SENSOR_W);
  return [{ x: 12, y: 12, w, h }, { x: 12 + w + 10, y: 12, w, h }];
}
function layoutSensorWindows() {
  const [a, b] = sensorRects();
  for (const [el, r] of [[winRGB, a], [winIR, b]]) {
    el.style.display = params.sensors ? 'block' : 'none';
    el.style.left = r.x + 'px'; el.style.width = r.w + 'px'; el.style.height = r.h + 'px'; el.style.bottom = r.y + 'px';
  }
}
function drawToWindow(mat, r) {
  renderer.setViewport(r.x, r.y, r.w, r.h);
  renderer.setScissor(r.x, r.y, r.w, r.h);
  fsQuad.material = mat;
  renderer.render(fsScene, fsCam);
}
let sensorFrame = 0;
function renderSensors(dtReal) {
  if (!params.sensors) return;
  sensorFrame++;
  // EO (RGB): same renderer and shading as the main view, from the gimbal
  renderView(sensorCam, rgbRT, sensorRefrRT);
  // LWIR: every material emits in-band radiance; lifejacket strobes and probe markers are not thermal sources
  THERM.uThermal.value = 1;
  people.forEach(P => { P.glow.visible = false; P.probeGroup.visible = false; });
  renderView(sensorCam, irRT, null);
  THERM.uThermal.value = 0;
  people.forEach(P => { P.glow.visible = true; P.probeGroup.visible = params.showProbes; });
  if (sensorFrame % 3 === 0) updateAGC(dtReal * 3);
  // composite into the two windows
  renderer.setRenderTarget(null);
  renderer.autoClear = false;
  renderer.setScissorTest(true);
  const [ra, rb] = sensorRects();
  rgbOutMat.uniforms.uExposure.value = compositePass.uniforms.uExposure.value;
  rgbOutMat.uniforms.uFrame.value = sensorFrame % 64;
  drawToWindow(rgbOutMat, ra);
  const iu = irSensorU, pr = renderer.getPixelRatio();
  iu.uPalette.value = params.palette;
  iu.uFrame.value = sensorFrame;
  iu.uViewport.value.set(rb.x * pr, rb.y * pr, rb.w * pr, rb.h * pr);
  iu.uNETD.value = 0.045 * params.irNoise;
  iu.uFPN.value = 0.5 * params.irNoise;
  iu.uFFC.value = (simTime % 180) / 180;      // offset drift builds up until the next shutter (FFC) event
  iu.uTref.value = THERM.uTair.value;
  drawToWindow(irOutMat, rb);
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, window.innerWidth, window.innerHeight);
  renderer.autoClear = true;
}
function updateSensorOSD() {
  if (!params.sensors) return;
  const d = sensorCam.position.distanceTo(droneTarget);
  const zh = LANG === 'zh', alt = sensorCam.position.y.toFixed(0), rng = d.toFixed(0);
  const hfov = (2 * Math.atan(Math.tan(sensorCam.fov * D2R / 2) * SENSOR_W / SENSOR_H) * R2D).toFixed(1);
  const tw = params.waterTempC.toFixed(1), ta = params.airTempC.toFixed(1), rh = (params.humidity * 100).toFixed(0);
  document.getElementById('osdRGB').textContent = zh ? `高度 ${alt} m  距离 ${rng} m  水平视场 ${hfov}°` : `ALT ${alt} m  RNG ${rng} m  HFOV ${hfov}°`;
  document.getElementById('osdIR').textContent = zh ? `水温 ${tw}°C  气温 ${ta}°C  湿度 ${rh}%` : `Tw ${tw}°C  Ta ${ta}°C  RH ${rh}%`;
  document.getElementById('osdPal').textContent = i18n(PALETTES[params.palette]);
  document.getElementById('osdSpan').textContent = Number.isFinite(agc.low)
    ? `${(agc.low - 273.15).toFixed(1)} … ${(agc.high - 273.15).toFixed(1)}°C` : i18n('AGC …');
}
function updateThermalEnv() {
  THERM.uTwater.value = params.waterTempC + 273.15;
  THERM.uTair.value = params.airTempC + 273.15;
  THERM.uRH.value = params.humidity;
  THERM.uWindSpd.value = params.windSpeed;
  THERM.uCloudFrac.value = clamp(params.clouds * 0.9 + params.veil * 0.3, 0, 1);
  THERM.uTcloud.value = params.airTempC + 273.15 - 6.5 * 1.5;                 // low cloud base ~1.5 km, standard lapse rate
  const E = oceanU.uSunE.value;
  THERM.uSunIrr.value = 1000 * (E.x * 0.2126 + E.y * 0.7152 + E.z * 0.0722) / 14 * Math.max(lightDir.y, 0);  // ~W/m2 on a horizontal surface
}
const fSensor = gui.addFolder('Drone EO / IR cameras');
fSensor.add(params, 'sensors').name('Show camera windows').onChange(layoutSensorWindows);
fSensor.add(params, 'sensorSource', { 'Follow main view (mouse)': 0, 'Drone gimbal': 1 }).name('Camera view');
fSensor.add(params, 'droneAlt', 10, 400, 1).name('Drone altitude (m)');
fSensor.add(params, 'droneRange', 20, 800, 1).name('Horizontal range (m)');
fSensor.add(params, 'sensorFov', 2, 50, 0.1).name('Zoom: vertical FOV (°)');
fSensor.add(params, 'droneOrbit', -10, 10, 0.1).name('Orbit (°/s)');
fSensor.add(params, 'palette', { 'White-hot': 0, 'Black-hot': 1, 'Ironbow': 2, 'Rainbow': 3 }).name('IR palette');
fSensor.add(params, 'irNoise', 0, 2, 0.01).name('IR sensor noise ×');
fSensor.add(params, 'waterTempC', -2, 32, 0.1).name('Sea temperature (°C)');
fSensor.add(params, 'airTempC', -10, 40, 0.1).name('Air temperature (°C)');
fSensor.add(params, 'humidity', 0.1, 1, 0.01).name('Relative humidity');
window.addEventListener('resize', layoutSensorWindows);
layoutSensorWindows();

