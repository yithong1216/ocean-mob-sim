// ============================================================================
// Cinematic post-processing and lens (feature module)
// ============================================================================
// Runs inside the final composite pass, so every pass inserted after the RenderPass (composer index 1..) is included:
//   scene depth (captured right after the RenderPass, linearised)
//   -> TAA: Halton(2,3) projection jitter, camera reprojection from depth, YCoCg variance clipping, Catmull-Rom history
//   -> energy-conserving dual-filter bloom (Karis-averaged first downsample, levels mixed into the image, not added)
//   -> auto exposure: centre-weighted log-average luminance, EV clamps, separate brightening/darkening speeds
//   -> depth of field: thin-lens circle of confusion, autofocus, polygonal-aperture gather at half resolution
//   -> composite: camera motion blur, lens water droplets, sharpening, chromatic aberration, bloom, sun ghosts and
//      starburst (occluded by clouds, people and waves), lens dirt, exposure, white balance and grade, AgX, grain.
// The RGB sensor window keeps the basic composite of 01_setup.js (09_sensors.js captured its shader at load time).
Object.assign(params, {
  cineAA: 1, cineSharpen: 0.3,
  cineAutoExp: true, cineAEStrength: 0.75, cineEVMin: -2.0, cineEVMax: 1.5, cineAdaptUp: 3.0, cineAdaptDown: 1.0,
  cineBloom: true,
  cineFlareOn: true, cineFlare: 1.0, cineStarburst: 1.0,
  cineDirtOn: true, cineDirt: 1.0,
  cineCAOn: true, cineCA: 0.5,
  cineGrainOn: true, cineGrain: 0.35,
  cineVignette: 1.0,
  cineDof: true, cineFstop: 2.8, cineAF: 0, cineFocusDist: 16,
  cineMB: true, cineShutter: 180,
  cineDrops: true, cineRainLens: 0,
  cineTemp: 5150, cineTint: 0, cineContrast: 1, cineSat: 1, cineLift: 0, cineGamma: 1, cineGain: 1,
});
const CINE_SUN_R = 0.004675;                                    // solar angular radius (rad), as SUN_ANG_R in the sky
const cineQ = () => clamp(Math.round(params.quality ?? 2), 0, 3);
const CINE_BLOOM_LEVELS = [5, 6, 7, 8];
const CINE_DOF_TAPS = [24, 32, 44, 60], CINE_DOF_MAXR = [8, 11, 14, 18];   // gather taps, max CoC (half-res px)
const CINE_MB_TAPS = [6, 8, 10, 14];

// ---------------------------------------------------------------------------- shaders
const CINE_COMMON = /* glsl */`
float luma(vec3 c) { return dot(c, vec3(0.2126, 0.7152, 0.0722)); }
float hash12(vec2 p) { vec3 p3 = fract(vec3(p.xyx) * 0.1031); p3 += dot(p3, p3.yzx + 33.33); return fract((p3.x + p3.y) * p3.z); }
`;
// linear view depth (m) from the resolved depth buffer
const CINE_LINZ_FRAG = /* glsl */`
uniform sampler2D tDepth;
uniform float uNear, uFar;
layout(location = 0) out vec4 fragColor;
void main() {
  float d = texelFetch(tDepth, ivec2(gl_FragCoord.xy), 0).r;
  fragColor = vec4(min(uNear * uFar / (uFar - d * (uFar - uNear)), 60000.0), 0.0, 0.0, 1.0);
}`;
const CINE_TAA_FRAG = CINE_COMMON + /* glsl */`
uniform sampler2D tCur, tHist, tZ, tZPrev;
uniform vec2 uRes, uProjK;
uniform mat4 uCamWorld, uPrevVP;
uniform float uAlpha, uReset;
layout(location = 0) out vec4 fragColor;
vec3 toY(vec3 c) { return vec3(0.25 * c.r + 0.5 * c.g + 0.25 * c.b, 0.5 * c.r - 0.5 * c.b, -0.25 * c.r + 0.5 * c.g - 0.25 * c.b); }
vec3 toRGB(vec3 c) { return vec3(c.x + c.y - c.z, c.x + c.z, c.x - c.y - c.z); }
// reversible luminance compression: blending and clipping in this space keeps HDR glints from flickering
vec3 tm(vec3 c) { return c / (1.0 + luma(c)); }
vec3 itm(vec3 c) { return c / max(1.0 - luma(c), 1e-4); }
vec3 histCR(vec2 uv) {      // Catmull-Rom history in 5 bilinear taps
  vec2 sp = uv * uRes, t1 = floor(sp - 0.5) + 0.5, f = sp - t1;
  vec2 w0 = f * (-0.5 + f * (1.0 - 0.5 * f)), w1 = 1.0 + f * f * (-2.5 + 1.5 * f);
  vec2 w2 = f * (0.5 + f * (2.0 - 1.5 * f)), w3 = f * f * (-0.5 + 0.5 * f), w12 = w1 + w2;
  vec2 t0 = (t1 - 1.0) / uRes, t3 = (t1 + 2.0) / uRes, t12 = (t1 + w2 / w12) / uRes;
  vec3 r = textureLod(tHist, vec2(t12.x, t0.y), 0.0).rgb * (w12.x * w0.y)
         + textureLod(tHist, vec2(t0.x, t12.y), 0.0).rgb * (w0.x * w12.y)
         + textureLod(tHist, t12, 0.0).rgb * (w12.x * w12.y)
         + textureLod(tHist, vec2(t3.x, t12.y), 0.0).rgb * (w3.x * w12.y)
         + textureLod(tHist, vec2(t12.x, t3.y), 0.0).rgb * (w12.x * w3.y);
  float ws = w12.x * w0.y + w0.x * w12.y + w12.x * w12.y + w3.x * w12.y + w12.x * w3.y;
  return max(r / ws, vec3(0.0));
}
vec3 clipAABB(vec3 q, vec3 mn, vec3 mx) {
  vec3 c = 0.5 * (mx + mn), e = 0.5 * (mx - mn) + 1e-5, v = q - c, u = abs(v / e);
  float m = max(u.x, max(u.y, u.z));
  return m > 1.0 ? c + v / m : q;
}
void main() {
  ivec2 p = ivec2(gl_FragCoord.xy), pmax = ivec2(uRes) - 1;
  vec2 uv = gl_FragCoord.xy / uRes;
  vec3 m1 = vec3(0.0), m2 = vec3(0.0), nmin = vec3(1e9), nmax = vec3(-1e9), cur = vec3(0.0);
  float zc = 1e9; ivec2 pc = p;
  for (int j = -1; j <= 1; j++) for (int i = -1; i <= 1; i++) {
    ivec2 q = clamp(p + ivec2(i, j), ivec2(0), pmax);
    vec3 c = toY(tm(texelFetch(tCur, q, 0).rgb));
    if (i == 0 && j == 0) cur = c;
    m1 += c; m2 += c * c; nmin = min(nmin, c); nmax = max(nmax, c);
    float z = texelFetch(tZ, q, 0).r;
    if (z < zc) { zc = z; pc = q; }
  }
  // camera reprojection of the nearest surface in the 3x3 (keeps silhouettes from trailing)
  vec2 uvc = (vec2(pc) + 0.5) / uRes;
  vec4 pp = uPrevVP * (uCamWorld * vec4(vec3((uvc * 2.0 - 1.0) * uProjK * zc, -zc), 1.0));
  vec2 prevUV = uv - (uvc - (pp.xy / pp.w * 0.5 + 0.5));
  if (uReset > 0.5 || pp.w <= 0.0 || any(lessThan(prevUV, vec2(0.0))) || any(greaterThan(prevUV, vec2(1.0)))) {
    fragColor = vec4(texelFetch(tCur, p, 0).rgb, 1.0);
    return;
  }
  vec3 mu = m1 / 9.0, sig = sqrt(max(m2 / 9.0 - mu * mu, 0.0));
  vec3 hist = clipAABB(toY(tm(histCR(prevUV))), max(mu - 1.1 * sig, nmin), min(mu + 1.1 * sig, nmax));
  float vel = length((uv - prevUV) * uRes);
  float a = mix(uAlpha, 0.4, clamp(vel / 48.0, 0.0, 1.0));
  // the people bob and the waves move without motion vectors: where the depth seen there last frame does not
  // match the reprojected depth (moving silhouette, disocclusion), trust the current frame
  float zp = texelFetch(tZPrev, clamp(ivec2(prevUV * uRes), ivec2(0), pmax), 0).r;
  float dz = abs(zp - pp.w) / max(pp.w, 0.1);
  a = mix(a, 0.85, smoothstep(0.06, 0.2, dz) * step(zc, 5000.0));
  fragColor = vec4(itm(toRGB(mix(hist, cur, a))), 1.0);
}`;
// bloom: 13-tap downsample (Karis average on the first level), 9-tap tent upsample, as web-ocean's post.js
const CINE_DOWN_FRAG = CINE_COMMON + /* glsl */`
uniform sampler2D tSrc;
uniform vec2 uTexel, uOut;
uniform float uKaris;
layout(location = 0) out vec4 fragColor;
vec3 tap(vec2 uv) { return textureLod(tSrc, uv, 0.0).rgb; }
float karis(vec3 c) { return 1.0 / (1.0 + luma(c) * 0.25); }
void main() {
  vec2 uv = gl_FragCoord.xy / uOut, t = uTexel;
  vec3 a = tap(uv + t * vec2(-2.0, -2.0)), b = tap(uv + t * vec2(0.0, -2.0)), c = tap(uv + t * vec2(2.0, -2.0));
  vec3 d = tap(uv + t * vec2(-2.0, 0.0)), e = tap(uv), f = tap(uv + t * vec2(2.0, 0.0));
  vec3 g = tap(uv + t * vec2(-2.0, 2.0)), h = tap(uv + t * vec2(0.0, 2.0)), i = tap(uv + t * vec2(2.0, 2.0));
  vec3 j = tap(uv + t * vec2(-1.0, -1.0)), k = tap(uv + t * vec2(1.0, -1.0)), l = tap(uv + t * vec2(-1.0, 1.0)), m = tap(uv + t * vec2(1.0, 1.0));
  vec3 o;
  if (uKaris > 0.5) {
    vec3 g0 = (a + b + d + e) * 0.25, g1 = (b + c + e + f) * 0.25, g2 = (d + e + g + h) * 0.25, g3 = (e + f + h + i) * 0.25, g4 = (j + k + l + m) * 0.25;
    float w0 = karis(g0) * 0.125, w1 = karis(g1) * 0.125, w2 = karis(g2) * 0.125, w3 = karis(g3) * 0.125, w4 = karis(g4) * 0.5;
    o = (g0 * w0 + g1 * w1 + g2 * w2 + g3 * w3 + g4 * w4) / (w0 + w1 + w2 + w3 + w4);
  } else {
    o = e * 0.125 + (a + c + g + i) * 0.03125 + (b + d + f + h) * 0.0625 + (j + k + l + m) * 0.125;
  }
  fragColor = vec4(min(o, vec3(6.0e4)), 1.0);
}`;
// up[m] = down[m] + tent(up[m+1]): the top level ends up holding the sum of all levels
const CINE_UP_FRAG = /* glsl */`
uniform sampler2D tSrc, tAdd;
uniform vec2 uTexel, uOut;
layout(location = 0) out vec4 fragColor;
vec3 tap(vec2 uv) { return textureLod(tSrc, uv, 0.0).rgb; }
void main() {
  vec2 uv = gl_FragCoord.xy / uOut, t = uTexel;
  vec3 o = tap(uv) * 4.0;
  o += (tap(uv + vec2(-t.x, 0.0)) + tap(uv + vec2(t.x, 0.0)) + tap(uv + vec2(0.0, -t.y)) + tap(uv + vec2(0.0, t.y))) * 2.0;
  o += tap(uv - t) + tap(uv + vec2(t.x, -t.y)) + tap(uv + vec2(-t.x, t.y)) + tap(uv + t);
  fragColor = vec4(o / 16.0 + textureLod(tAdd, uv, 0.0).rgb, 1.0);
}`;
// metering: centre-weighted log luminance on a 64x64 grid
const CINE_LUM_FRAG = CINE_COMMON + /* glsl */`
uniform sampler2D tSrc;
uniform float uAspect;
layout(location = 0) out vec4 fragColor;
void main() {
  vec2 uv = gl_FragCoord.xy / 64.0;
  float l = clamp(log2(max(luma(textureLod(tSrc, uv, 0.0).rgb), 1e-5)), -14.0, 16.0);
  vec2 d = (uv - 0.5) * vec2(uAspect, 1.0);
  float w = exp(-dot(d, d) * 4.0) + 0.2;
  fragColor = vec4(l * w, w, 0.0, 1.0);
}`;
const CINE_RED_FRAG = /* glsl */`
uniform sampler2D tSrc;
layout(location = 0) out vec4 fragColor;
void main() {
  ivec2 b = ivec2(gl_FragCoord.xy) * 8;
  vec2 s = vec2(0.0);
  for (int j = 0; j < 8; j++) for (int i = 0; i < 8; i++) s += texelFetch(tSrc, b + ivec2(i, j), 0).xy;
  fragColor = vec4(s, 0.0, 1.0);
}`;
// 1x1 state: x = adapted exposure (EV relative to manual), y = sun visibility, z = metered log2 luminance
const CINE_STATE_FRAG = CINE_COMMON + /* glsl */`
uniform sampler2D tLum, tPrev, tScene;
uniform float uDt, uMinEV, uMaxEV, uUp, uDown, uKey, uStrength, uLogManual, uSunExpect, uCloudVis;
uniform vec4 uSun;           // xy sun uv, z disc radius (px), w sun above the horizon and in front
uniform vec2 uRes;
layout(location = 0) out vec4 fragColor;
void main() {
  vec4 prev = texelFetch(tPrev, ivec2(0), 0);
  vec2 acc = vec2(0.0);
  for (int j = 0; j < 8; j++) for (int i = 0; i < 8; i++) acc += texelFetch(tLum, ivec2(i, j), 0).xy;
  float avgLog = acc.x / max(acc.y, 1e-6);
  // partial adaptation: a bright scene should still read brighter than a dark one
  float target = clamp(uStrength * (log2(uKey) - avgLog - uLogManual), uMinEV, uMaxEV);
  float ev = prev.x;
  if (prev.w < 0.5) ev = target;
  else ev += (target - ev) * (1.0 - exp(-uDt * (target < ev ? uUp : uDown)));
  // how much of the solar disc reaches the lens (clouds, people, waves): measured on the rendered frame
  float vis = 0.0;
  if (uSun.w > 0.5) {
    if (all(greaterThan(uSun.xy, vec2(0.002))) && all(lessThan(uSun.xy, vec2(0.998)))) {
      float s = 0.0;
      for (int k = 0; k < 16; k++) {
        float a = float(k) * 2.39996, r = sqrt((float(k) + 0.5) / 16.0) * 0.8 * uSun.z;
        s += clamp(luma(textureLod(tScene, uSun.xy + vec2(cos(a), sin(a)) * r / uRes, 0.0).rgb) / uSunExpect, 0.0, 1.0);
      }
      vis = s / 16.0;
    } else {
      vis = uCloudVis;        // just outside the frame: the lens still catches it
    }
  }
  float v = prev.w < 0.5 ? vis : mix(prev.y, vis, 1.0 - exp(-uDt * 14.0));
  fragColor = vec4(ev, v, avgLog, 1.0);
}`;
// depth of field: half-resolution colour + signed circle of confusion (half-res px, negative = near field)
const CINE_COC_FRAG = /* glsl */`
uniform sampler2D tSrc, tZ;
uniform vec2 uRes;
uniform vec3 uDof;           // CoC scale (full-res px), focus distance (m), max CoC (half-res px)
layout(location = 0) out vec4 fragColor;
void main() {
  ivec2 p = ivec2(gl_FragCoord.xy) * 2, pm = ivec2(uRes) - 1;
  vec3 c = textureLod(tSrc, (vec2(p) + 1.0) / uRes, 0.0).rgb;
  float z = min(min(texelFetch(tZ, min(p, pm), 0).r, texelFetch(tZ, min(p + ivec2(1, 0), pm), 0).r),
                min(texelFetch(tZ, min(p + ivec2(0, 1), pm), 0).r, texelFetch(tZ, min(p + 1, pm), 0).r));
  float coc = 0.5 * uDof.x * (z - uDof.y) / z;
  fragColor = vec4(min(c, vec3(6.0e4)), clamp(coc, -uDof.z, uDof.z));
}`;
// near-field CoC tiles (8x8 half-res px), so foreground blur can spread over in-focus background
const CINE_TILE_FRAG = /* glsl */`
uniform sampler2D tSrc;
uniform vec2 uSrcRes;
layout(location = 0) out vec4 fragColor;
void main() {
  ivec2 b = ivec2(gl_FragCoord.xy) * 8, pm = ivec2(uSrcRes) - 1;
  float n = 0.0;
  for (int j = 0; j < 8; j++) for (int i = 0; i < 8; i++) n = max(n, -texelFetch(tSrc, min(b + ivec2(i, j), pm), 0).a);
  fragColor = vec4(n, 0.0, 0.0, 1.0);
}`;
// scatter-as-gather bokeh on a 7-blade aperture
const CINE_BOKEH_FRAG = /* glsl */`
uniform sampler2D tSrc, tTile;
uniform vec2 uHalfRes, uTileTexel;
uniform float uMaxR;
layout(location = 0) out vec4 fragColor;
void main() {
  vec2 uv = gl_FragCoord.xy / uHalfRes;
  vec4 c0 = textureLod(tSrc, uv, 0.0);
  float cs = abs(c0.a);
  float near = textureLod(tTile, uv, 0.0).r;
  near = max(near, max(max(textureLod(tTile, uv + vec2(uTileTexel.x, 0.0), 0.0).r, textureLod(tTile, uv - vec2(uTileTexel.x, 0.0), 0.0).r),
                       max(textureLod(tTile, uv + vec2(0.0, uTileTexel.y), 0.0).r, textureLod(tTile, uv - vec2(0.0, uTileTexel.y), 0.0).r)));
  float R = clamp(max(cs, near), 0.5, uMaxR);
  vec3 acc = c0.rgb; float tot = 1.0, spread = cs;
  float dr = 0.5 * R / sqrt(float(DOF_TAPS)) + 0.5;
  const float seg = 6.2831853 / 7.0;
  for (int i = 0; i < DOF_TAPS; i++) {
    float a = float(i) * 2.3999632 + 0.3;
    float k = cos(seg * 0.5) / cos(mod(a, seg) - seg * 0.5);         // aperture polygon
    float r = sqrt((float(i) + 0.5) / float(DOF_TAPS)) * R * mix(1.0, k, 0.6);
    vec4 s = textureLod(tSrc, uv + vec2(cos(a), sin(a)) * r / uHalfRes, 0.0);
    float ss = abs(s.a);
    if (s.a > c0.a) ss = min(ss, cs * 2.0);                          // background cannot bleed over a sharper foreground
    float m = smoothstep(r - dr, r + dr, ss);
    acc += mix(acc / tot, s.rgb, m);
    tot += 1.0;
    if (s.a < c0.a) spread = max(spread, m * ss);
  }
  fragColor = vec4(acc / tot, spread);
}`;
// lens droplets: the height field decays (running drops leave thinning trails), then the drops are drawn (MAX blend)
const CINE_DROPDECAY_FRAG = /* glsl */`
uniform sampler2D tPrev;
uniform vec2 uDecay;
layout(location = 0) out vec4 fragColor;
void main() {
  float h = texelFetch(tPrev, ivec2(gl_FragCoord.xy), 0).r;
  fragColor = vec4(max(h * uDecay.x - uDecay.y, 0.0), 0.0, 0.0, 1.0);
}`;
const CINE_DROP_VERT = /* glsl */`
in vec4 aD;
out vec3 vD;
void main() { vD = aD.xyz; gl_Position = vec4(position.xy, 0.0, 1.0); }`;
const CINE_DROP_FRAG = /* glsl */`
in vec3 vD;
layout(location = 0) out vec4 fragColor;
void main() {
  float d2 = dot(vD.xy, vD.xy);
  if (d2 > 1.0) discard;
  fragColor = vec4(vD.z * (1.0 - d2), 0.0, 0.0, 1.0);
}`;
// procedural lens dirt (generated once): smudges, dried water spots with rims, dust specks, wipe marks
const CINE_DIRT_FRAG = CINE_COMMON + /* glsl */`
uniform vec2 uRes;
layout(location = 0) out vec4 fragColor;
vec4 hash42(vec2 p) {
  vec4 p4 = fract(vec4(p.xyxy) * vec4(0.1031, 0.1030, 0.0973, 0.1099));
  p4 += dot(p4, p4.wzxy + 33.33);
  return fract((p4.xxyz + p4.yzzw) * p4.zywx);
}
float blobs(vec2 p, float cells, float prob, float rmin, float rmax, float ring, float seed) {
  vec2 g = p * cells, id = floor(g);
  float acc = 0.0;
  for (int j = -2; j <= 2; j++) for (int i = -2; i <= 2; i++) {
    vec2 c = id + vec2(i, j);
    vec4 h = hash42(c + seed);
    if (h.w > prob) continue;
    vec4 h2 = hash42(c * 1.7 + seed + 11.0);
    float r = mix(rmin, rmax, h.z * h.z);
    vec2 dd = (g - c - h.xy) / r;
    dd.x *= 1.0 + 0.35 * (h2.x - 0.5);
    float d = length(dd);
    float disc = smoothstep(1.0, 0.25, d);
    float rim = smoothstep(1.0, 0.9, d) * smoothstep(0.6, 0.92, d);
    acc += mix(disc, rim + disc * 0.2, ring) * (0.35 + 0.65 * h2.y);
  }
  return acc;
}
float vnoise(vec2 p) {
  vec2 i = floor(p), f = fract(p), u = f * f * (3.0 - 2.0 * f);
  return mix(mix(hash12(i), hash12(i + vec2(1.0, 0.0)), u.x), mix(hash12(i + vec2(0.0, 1.0)), hash12(i + 1.0), u.x), u.y);
}
void main() {
  vec2 p = gl_FragCoord.xy / uRes.y;
  float grain = 0.55 + 0.45 * vnoise(p * 90.0);
  float d = 0.1 * blobs(p, 3.0, 0.5, 0.3, 0.7, 0.1, 3.0) * grain         // faint greasy smudges
          + 0.3 * blobs(p, 11.0, 0.3, 0.2, 0.55, 0.8, 17.0)               // dried water spots
          + 0.6 * blobs(p, 34.0, 0.2, 0.1, 0.35, 0.0, 41.0);              // dust specks
  vec2 q = vec2(p.x * 0.9 + p.y * 0.4, p.y * 2.2 - p.x * 0.3);
  float wipe = vnoise(q * vec2(3.0, 14.0)) * vnoise(q * vec2(1.2, 2.5) + 7.0);
  d += 0.12 * smoothstep(0.3, 0.6, wipe) * grain;
  d *= 0.5 + 0.9 * vnoise(p * 2.5 + 2.0);
  fragColor = vec4(vec3(d) * vec3(1.0, 0.97, 0.92), 1.0);
}`;

const CINE_COMPOSITE_FRAG = CINE_COMMON + /* glsl */`
#define MB_TAPS_MAX 16
uniform sampler2D tDiffuse, tZ, tBloom, tHalf, tDof, tState, tDirt, tDrops;
uniform float uExposure, uFrame, uAspect;
uniform vec2 uRes, uProjK;
uniform vec4 uFx;            // bloom mix, 1/levels, auto exposure on, sharpen
uniform vec4 uLens;          // chromatic aberration, grain, vignette, dirt
uniform vec4 uSun;           // sun uv, ghost gain (on-screen fade included), starburst gain
uniform vec3 uSunCol;        // unoccluded sun irradiance (scene-referred)
uniform vec4 uDofP;          // on
uniform vec4 uMB;            // blur scale (shutter / 360), max length (px), on, taps
uniform mat4 uCamWorld, uPrevVP;
uniform vec4 uDrop;          // on, texel (screen-height units), refraction, map aspect
uniform vec3 uWB;
uniform vec4 uGrade;         // contrast, saturation, lift, gamma
uniform float uGain;
varying vec2 vUv;
vec3 agxContrast(vec3 x) {
  vec3 x2 = x * x, x4 = x2 * x2;
  return 15.5 * x4 * x2 - 40.14 * x4 * x + 31.96 * x4 - 6.868 * x2 * x + 0.4298 * x2 + 0.1191 * x - 0.00232;
}
vec3 agx(vec3 v) {
  mat3 m = mat3(0.842479062253094, 0.0423282422610123, 0.0423756549057051,
                0.0784335999999992, 0.878468636469772, 0.0784336,
                0.0792237451477643, 0.0791661274605434, 0.879142973793104);
  const float minEv = -12.47393, maxEv = 4.026069;
  vec3 x = clamp(log2(m * max(v, vec3(1e-10))), vec3(minEv), vec3(maxEv));
  return agxContrast((x - minEv) / (maxEv - minEv));
}
vec3 agxEotf(vec3 v) {
  mat3 mi = mat3(1.19687900512017, -0.0528968517574562, -0.0529716355144438,
                 -0.0980208811401368, 1.15190312990417, -0.0980434501171241,
                 -0.0990297440797205, -0.0989611768448433, 1.15107367264116);
  return mi * v;
}
vec3 agxLook(vec3 v, float sat, float power) {
  float l = luma(v);
  return l + sat * (pow(max(v, vec3(0.0)), vec3(power)) - l);
}
float ign(vec2 p) { return fract(52.9829189 * fract(dot(p, vec2(0.06711056, 0.00583715)))); }
vec3 tex(vec2 uv) { return textureLod(tDiffuse, uv, 0.0).rgb; }

// sun ghosts (aperture-shaped reflections between lens elements, along the line through the frame centre)
float aperture(vec2 d, float rot) {
  const float seg = 6.2831853 / 7.0;
  float a = atan(d.y, d.x) + rot;
  return length(d) / mix(1.0, cos(seg * 0.5) / cos(mod(a, seg) - seg * 0.5), 0.7);
}
vec3 ghosts(vec2 uv) {
  const float T[8] = float[8](0.38, 0.62, 0.88, 1.1, 1.32, 1.56, 1.84, 2.22);
  const float R[8] = float[8](0.018, 0.045, 0.013, 0.075, 0.03, 0.12, 0.055, 0.19);
  const vec3 C[8] = vec3[8](vec3(1.0, 0.62, 0.25), vec3(0.35, 0.75, 1.0), vec3(1.0, 0.9, 0.55), vec3(0.45, 1.0, 0.6),
                            vec3(1.0, 0.45, 0.6), vec3(0.55, 0.7, 1.0), vec3(1.0, 0.75, 0.4), vec3(0.4, 0.55, 1.0));
  vec2 asp = vec2(uAspect, 1.0), axis = vec2(0.5) - uSun.xy;
  vec3 acc = vec3(0.0);
  for (int i = 0; i < 8; i++) {
    vec2 d = (uv - uSun.xy - axis * T[i]) * asp;
    float q = aperture(d / R[i], 0.35);
    float disc = smoothstep(1.0, 0.8, q), rim = smoothstep(0.6, 0.97, q) * disc;
    // dispersion: the red edge sits slightly outside the blue one
    vec3 edge = vec3(smoothstep(1.02, 0.86, q), disc, smoothstep(0.98, 0.78, q));
    acc += mix(vec3(0.8), C[i], 0.6) * (edge * 0.4 + rim * 0.35) / (R[i] * R[i]);
  }
  // ghosts are washed out by the glare when they sit on the sun
  float near = length((uv - uSun.xy) * asp);
  return acc * 2.2e-5 * smoothstep(0.03, 0.25, near);
}
// diffraction starburst of the 7-blade iris (14 spikes), longer towards red
vec3 starburst(vec2 uv) {
  vec2 d = (uv - uSun.xy) * vec2(uAspect, 1.0);
  float rr = length(d), th = atan(d.y, d.x) + 0.21;
  const float seg = 6.2831853 / 14.0;
  float a = mod(th, seg) - seg * 0.5;
  float len = 0.06 + 0.12 * hash12(vec2(floor(th / seg), 3.7));
  float w = 1.3 / uRes.y + rr * 0.005;
  float line = exp(-pow(abs(sin(a)) * rr / w, 2.0));
  vec3 fall = exp(-rr / (len * vec3(1.15, 1.0, 0.85)));
  return line * fall / (1.0 + rr * uRes.y * 0.06);
}

void main() {
  vec2 uv = vUv;
  vec2 dv = (uv - 0.5) * vec2(uAspect, 1.0);
  float r2 = dot(dv, dv);
  // ---- water on the front element: the height field refracts like a thin lens (thin film just distorts)
  float dh = 0.0; vec2 slope = vec2(0.0);
  if (uDrop.x > 0.5) {
    vec2 tx = vec2(uDrop.y / uAspect * uDrop.w, uDrop.y);
    dh = texture(tDrops, uv).r;
    slope = vec2(texture(tDrops, uv + vec2(tx.x, 0.0)).r - texture(tDrops, uv - vec2(tx.x, 0.0)).r,
                 texture(tDrops, uv + vec2(0.0, tx.y)).r - texture(tDrops, uv - vec2(0.0, tx.y)).r) / (2.0 * uDrop.y);
    uv -= slope * uDrop.z * 0.25 * (1.0 - smoothstep(0.004, 0.012, dh));
  }
  // ---- camera motion blur (reprojected from depth, shutter angle), else sharpening + lateral chromatic aberration
  vec3 c;
  vec2 vel = vec2(0.0);
  if (uMB.z > 0.5) {
    float z = texture(tZ, uv).r;
    vec4 pp = uPrevVP * (uCamWorld * vec4(vec3((uv * 2.0 - 1.0) * uProjK * z, -z), 1.0));
    vel = pp.w > 0.0 ? (uv - (pp.xy / pp.w * 0.5 + 0.5)) * uMB.x : vec2(0.0);
    float lp = length(vel * uRes);
    if (lp > uMB.y) vel *= uMB.y / lp;
  }
  vec2 ca = (uv - 0.5) * r2 * uLens.x * 0.004;
  if (length(vel * uRes) > 0.75) {
    c = vec3(0.0);
    int n = int(uMB.w);
    float jit = ign(gl_FragCoord.xy + uFrame * 7.13) - 0.5;
    for (int i = 0; i < MB_TAPS_MAX; i++) {
      if (i >= n) break;
      vec2 u = uv + vel * ((float(i) + 0.5 + jit) / float(n) - 0.5);
      c += vec3(tex(u + ca).r, tex(u).g, tex(u - ca).b);
    }
    c /= float(n);
  } else {
    vec2 t = 1.0 / uRes;
    vec3 e = tex(uv), n0 = tex(uv + vec2(t.x, 0.0)), n1 = tex(uv - vec2(t.x, 0.0)), n2 = tex(uv + vec2(0.0, t.y)), n3 = tex(uv - vec2(0.0, t.y));
    vec3 mn = min(e, min(min(n0, n1), min(n2, n3))), mx = max(e, max(max(n0, n1), max(n2, n3)));
    // contrast-adaptive: sharpen less where the neighbourhood already has strong (HDR) contrast
    float amt = uFx.w * clamp(1.0 - (luma(mx) - luma(mn)) / (luma(mx) + 0.25), 0.0, 1.0);
    c = clamp(e + (4.0 * e - n0 - n1 - n2 - n3) * amt * 0.25, mn, mx);
    if (uLens.x > 0.0) c += vec3(tex(uv + ca).r - e.r, 0.0, tex(uv - ca).b - e.b);
  }
  // ---- depth of field (half-resolution bokeh blended in by the circle of confusion)
  if (uDofP.x > 0.5) {
    vec4 d = texture(tDof, uv);
    c = mix(c, d.rgb, smoothstep(0.35, 1.0, d.a));
  }
  // ---- lens droplets: inverted, defocused view through each drop, darker rims, a little scattered light
  vec3 bl = texture(tBloom, uv).rgb * uFx.y;
  if (uDrop.x > 0.5) {
    float m = smoothstep(0.003, 0.03, dh);           // the drop sits on the front element: its outline is defocused
    if (m > 0.0) {
      vec3 seen = textureLod(tHalf, clamp(vUv - slope * uDrop.z, 0.001, 0.999), 0.0).rgb;
      float rim = smoothstep(45.0, 100.0, length(slope));
      c = mix(c, seen * (0.94 - 0.3 * rim) + bl * 0.12, m);
    }
  }
  // ---- energy-conserving bloom, sun flare, lens dirt
  c = mix(c, bl, uFx.x);
  vec4 st = texelFetch(tState, ivec2(0), 0);
  float sunVis = st.y;
  if (sunVis > 1e-4 && (uSun.z > 0.0 || uSun.w > 0.0)) {
    vec3 E = uSunCol * sunVis;
    c += E * (ghosts(vUv) * uSun.z + starburst(vUv) * uSun.w);
  }
  if (uLens.w > 0.0) {
    // dirt scatters light from sources much brighter than the frame average (sun, glints, strobes)
    vec2 ds = (vUv - uSun.xy) * vec2(uAspect, 1.0);
    vec3 sunGlow = uSunCol * sunVis * uSun.z * 0.0025 / (1.0 + dot(ds, ds) * 40.0);
    float excess = smoothstep(2.0, 16.0, luma(bl) / exp2(st.z));
    c += texture(tDirt, vUv).rgb * (bl * excess * 0.18 + sunGlow) * uLens.w;
  }
  // ---- exposure (manual x adapted), optical vignette, white balance, grade
  c *= uExposure * (uFx.z > 0.5 ? exp2(st.x) : 1.0);
  c *= 1.0 - 0.10 * uLens.z * r2;
  c *= uWB;
  c = max(c, vec3(0.0));
  c = 0.18 * pow(c / 0.18 + 1e-7, vec3(uGrade.x));
  float l0 = luma(c);
  c = max(l0 + (c - l0) * uGrade.y, vec3(0.0));
  float lum = luma(c);
  c = mix(c, vec3(lum), smoothstep(0.7, 6.0, lum) * 0.55);
  vec3 o = agxEotf(agxLook(agx(c), 1.05, 1.06));
  o = clamp(o, 0.0, 1.0);
  o = pow(max(o * uGain + uGrade.z * (1.0 - o), 0.0), vec3(1.0 / uGrade.w));
  // ---- film grain (strongest in the mid-tones) and triangular dither
  if (uLens.y > 0.0) {
    vec2 gp = gl_FragCoord.xy + vec2(uFrame * 17.0, uFrame * 31.0);
    float g = hash12(gp) + hash12(gp + 71.3) - 1.0;
    float ol = luma(o);
    o += g * uLens.y * 0.055 * sqrt(max(ol * (1.0 - ol), 0.0));
  }
  o += (ign(gl_FragCoord.xy + uFrame * 5.588238) + ign(gl_FragCoord.xy * 1.37 + 17.0 + uFrame * 3.1) - 1.0) / 255.0;
  gl_FragColor = vec4(clamp(o, 0.0, 1.0), 1.0);
}`;

// ---------------------------------------------------------------------------- targets and materials
const cineT = (w, h, o) => target(Math.max(1, w), Math.max(1, h), o);
const CINE_NEAREST = { minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter };
const cineRT = {
  linZ: [cineT(drawSize.x, drawSize.y, { format: THREE.RedFormat, ...CINE_NEAREST }), cineT(drawSize.x, drawSize.y, { format: THREE.RedFormat, ...CINE_NEAREST })],
  hist: [cineT(drawSize.x, drawSize.y), cineT(drawSize.x, drawSize.y)],
  down: [], up: [],
  lum: cineT(64, 64, CINE_NEAREST), lum8: cineT(8, 8, { type: THREE.FloatType, ...CINE_NEAREST }),
  state: [cineT(1, 1, { type: THREE.FloatType, ...CINE_NEAREST }), cineT(1, 1, { type: THREE.FloatType, ...CINE_NEAREST })],
  coc: cineT(1, 1), tile: cineT(1, 1), dof: cineT(1, 1),
  drops: [cineT(1, 1, { format: THREE.RedFormat }), cineT(1, 1, { format: THREE.RedFormat })],
  dirt: cineT(1280, 720, { type: THREE.UnsignedByteType }),
};
for (let i = 0; i < 8; i++) { cineRT.down.push(cineT(1, 1)); cineRT.up.push(cineT(1, 1)); }
const cineMats = {
  linZ: passMat(CINE_LINZ_FRAG, { tDepth: { value: null }, uNear: { value: 0.1 }, uFar: { value: 1 } }),
  taa: passMat(CINE_TAA_FRAG, {
    tCur: { value: null }, tHist: { value: null }, tZ: { value: null }, tZPrev: { value: null }, uRes: { value: new THREE.Vector2() },
    uProjK: { value: new THREE.Vector2(1, 1) }, uCamWorld: { value: new THREE.Matrix4() }, uPrevVP: { value: new THREE.Matrix4() },
    uAlpha: { value: 0.1 }, uReset: { value: 1 },
  }),
  down: passMat(CINE_DOWN_FRAG, { tSrc: { value: null }, uTexel: { value: new THREE.Vector2() }, uOut: { value: new THREE.Vector2() }, uKaris: { value: 0 } }),
  up: passMat(CINE_UP_FRAG, { tSrc: { value: null }, tAdd: { value: null }, uTexel: { value: new THREE.Vector2() }, uOut: { value: new THREE.Vector2() } }),
  lum: passMat(CINE_LUM_FRAG, { tSrc: { value: null }, uAspect: { value: 1 } }),
  red: passMat(CINE_RED_FRAG, { tSrc: { value: cineRT.lum.texture } }),
  state: passMat(CINE_STATE_FRAG, {
    tLum: { value: cineRT.lum8.texture }, tPrev: { value: null }, tScene: { value: null }, uDt: { value: 0 },
    uMinEV: { value: -2 }, uMaxEV: { value: 2 }, uUp: { value: 3 }, uDown: { value: 1 }, uKey: { value: 0.25 }, uStrength: { value: 0.75 }, uLogManual: { value: 0 },
    uSunExpect: { value: 1 }, uCloudVis: { value: 1 }, uSun: { value: new THREE.Vector4() }, uRes: { value: new THREE.Vector2() },
  }),
  coc: passMat(CINE_COC_FRAG, { tSrc: { value: null }, tZ: { value: null }, uRes: { value: new THREE.Vector2() }, uDof: { value: new THREE.Vector3() } }),
  tile: passMat(CINE_TILE_FRAG, { tSrc: { value: null }, uSrcRes: { value: new THREE.Vector2() } }),
  bokeh: passMat(CINE_BOKEH_FRAG, {
    tSrc: { value: null }, tTile: { value: null }, uHalfRes: { value: new THREE.Vector2() }, uTileTexel: { value: new THREE.Vector2() }, uMaxR: { value: 8 },
  }, { DOF_TAPS: CINE_DOF_TAPS[cineQ()] }),
  decay: passMat(CINE_DROPDECAY_FRAG, { tPrev: { value: null }, uDecay: { value: new THREE.Vector2(1, 0) } }),
  dirt: passMat(CINE_DIRT_FRAG, { uRes: { value: new THREE.Vector2(1280, 720) } }),
};

// lens droplets drawn as quads into the height field
const CINE_MAX_DROPS = 240, CINE_DROP_SLIDE = 0.022;   // drops larger than this run down the lens
const cineDropGeo = new THREE.BufferGeometry();
{
  const idx = new Uint16Array(CINE_MAX_DROPS * 6);
  for (let i = 0; i < CINE_MAX_DROPS; i++) idx.set([i * 4, i * 4 + 1, i * 4 + 2, i * 4, i * 4 + 2, i * 4 + 3], i * 6);
  cineDropGeo.setIndex(new THREE.BufferAttribute(idx, 1));
  cineDropGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(CINE_MAX_DROPS * 12), 3).setUsage(THREE.DynamicDrawUsage));
  cineDropGeo.setAttribute('aD', new THREE.BufferAttribute(new Float32Array(CINE_MAX_DROPS * 16), 4).setUsage(THREE.DynamicDrawUsage));
  cineDropGeo.setDrawRange(0, 0);
}
const cineDropMesh = new THREE.Mesh(cineDropGeo, new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3, vertexShader: CINE_DROP_VERT, fragmentShader: CINE_DROP_FRAG, depthTest: false, depthWrite: false,
  blending: THREE.CustomBlending, blendEquation: THREE.MaxEquation, blendSrc: THREE.OneFactor, blendDst: THREE.OneFactor,
}));
cineDropMesh.frustumCulled = false;
const cineDropScene = new THREE.Scene();
cineDropScene.add(cineDropMesh);

// ---------------------------------------------------------------------------- composite (extends the core pass)
const cineU = compositePass.uniforms;
Object.assign(cineU, {
  tScene: { value: null }, tZ: { value: null }, tBloom: { value: null }, tHalf: { value: null }, tDof: { value: null },
  tState: { value: cineRT.state[0].texture }, tDirt: { value: cineRT.dirt.texture }, tDrops: { value: cineRT.drops[0].texture },
  uRes: { value: new THREE.Vector2() }, uProjK: cineMats.taa.uniforms.uProjK,
  uFx: { value: new THREE.Vector4() }, uLens: { value: new THREE.Vector4() }, uSun: { value: new THREE.Vector4() }, uSunCol: { value: new THREE.Vector3() },
  uDofP: { value: new THREE.Vector4() }, uMB: { value: new THREE.Vector4() },
  uCamWorld: cineMats.taa.uniforms.uCamWorld, uPrevVP: cineMats.taa.uniforms.uPrevVP,
  uDrop: { value: new THREE.Vector4() }, uWB: { value: new THREE.Vector3(1, 1, 1) }, uGrade: { value: new THREE.Vector4(1, 1, 0, 1) }, uGain: { value: 1 },
});
compositePass.textureID = 'tScene';            // ShaderPass binds the composer input here; tDiffuse is set by cinePost
compositePass.material.fragmentShader = CINE_COMPOSITE_FRAG;
compositePass.material.needsUpdate = true;
composer.removePass(bloomPass);                 // replaced by the dual-filter bloom below

// ---------------------------------------------------------------------------- per-frame state
const cine = {
  frame: 0, zIdx: 0, histIdx: 0, stateIdx: 0, dropIdx: 0, reset: true, dt: 1 / 60, dtAcc: 0, depthOK: false,
  jitter: new THREE.Vector2(), proj: new THREE.Matrix4(), projInv: new THREE.Matrix4(),
  vp: new THREE.Matrix4(), prevVP: new THREE.Matrix4(), prevPos: new THREE.Vector3(), prevQ: new THREE.Quaternion(), havePrev: false,
  focus: 16, dofScale: 0, sunUV: new THREE.Vector2(-9, -9), sunOK: false, sunFade: 0,
  drops: [], dirtReady: false, dofTaps: CINE_DOF_TAPS[cineQ()], wb: new THREE.Vector3(1, 1, 1),
};
const _cineV4 = new THREE.Vector4(), _cineSunP = new THREE.Vector3(), _cineFwd = new THREE.Vector3(), _cineHead = new THREE.Vector3();
const cineHalton = (i, b) => { let f = 1, r = 0; while (i > 0) { f /= b; r += f * (i % b); i = Math.floor(i / b); } return r; };
const CINE_JITTER = Array.from({ length: 8 }, (_, i) => [cineHalton(i + 1, 2) - 0.5, cineHalton(i + 1, 3) - 0.5]);
const cineTaaOn = () => params.cineAA === 1 && cine.depthOK;

function cineResize(w, h) {
  cineRT.linZ[0].setSize(w, h); cineRT.linZ[1].setSize(w, h); cineRT.hist[0].setSize(w, h); cineRT.hist[1].setSize(w, h);
  for (let i = 0; i < 8; i++) {
    const bw = Math.max(1, w >> (i + 1)), bh = Math.max(1, h >> (i + 1));
    cineRT.down[i].setSize(bw, bh); cineRT.up[i].setSize(bw, bh);
  }
  const hw = Math.max(1, w >> 1), hh = Math.max(1, h >> 1);
  cineRT.coc.setSize(hw, hh); cineRT.dof.setSize(hw, hh);
  cineRT.tile.setSize(Math.ceil(hw / 8), Math.ceil(hh / 8));
  const ds = cineQ() >= 2 ? 1 : 2;
  cineRT.drops[0].setSize(hw >> (ds - 1), hh >> (ds - 1)); cineRT.drops[1].setSize(hw >> (ds - 1), hh >> (ds - 1));
  cine.reset = true;
}
HOOKS.resize.push(cineResize);
cineResize(drawSize.x, drawSize.y);

// capture the scene depth right after the RenderPass (later passes may overwrite the composer targets)
{
  const rp = composer.passes[0], rpRender = rp.render.bind(rp);
  rp.render = (r, w, read, dt, mask) => {
    rpRender(r, w, read, dt, mask);
    cine.depthOK = !!(read && read.depthTexture) && !rp.renderToScreen;
    if (!cine.depthOK || !(params.cineAA === 1 || params.cineDof || params.cineMB)) return;
    const u = cineMats.linZ.uniforms;
    u.tDepth.value = read.depthTexture; u.uNear.value = camera.near; u.uFar.value = camera.far;
    cine.zIdx ^= 1;
    runPass(cineMats.linZ, cineRT.linZ[cine.zIdx]);
    const z = cineRT.linZ[cine.zIdx].texture;
    cineMats.taa.uniforms.tZ.value = cineMats.coc.uniforms.tZ.value = cineU.tZ.value = z;
    cineMats.taa.uniforms.tZPrev.value = cineRT.linZ[cine.zIdx ^ 1].texture;
  };
}

// colour temperature -> white-balance gains (Planckian locus, Kang et al. 2002), relative to 6504 K, G = 1
function cineCCT(T) {
  const t = 1e3 / T, t2 = t * t, t3 = t2 * t;
  const x = T <= 4000 ? -0.2661239 * t3 - 0.2343589 * t2 + 0.8776956 * t + 0.179910 : -3.0258469 * t3 + 2.1070379 * t2 + 0.2226347 * t + 0.240390;
  const x2 = x * x, x3 = x2 * x;
  const y = T <= 2222 ? -1.1063814 * x3 - 1.3481102 * x2 + 2.18555832 * x - 0.20219683
    : T <= 4000 ? -0.9549476 * x3 - 1.37418593 * x2 + 2.09137015 * x - 0.16748867 : 3.081758 * x3 - 5.8733867 * x2 + 3.75112997 * x - 0.37001483;
  const X = x / y, Z = (1 - x - y) / y;
  return [3.2404542 * X - 1.5371385 - 0.4985314 * Z, -0.969266 * X + 1.8760108 + 0.041556 * Z, 0.0556434 * X - 0.2040259 + 1.0572252 * Z];
}
function cineUpdateWB() {
  const a = cineCCT(clamp(params.cineTemp, 1500, 15000)), r = cineCCT(6504);
  const g = [r[0] / a[0], r[1] / a[1], r[2] / a[2]];
  const tint = Math.pow(2, -0.25 * params.cineTint);
  cine.wb.set(g[0] / g[1], tint, g[2] / g[1]);
}
cineUpdateWB();

// ---------------------------------------------------------------------------- lens droplets (CPU drops)
function cineDropsUpdate(dt, dtReal) {
  const D = cine.drops, C = cineCam;
  if (!params.cineDrops) { D.length = 0; C.splash = 0; return; }
  const hA = C.heightAboveWater;
  let rate = 22 * smoothstep(1.7, 0.35, hA) * (0.35 + clamp(C.waterVy, 0, 2)) * (0.5 + params.windSpeed / 14);
  rate += 14 * smoothstep(11, 20, params.windSpeed) * smoothstep(5, 1, hA);           // wind-blown spray near the surface
  const rain = Math.max(params.cineRainLens, params.rain ?? 0);
  rate += 28 * rain;
  const spawn = (big) => {
    if (D.length >= CINE_MAX_DROPS) return;
    const u = Math.random();
    const r = big ? 0.012 + 0.045 * u * u * u : 0.004 + 0.014 * u * u;          // radius, fraction of frame height
    D.push({ x: Math.random(), y: Math.random() * 1.1, r, vx: 0, vy: 0, pause: Math.random() * 0.6 });
  };
  let n = rate * dt * (dt > 0 ? 1 : 0);
  n = Math.floor(n) + (Math.random() < n % 1 ? 1 : 0);
  for (let i = 0; i < n; i++) spawn(Math.random() < 0.3 * (1 - rain * 0.5));
  for (; C.splash >= 1; C.splash--) spawn(true);
  // gravity across the lens: world down seen from the camera
  _cineFwd.set(0, -1, 0).applyQuaternion(_cineQ.copy(C.shown.q).invert());
  const gx = _cineFwd.x, gy = _cineFwd.y, aspect = drawSize.x / drawSize.y;
  for (const d of D) {
    d.r -= dtReal * 0.0006;                                                          // evaporation
    if (d.r > CINE_DROP_SLIDE) {
      d.pause -= dtReal;
      if (d.pause <= 0) {
        const v = 0.3 * (d.r - CINE_DROP_SLIDE) / 0.02;
        d.x += (gx * v / aspect + d.vx) * dtReal; d.y += gy * v * dtReal;
        d.vx += (Math.random() - 0.5) * 0.02 * dtReal * 60 - d.vx * 2 * dtReal;
        d.r -= dtReal * v * 0.02;                                                     // leaves a trail
        if (Math.random() < dtReal * 0.6) d.pause = 0.2 + Math.random() * 0.9;        // stick-slip
      }
    }
  }
  // merge touching drops (volume conserving)
  for (let i = 0; i < D.length; i++) for (let j = i + 1; j < D.length; j++) {
    const a = D[i], b = D[j], dx = (a.x - b.x) * aspect, dy = a.y - b.y, rr = (a.r + b.r) * 0.75;
    if (dx * dx + dy * dy < rr * rr) {
      const va = a.r ** 3, vb = b.r ** 3, s = va + vb;
      a.x = (a.x * va + b.x * vb) / s; a.y = (a.y * va + b.y * vb) / s; a.r = Math.cbrt(s); a.pause = Math.min(a.pause, b.pause);
      b.r = 0;
    }
  }
  for (let i = D.length - 1; i >= 0; i--) { const d = D[i]; if (d.r < 0.003 || d.y < -0.05 || d.y > 1.2 || d.x < -0.05 || d.x > 1.05) D.splice(i, 1); }
}
function cineDropsRender() {
  const D = cine.drops, pos = cineDropGeo.attributes.position.array, ad = cineDropGeo.attributes.aD.array;
  const aspect = drawSize.x / drawSize.y;
  let k = 0;
  for (const d of D) {
    if (k >= CINE_MAX_DROPS) break;
    const rx = d.r / aspect * 2, ry = d.r * 2 * (d.pause > 0 || d.r <= CINE_DROP_SLIDE ? 1 : 1.25);   // NDC half sizes (sliding drops elongate)
    const cx = d.x * 2 - 1, cy = d.y * 2 - 1, h = d.r * 40;
    const corners = [[-1, -1], [1, -1], [1, 1], [-1, 1]];
    for (let c = 0; c < 4; c++) {
      const [lx, ly] = corners[c];
      pos.set([cx + lx * rx, cy + ly * ry, 0], (k * 4 + c) * 3);
      ad.set([lx, ly, h, 0], (k * 4 + c) * 4);
    }
    k++;
  }
  cineDropGeo.attributes.position.needsUpdate = true; cineDropGeo.attributes.aD.needsUpdate = true;
  cineDropGeo.setDrawRange(0, k * 6);
  const prev = cineRT.drops[cine.dropIdx], next = cineRT.drops[cine.dropIdx ^ 1];
  const u = cineMats.decay.uniforms, dt = cine.dtAcc;
  u.tPrev.value = prev.texture; u.uDecay.value.set(Math.exp(-dt * 1.4), dt * 0.012);
  runPass(cineMats.decay, next);
  if (k > 0) {
    const ac = renderer.autoClear;
    renderer.autoClear = false;
    renderer.setRenderTarget(next);
    renderer.render(cineDropScene, fsCam);
    renderer.autoClear = ac;
  }
  cine.dropIdx ^= 1;
  return next;
}

// ---------------------------------------------------------------------------- focus, sun, matrices
function cineUpdateFocus() {
  camera.getWorldDirection(_cineFwd);
  let fd = params.cineFocusDist;
  if (params.cineAF !== 2) {
    _cineHead.copy(controls.target).setY(0.25).sub(camera.position);
    fd = Math.max(_cineHead.dot(_cineFwd), 0.5);
    if (params.cineAF === 0 && people.length) {
      // the person nearest the frame centre (by angle), focused on the head
      let best = 0.35 * camera.fov * D2R;
      for (const P of people) {
        _cineHead.copy(P.wetU.head.value).sub(camera.position);
        const z = _cineHead.dot(_cineFwd);
        if (z < 0.4) continue;
        const ang = Math.acos(clamp(z / _cineHead.length(), -1, 1));
        if (ang < best) { best = ang; fd = z; }
      }
    }
  }
  // focus pulls ease in log distance (about 0.4 s)
  const k = 1 - Math.exp(-cine.dtAcc * 5);
  cine.focus = cine.focus > 0 ? Math.exp(Math.log(cine.focus) + (Math.log(Math.max(fd, 0.3)) - Math.log(cine.focus)) * k) : fd;
  // thin lens: CoC on the sensor = f^2 / (N (s - f)) |z - s| / z, in pixels for a 24 mm sensor height
  const f = 0.012 / Math.tan(camera.fov * D2R / 2), s = Math.max(cine.focus, f * 2);
  cine.dofScale = f * f / (params.cineFstop * (s - f)) / 0.024 * drawSize.y;
}
function cineUpdateSun() {
  const ld = lightDir, E = oceanU.uSunE.value, cv = Math.max(sunEBuf[3], 0.05);
  _cineSunP.copy(ld).multiplyScalar(1e4).add(camera.position);
  _cineV4.set(_cineSunP.x, _cineSunP.y, _cineSunP.z, 1).applyMatrix4(cine.vp);
  const inFront = _cineV4.w > 0 && ld.y > -0.02;
  if (inFront) cine.sunUV.set(_cineV4.x / _cineV4.w * 0.5 + 0.5, _cineV4.y / _cineV4.w * 0.5 + 0.5);
  const out = inFront ? Math.max(Math.abs(cine.sunUV.x - 0.5), Math.abs(cine.sunUV.y - 0.5)) - 0.5 : 9;
  cine.sunFade = inFront ? 1 - smoothstep(0, 0.3, out) : 0;
  // unoccluded disc radiance, clamped like the sky shader, and how much of it a 80 % disc sample should see
  const k = 0.82 / (Math.PI * CINE_SUN_R * CINE_SUN_R) / cv;
  const Lexp = 0.2126 * Math.min(E.x * k, 6e4) + 0.7152 * Math.min(E.y * k, 6e4) + 0.0722 * Math.min(E.z * k, 6e4);
  cine.sunOK = inFront && cine.sunFade > 0 && Lexp > 1e-3;
  const su = cineMats.state.uniforms;
  su.uSun.value.set(cine.sunUV.x, cine.sunUV.y, CINE_SUN_R / (2 * Math.tan(camera.fov * D2R / 2) / drawSize.y), cine.sunOK ? 1 : 0);
  su.uSunExpect.value = Math.max(Lexp * 0.75, 1e-6);
  su.uCloudVis.value = sunEBuf[3] * cine.sunFade;
  cineU.uSunCol.value.set(E.x, E.y, E.z).divideScalar(cv);
}

HOOKS.lateUpdate.push((dt, dtReal) => {
  cine.dt = dtReal; cine.dtAcc += dtReal;
  cineDropsUpdate(dt, dtReal);
});
HOOKS.beforeRender.push(() => {
  cineCamApply();                                   // camera preset pose (cine_camera.js)
  camera.updateMatrixWorld();
  // unjittered matrices for reprojection, motion blur, the sun position and the TAA jitter
  cine.proj.copy(camera.projectionMatrix); cine.projInv.copy(camera.projectionMatrixInverse);
  cine.vp.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
  cineMats.taa.uniforms.uProjK.value.set(1 / cine.proj.elements[0], 1 / cine.proj.elements[5]);
  cineMats.taa.uniforms.uCamWorld.value.copy(camera.matrixWorld);
  if (!cine.havePrev) cine.prevVP.copy(cine.vp);
  // camera cut (preset jump, reset, large step): drop the TAA history and the motion blur for this frame
  const jump = camera.position.distanceTo(cine.prevPos) > 6 || camera.quaternion.angleTo(cine.prevQ) > 0.5;
  cine.cut = !cine.havePrev || jump;
  cineMats.taa.uniforms.uPrevVP.value.copy(cine.cut ? cine.vp : cine.prevVP);
  if (params.cineAA === 1) {
    const j = CINE_JITTER[cine.frame % 8];
    cine.jitter.set(j[0], j[1]);
    camera.projectionMatrix.elements[8] += 2 * j[0] / drawSize.x;
    camera.projectionMatrix.elements[9] += 2 * j[1] / drawSize.y;
    camera.projectionMatrixInverse.copy(camera.projectionMatrix).invert();
  }
  cineUpdateFocus();
  cineUpdateSun();
});
HOOKS.afterRender.push(() => {
  cine.prevVP.copy(cine.vp); cine.prevPos.copy(camera.position); cine.prevQ.copy(camera.quaternion); cine.havePrev = true;
  camera.projectionMatrix.copy(cine.proj); camera.projectionMatrixInverse.copy(cine.projInv);
  cineCamRestore();
  cine.frame++; cine.dtAcc = 0;
});

// ---------------------------------------------------------------------------- the post chain
function cinePost(read) {
  const q = cineQ(), W = drawSize.x, H = drawSize.y, U = cineU;
  if (!cine.dirtReady) { runPass(cineMats.dirt, cineRT.dirt); cine.dirtReady = true; }
  if (cine.dofTaps !== CINE_DOF_TAPS[q]) {
    cine.dofTaps = CINE_DOF_TAPS[q];
    cineMats.bokeh.defines.DOF_TAPS = cine.dofTaps; cineMats.bokeh.needsUpdate = true;
  }
  const sceneTex = read.texture;
  let src = sceneTex;
  // ---- TAA
  if (cineTaaOn()) {
    const u = cineMats.taa.uniforms, out = cineRT.hist[cine.histIdx ^ 1];
    u.tCur.value = sceneTex; u.tHist.value = cineRT.hist[cine.histIdx].texture;
    u.uRes.value.set(W, H); u.uReset.value = cine.reset || cine.cut ? 1 : 0; u.uAlpha.value = 0.1;
    runPass(cineMats.taa, out);
    cine.histIdx ^= 1; cine.reset = false;
    src = out.texture;
  } else cine.reset = true;
  // ---- bloom downsample chain (also feeds the metering and the droplets)
  const L = CINE_BLOOM_LEVELS[q], dm = cineMats.down.uniforms;
  for (let i = 0; i < L; i++) {
    const s = i === 0 ? src : cineRT.down[i - 1].texture, sw = i === 0 ? W : cineRT.down[i - 1].width, sh = i === 0 ? H : cineRT.down[i - 1].height;
    dm.tSrc.value = s; dm.uTexel.value.set(1 / sw, 1 / sh); dm.uOut.value.set(cineRT.down[i].width, cineRT.down[i].height); dm.uKaris.value = i === 0 ? 1 : 0;
    runPass(cineMats.down, cineRT.down[i]);
  }
  // ---- metering and adaptation (+ sun visibility for the flare)
  cineMats.lum.uniforms.tSrc.value = cineRT.down[3].texture; cineMats.lum.uniforms.uAspect.value = W / H;
  runPass(cineMats.lum, cineRT.lum);
  runPass(cineMats.red, cineRT.lum8);
  const su = cineMats.state.uniforms, sOut = cineRT.state[cine.stateIdx ^ 1];
  su.tPrev.value = cineRT.state[cine.stateIdx].texture; su.tScene.value = sceneTex; su.uRes.value.set(W, H);
  su.uDt.value = cine.dtAcc; su.uMinEV.value = params.cineEVMin;
  su.uMaxEV.value = params.cineEVMax * (1 - nightF);        // the night factor in 06_sky_state already adapts to the dark
  su.uUp.value = params.cineAdaptUp; su.uDown.value = params.cineAdaptDown; su.uStrength.value = params.cineAEStrength;
  su.uLogManual.value = Math.log2(Math.max(U.uExposure.value, 1e-6));
  runPass(cineMats.state, sOut);
  cine.stateIdx ^= 1;
  U.tState.value = sOut.texture;
  // ---- bloom upsample chain
  if (params.cineBloom || params.cineDirtOn || params.cineDrops) {
    const um = cineMats.up.uniforms;
    for (let i = L - 2; i >= 0; i--) {
      const s = i === L - 2 ? cineRT.down[L - 1] : cineRT.up[i + 1];
      um.tSrc.value = s.texture; um.tAdd.value = cineRT.down[i].texture;
      um.uTexel.value.set(1 / s.width, 1 / s.height); um.uOut.value.set(cineRT.up[i].width, cineRT.up[i].height);
      runPass(cineMats.up, cineRT.up[i]);
    }
  }
  // ---- depth of field
  const dofOn = params.cineDof && cine.depthOK && cine.dofScale * 0.5 > 0.05;
  if (dofOn) {
    const maxR = CINE_DOF_MAXR[q];
    const cu = cineMats.coc.uniforms;
    cu.tSrc.value = src; cu.uRes.value.set(W, H); cu.uDof.value.set(cine.dofScale, cine.focus, maxR);
    runPass(cineMats.coc, cineRT.coc);
    cineMats.tile.uniforms.tSrc.value = cineRT.coc.texture; cineMats.tile.uniforms.uSrcRes.value.set(cineRT.coc.width, cineRT.coc.height);
    runPass(cineMats.tile, cineRT.tile);
    const bu = cineMats.bokeh.uniforms;
    bu.tSrc.value = cineRT.coc.texture; bu.tTile.value = cineRT.tile.texture; bu.uMaxR.value = maxR;
    bu.uHalfRes.value.set(cineRT.coc.width, cineRT.coc.height); bu.uTileTexel.value.set(1 / cineRT.tile.width, 1 / cineRT.tile.height);
    runPass(cineMats.bokeh, cineRT.dof);
  }
  // ---- droplets
  // keep updating for a few seconds after the last drop so its trail and film fade out
  cine.dropTail = cine.drops.length > 0 ? 4 : (cine.dropTail || 0) - cine.dtAcc;
  const drops = params.cineDrops && cine.dropTail > 0;
  if (drops) U.tDrops.value = cineDropsRender().texture;
  // ---- composite uniforms
  U.tDiffuse.value = src;
  U.tBloom.value = cineRT.up[0].texture; U.tHalf.value = cineRT.down[1].texture; U.tDof.value = cineRT.dof.texture;
  U.uRes.value.set(W, H);
  U.uFrame.value = cine.frame % 4096;
  U.uFx.value.set(params.cineBloom ? params.bloom : 0, 1 / L, params.cineAutoExp ? 1 : 0, cineTaaOn() ? params.cineSharpen : params.cineSharpen * 0.4);
  U.uLens.value.set(params.cineCAOn ? params.cineCA : 0, params.cineGrainOn ? params.cineGrain : 0, params.cineVignette, params.cineDirtOn ? params.cineDirt : 0);
  const fl = params.cineFlareOn && cine.sunOK ? params.cineFlare : 0;
  // diffraction spikes grow as the iris closes (larger f-number); only while the sun itself is in frame
  const onScreen = smoothstep(0.05, 0.0, Math.max(Math.abs(cine.sunUV.x - 0.5), Math.abs(cine.sunUV.y - 0.5)) - 0.5);
  const iris = clamp(params.cineFstop / 8, 0.3, 1.6);
  U.uSun.value.set(cine.sunUV.x, cine.sunUV.y, fl * cine.sunFade, (params.cineFlareOn && cine.sunOK ? 1 : 0) * params.cineStarburst * iris * onScreen);
  U.uDofP.value.set(dofOn ? 1 : 0, 0, 0, 0);
  const mbOn = params.cineMB && cine.depthOK && !cine.cut;
  U.uMB.value.set(params.cineShutter / 360, 0.04 * H, mbOn ? 1 : 0, CINE_MB_TAPS[q]);
  U.uDrop.value.set(drops ? 1 : 0, 1 / cineRT.drops[0].height, 0.00045, cineRT.drops[0].height * W / (cineRT.drops[0].width * H));
  U.uWB.value.copy(cine.wb);
  U.uGrade.value.set(params.cineContrast, params.cineSat, params.cineLift, params.cineGamma);
  U.uGain.value = params.cineGain;
}
{
  const compRender = compositePass.render.bind(compositePass);
  compositePass.render = (r, w, read, dt, mask) => { cinePost(read); compRender(r, w, read, dt, mask); };
}

// ---------------------------------------------------------------------------- GUI
{
  const fe = cineGui.addFolder('Exposure & anti-aliasing');
  fe.add(params, 'cineAutoExp', { 'Manual': false, 'Auto (eye adaptation)': true }).name('Exposure');
  fe.add(params, 'cineAEStrength', 0, 1, 0.01).name('Auto: adaptation strength');
  fe.add(params, 'cineEVMin', -6, 0, 0.1).name('Auto: min EV');
  fe.add(params, 'cineEVMax', 0, 6, 0.1).name('Auto: max EV');
  fe.add(params, 'cineAdaptUp', 0.1, 10, 0.1).name('Adapt to bright (1/s)');
  fe.add(params, 'cineAdaptDown', 0.1, 10, 0.1).name('Adapt to dark (1/s)');
  fe.add(params, 'cineAA', { 'MSAA x4 only': 0, 'TAA + MSAA x4': 1 }).name('Anti-aliasing').onChange(() => { cine.reset = true; });
  fe.add(params, 'cineSharpen', 0, 1, 0.01).name('Sharpen');
  const fl = cineGui.addFolder('Bloom & lens');
  fl.add(params, 'cineBloom').name('Bloom (amount: Sky & light)');
  fl.add(params, 'cineFlareOn').name('Sun flare (ghosts, starburst)');
  fl.add(params, 'cineFlare', 0, 4, 0.01).name('Ghost intensity');
  fl.add(params, 'cineStarburst', 0, 4, 0.01).name('Starburst');
  fl.add(params, 'cineDirtOn').name('Lens dirt');
  fl.add(params, 'cineDirt', 0, 4, 0.01).name('Dirt intensity');
  fl.add(params, 'cineCAOn').name('Chromatic aberration');
  fl.add(params, 'cineCA', 0, 3, 0.01).name('Aberration amount');
  fl.add(params, 'cineGrainOn').name('Film grain');
  fl.add(params, 'cineGrain', 0, 2, 0.01).name('Grain amount');
  fl.add(params, 'cineVignette', 0, 4, 0.01).name('Vignette');
  fl.add(params, 'cineDrops').name('Lens water droplets');
  fl.add(params, 'cineRainLens', 0, 1, 0.01).name('Rain on lens');
  const fd = cineGui.addFolder('Depth of field & motion blur');
  fd.add(params, 'cineDof').name('Depth of field');
  fd.add(params, 'cineFstop', 1.2, 22, 0.1).name('Aperture (f-number)');
  fd.add(params, 'cineAF', { 'Auto: subject near centre': 0, 'Orbit target': 1, 'Manual distance': 2 }).name('Focus');
  fd.add(params, 'cineFocusDist', 0.5, 300, 0.1).name('Manual focus (m)');
  fd.add(params, 'cineMB').name('Camera motion blur');
  fd.add(params, 'cineShutter', 0, 360, 1).name('Shutter angle (°)');
  const fg = cineGui.addFolder('Colour grade');
  fg.add(params, 'cineTemp', 2500, 12000, 10).name('White balance (K)').onChange(cineUpdateWB);
  fg.add(params, 'cineTint', -1, 1, 0.01).name('Tint (green–magenta)').onChange(cineUpdateWB);
  fg.add(params, 'cineContrast', 0.5, 1.6, 0.01).name('Contrast');
  fg.add(params, 'cineSat', 0, 2, 0.01).name('Saturation');
  fg.add(params, 'cineLift', -0.1, 0.1, 0.001).name('Lift');
  fg.add(params, 'cineGamma', 0.5, 2, 0.01).name('Gamma');
  fg.add(params, 'cineGain', 0.5, 2, 0.01).name('Gain');
  fl.close(); fd.close(); fg.close(); fe.close();
}
// scripted testing: window.sim.cine exposes the post state, render targets and camera preset state
HOOKS.ready.push(() => { window.sim.cine = { cine, cineRT, cineMats, cineCam, cineSetMode, cineU, composer, compositePass }; });
