// ============================================================================
// Volumetric clouds (feature module "sky")
// ============================================================================
// Raymarched cumulus / stratocumulus / stratus in a spherical shell (default 1.2-3.4 km) lit by the Hillaire
// atmosphere of 03_atmosphere.js: tileable Perlin-Worley shape noise (128^3) and Worley detail noise (32^3) built
// once on the GPU, a 40 km weather map (coverage, cloud type, storm cells), Beer-powder lighting with a dual-lobe
// Henyey-Greenstein phase, three multiple-scattering octaves plus a diffuse-transmission term, a short cone march
// toward the sun (or moon), sky-LUT ambient and aerial perspective toward the horizon.
//
// Per frame (HOOKS.beforeRender):
//   1. march the main view at 1/2..1/4 resolution; ray start and sub-pixel position are jittered every frame;
//   2. temporal resolve: the clouds are kilometres away, so the previous result is reprojected by direction only
//      (exact under orbiting / panning) and ~1/10 of the new frame is blended in;
//   3. one strip of a hemispherical cloud panorama, accumulated over time; it feeds the reflection cube (ENV_PASS),
//      the drone camera and any view that is not the main camera.
// The sky dome samples (2) in screen space, so the clouds sit behind the ocean, the people and everything else.
// The sunlight readback in refreshEnv() (SUNE_FRAG) marches toward the sun, so cloud shadows dim the sea and people.
Object.assign(params, {
  veil: 0.2,                       // the flat high veil reads as lenticular slabs next to volumetric cumulus: thinner
  volClouds: true, cloudBase: 1.2, cloudThick: 2.2, cloudType: 0.8, cloudDensity: 1.0, cloudDetail: 1.0,
  cloudDrift: 1.0, rainShafts: 0.0,
});

// ---- noise generation (tileable; integer-hash Perlin and Worley)
const SKY_NOISEGEN_GLSL = /* glsl */`
uniform float uLayer, uSize, uSeed;
layout(location = 0) out vec4 fragColor;
uvec3 nPcg3(uvec3 v) {
  v = v * 1664525u + 1013904223u;
  v.x += v.y * v.z; v.y += v.z * v.x; v.z += v.x * v.y;
  v ^= v >> 16u;
  v.x += v.y * v.z; v.y += v.z * v.x; v.z += v.x * v.y;
  return v;
}
vec3 nRnd3(ivec3 p) { return vec3(nPcg3(uvec3(p) + uvec3(uint(uSeed) * 7919u))) * (1.0 / 4294967295.0); }
ivec3 nWrap(ivec3 p, int per) { return p - per * ivec3(floor(vec3(p) / float(per))); }
float nGrad(ivec3 c, vec3 f, int per) { return dot(normalize(nRnd3(nWrap(c, per)) * 2.0 - 1.0), f); }
float nPerlin(vec3 p, int per) {
  ivec3 i = ivec3(floor(p));
  vec3 f = p - floor(p);
  vec3 u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
  float n000 = nGrad(i, f, per);
  float n100 = nGrad(i + ivec3(1, 0, 0), f - vec3(1.0, 0.0, 0.0), per);
  float n010 = nGrad(i + ivec3(0, 1, 0), f - vec3(0.0, 1.0, 0.0), per);
  float n110 = nGrad(i + ivec3(1, 1, 0), f - vec3(1.0, 1.0, 0.0), per);
  float n001 = nGrad(i + ivec3(0, 0, 1), f - vec3(0.0, 0.0, 1.0), per);
  float n101 = nGrad(i + ivec3(1, 0, 1), f - vec3(1.0, 0.0, 1.0), per);
  float n011 = nGrad(i + ivec3(0, 1, 1), f - vec3(0.0, 1.0, 1.0), per);
  float n111 = nGrad(i + ivec3(1, 1, 1), f - vec3(1.0, 1.0, 1.0), per);
  return mix(mix(mix(n000, n100, u.x), mix(n010, n110, u.x), u.y), mix(mix(n001, n101, u.x), mix(n011, n111, u.x), u.y), u.z);
}
float nPerlinFbm(vec3 p, int per, int oct) {
  float s = 0.0, a = 1.0, g = exp2(-0.85);
  for (int o = 0; o < 8; o++) {
    if (o >= oct) break;
    s += a * nPerlin(p, per);
    p *= 2.0; per *= 2; a *= g;
  }
  return s;
}
// inverted F1 Worley (1 at the feature points)
float nWorley(vec3 p, int per) {
  ivec3 i = ivec3(floor(p));
  vec3 f = p - floor(p);
  float d = 1e9;
  for (int z = -1; z <= 1; z++) for (int y = -1; y <= 1; y++) for (int x = -1; x <= 1; x++) {
    ivec3 o = ivec3(x, y, z);
    vec3 r = vec3(o) + nRnd3(nWrap(i + o, per)) - f;
    d = min(d, dot(r, r));
  }
  return 1.0 - clamp(sqrt(d), 0.0, 1.0);
}
float nWorleyFbm(vec3 p, int per) {
  return nWorley(p, per) * 0.625 + nWorley(p * 2.0, per * 2) * 0.25 + nWorley(p * 4.0, per * 4) * 0.125;
}
`;
const SKY_SHAPE_FRAG = SKY_NOISEGEN_GLSL + /* glsl */`
void main() {
  vec3 p = vec3(gl_FragCoord.xy, uLayer + 0.5) / uSize;
  float pf = abs(nPerlinFbm(p * 4.0, 4, 7));                // billowy Perlin
  float g = nWorleyFbm(p * 4.0, 4), b = nWorleyFbm(p * 8.0, 8), a = nWorleyFbm(p * 16.0, 16);
  fragColor = vec4(g + pf * (1.0 - g), g, b, a);            // Perlin-Worley, Worley fbm x3
}`;
const SKY_DETAIL_FRAG = SKY_NOISEGEN_GLSL + /* glsl */`
void main() {
  vec3 p = vec3(gl_FragCoord.xy, uLayer + 0.5) / uSize;
  fragColor = vec4(nWorleyFbm(p * 2.0, 2), nWorleyFbm(p * 4.0, 4), nWorleyFbm(p * 8.0, 8), 1.0);
}`;
const SKY_WEATHER_FRAG = SKY_NOISEGEN_GLSL + /* glsl */`
void main() {
  vec3 p = vec3(gl_FragCoord.xy / uSize, 0.31);
  float large = nPerlinFbm(p * 3.0, 3, 5);
  float cells = nWorleyFbm(p * 9.0, 9);
  float fine = nWorleyFbm(p * 28.0, 28);
  float cover = large * 0.55 + cells * 0.75 + fine * 0.25;  // equalised on the CPU afterwards
  float type = nPerlinFbm(p * 2.0 + 5.3, 2, 3);
  float storm = pow(nWorley(p * 4.0 + 1.7, 4), 2.0) * (0.6 + 0.6 * large);
  fragColor = vec4(cover, type, storm, fine);
}`;

function skyBuildNoise3D(size, frag, seed) {
  const rt = new THREE.WebGL3DRenderTarget(size, size, size, { depthBuffer: false });
  const tx = rt.texture;
  tx.format = THREE.RGBAFormat; tx.type = THREE.UnsignedByteType;
  tx.wrapS = tx.wrapT = tx.wrapR = THREE.RepeatWrapping;
  // three r169 cannot build mipmaps for 3D render targets: build them by hand once all layers are written
  tx.magFilter = THREE.LinearFilter; tx.minFilter = THREE.LinearMipmapLinearFilter; tx.generateMipmaps = false;
  const mat = passMat(frag, { uLayer: { value: 0 }, uSize: { value: size }, uSeed: { value: seed } });
  fsQuad.material = mat;
  for (let z = 0; z < size; z++) {
    mat.uniforms.uLayer.value = z;
    renderer.setRenderTarget(rt, z);
    renderer.render(fsScene, fsCam);
  }
  renderer.setRenderTarget(null);
  const gl = renderer.getContext();
  renderer.state.bindTexture(gl.TEXTURE_3D, renderer.properties.get(tx).__webglTexture);
  gl.generateMipmap(gl.TEXTURE_3D);
  renderer.state.unbindTexture();
  mat.dispose();
  return rt;
}
// weather map: generated on the GPU, read back, coverage and type histogram-equalised so the coverage slider
// maps linearly to the fraction of sky that can hold cloud
function skyBuildWeather(size) {
  const rt = target(size, size, { type: THREE.FloatType, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter });
  const mat = passMat(SKY_WEATHER_FRAG, { uLayer: { value: 0 }, uSize: { value: size }, uSeed: { value: 3 } });
  runPass(mat, rt);
  const n = size * size, buf = new Float32Array(n * 4);
  renderer.readRenderTargetPixels(rt, 0, 0, size, size, buf);
  renderer.setRenderTarget(null);
  rt.dispose(); mat.dispose();
  const out = new Uint8Array(n * 4);
  const equalise = ch => {
    const v = new Float32Array(n);
    for (let i = 0; i < n; i++) v[i] = buf[i * 4 + ch];
    const s = Float32Array.from(v).sort();
    for (let i = 0; i < n; i++) {
      let lo = 0, hi = n;
      while (lo < hi) { const m = (lo + hi) >> 1; if (s[m] < v[i]) lo = m + 1; else hi = m; }
      out[i * 4 + ch] = Math.round(255 * lo / (n - 1));
    }
  };
  equalise(0); equalise(1); equalise(3);
  for (let i = 0; i < n; i++) out[i * 4 + 2] = Math.round(255 * clamp(buf[i * 4 + 2], 0, 1));
  const tx = new THREE.DataTexture(out, size, size, THREE.RGBAFormat, THREE.UnsignedByteType);
  tx.wrapS = tx.wrapT = THREE.RepeatWrapping;
  tx.magFilter = THREE.LinearFilter; tx.minFilter = THREE.LinearMipmapLinearFilter; tx.generateMipmaps = true;
  tx.needsUpdate = true;
  return tx;
}
const skyShapeRT = skyBuildNoise3D(128, SKY_SHAPE_FRAG, 1);
const skyDetailRT = skyBuildNoise3D(32, SKY_DETAIL_FRAG, 2);
const skyWeatherTex = skyBuildWeather(512);

// ---- cloud field + lighting (shared by the screen march, the panorama and the sun readback)
const CLOUD_U = {
  uCShape: { value: skyShapeRT.texture }, uCDetail: { value: skyDetailRT.texture }, uCWeather: { value: skyWeatherTex },
  uCLayer: { value: new THREE.Vector4(1.2, 3.4, 0.4, 0.8) },   // base km, top km, coverage, type (0 stratus .. 1 cumulus)
  uCDens: { value: new THREE.Vector4(60, 0.35, 1 / 5.5, 1 / 0.55) }, // extinction 1/km, erosion, shape and detail frequency 1/km
  uCWind: { value: new THREE.Vector4(0, 0, 1, 0) },            // drift offset km (xz), wind direction (xz)
  uCMisc: { value: new THREE.Vector4(0, 0, 1, 1) },            // evolution time s, rain shafts, ambient scale, sun scale
};
Object.assign(skyU, CLOUD_U, {
  uCloudScr: { value: null }, uCloudPano: { value: null }, uCloudVP: { value: new THREE.Matrix4() },
  uCloudCamXZ: { value: new THREE.Vector2(1e9, 1e9) }, uCloudOn: { value: 1 },
  uLtn: { value: new THREE.Vector4(0, 1, 0, 0) },              // lightning flash direction, intensity
  uSeaFog: { value: new THREE.Vector4(0, 150, 1, 0) },         // fog extinction 1/m, scale height m, sun transmittance
});

const CLOUD_GLSL = /* glsl */`
// ---- volumetric clouds (sky_clouds.js)
uniform highp sampler3D uCShape;
uniform highp sampler3D uCDetail;
uniform sampler2D uCWeather;
uniform vec4 uCLayer, uCDens, uCWind, uCMisc;
const float C_WKM = 40.0;                 // weather map tile (km)
const float C_SHAPE_TEX = 128.0;
float cRemap(float v, float l0, float h0, float l1, float h1) { return l1 + clamp((v - l0) / max(h0 - l0, 1e-5), 0.0, 1.0) * (h1 - l1); }
vec4 cWeather(vec2 xz) { return textureLod(uCWeather, (xz + uCWind.xy) / C_WKM, 0.0); }
// continuous local coverage: where it is between ~0.2 and ~0.6 the 3D noise decides the cloud boundary
float cCover(vec4 w) {
  float cov = uCLayer.z;
  return clamp(cov * 1.15 - 0.2 + (w.r - 0.5) * 0.9 + (w.a - 0.5) * 0.15, 0.0, 1.0) * smoothstep(0.0, 0.08, cov);
}
// p = (x km, altitude km, z km); lod = shape-noise mip; hi = with detail erosion
float cDensity(vec3 p, vec4 w, float c, float lod, bool hi) {
  float hf = (p.y - uCLayer.x) / (uCLayer.y - uCLayer.x);
  if (hf <= 0.0 || hf >= 1.0) return 0.0;
  float type = clamp(uCLayer.w + (w.g - 0.5) * 0.5, 0.0, 1.0);
  float hp = smoothstep(0.0, mix(0.32, 0.07, type), hf) * (1.0 - smoothstep(mix(0.4, 0.6, type), 1.0, hf));
  vec3 q = vec3(p.x + uCWind.x, p.y - uCMisc.x * 0.0012, p.z + uCWind.y);
  q.xz += uCWind.zw * hf * 0.35;                       // tops lean downwind
  vec4 s = textureLod(uCShape, q * uCDens.z, lod);
  float wf = s.g * 0.625 + s.b * 0.25 + s.a * 0.125;
  float sh = clamp((cRemap(s.r, wf - 1.0, 1.0, 0.0, 1.0) - 0.42) * 1.9, 0.0, 1.0);
  float ovc = smoothstep(0.75, 1.0, uCLayer.z) * (1.0 - 0.6 * type);
  sh = mix(sh, 0.45 + 0.55 * sh, ovc);                 // overcast: a continuous, still textured deck
  float base = cRemap(sh * hp, 1.0 - c, 1.0, 0.0, 1.0) * c;
  if (!hi || base <= 0.0) return base;
  vec3 d = textureLod(uCDetail, q * uCDens.w, max(lod - 1.0, 0.0)).rgb;
  float df = d.r * 0.625 + d.g * 0.25 + d.b * 0.125;
  float dm = mix(df, 1.0 - df, clamp(hf * 6.0, 0.0, 1.0));   // wispy bases, billowy tops
  return cRemap(base, dm * uCDens.y, 1.0, 0.0, 1.0);
}
// optical depth toward the light (cone of doubling steps, ~2.5 km)
float cLightOD(vec3 p, float ls, float lod) {
  float od = 0.0, t = 0.0, dt = 0.04;
  for (int j = 0; j < 8; j++) {
    if (float(j) >= ls) break;
    vec3 q = p + uSun.xyz * (t + dt * 0.5);
    vec4 w = cWeather(q.xz);
    float c = cCover(w);
    if (c > 0.0) od += cDensity(q, w, c, lod + float(j) * 0.5, j < 1) * dt;
    t += dt; dt *= 2.0;
  }
  return od * uCDens.x;
}
float cPhase(float c, float k) { return mix(phaseHG(c, 0.8 * k), phaseHG(c, -0.25 * k), 0.3); }
float cAvg(float x) { return x > 1e-3 ? (1.0 - exp(-x)) / x : 1.0 - 0.5 * x; }
// transmittance of the clear air between the camera (altitude h0 km) and distance d km
vec3 cAirT(vec3 rd, float d, float h0) {
  float y = max(rd.y, 0.0) + d / (3.0 * RG);
  vec3 od = (RAY_S * exp(-h0 / 8.0) * cAvg(y * d / 8.0) + vec3(MIE_E * uHaze) * exp(-h0 / 1.2) * cAvg(y * d / 1.2)) * d;
  return exp(-od);
}
// ray/shell interval from org (x, altitude, z km); returns false if the ray misses
bool cShell(vec3 ro, vec3 rd, out float t0, out float t1) {
  float rb = RG + uCLayer.x, rt = RG + uCLayer.y, r0 = ro.y;
  bool hit = true;
  t0 = 0.0; t1 = 0.0;
  if (r0 < rb) {
    hit = raySphere(ro, rd, RG).x <= 0.0;            // rays that reach the sea first see no cloud
    t0 = raySphere(ro, rd, rb).y; t1 = raySphere(ro, rd, rt).y;
  } else if (r0 < rt) {
    vec2 b = raySphere(ro, rd, rb);
    t1 = b.x > 0.0 ? b.x : raySphere(ro, rd, rt).y;
  } else {
    vec2 tt = raySphere(ro, rd, rt);
    vec2 b = raySphere(ro, rd, rb);
    hit = tt.x >= 0.0;
    t0 = max(tt.x, 0.0); t1 = b.x > 0.0 ? b.x : tt.y;
  }
  return hit;
}
// premultiplied cloud radiance over the sky (rgb, aerial perspective applied) and cloud transmittance (a)
// st = (primary steps, light steps, max distance km, pixel angle rad)
vec4 cMarch(vec3 org, vec3 rd, float jit, vec4 st) {
  vec4 none = vec4(0.0, 0.0, 0.0, 1.0);
  if (rd.y < -0.05 || uCLayer.z <= 0.001) return none;
  vec3 ro = vec3(0.0, RG + org.y, 0.0);
  float t0, t1;
  if (!cShell(ro, rd, t0, t1) || t0 > st.z) return none;
  t1 = min(t1, min(st.z, t0 + 40.0));
  float dt = (t1 - t0) / st.x;
  float cosT = dot(rd, uSun.xyz);
  float ph0 = cPhase(cosT, 1.0), ph1 = cPhase(cosT, 0.5), ph2 = cPhase(cosT, 0.25);
  float pw = 0.65 * smoothstep(0.6, -0.6, cosT);
  vec3 ambT = skyAtmo(vec3(0.0, 1.0, 0.0)) * 0.75 + skyAtmo(normalize(vec3(-uSun.x, 0.25, -uSun.z) + vec3(1e-4, 0.0, 0.0))) * 0.35;
  vec3 ambB = ambT * 0.35 + skyAtmo(normalize(vec3(rd.x, 0.02, rd.z) + vec3(1e-4, 0.0, 0.0))) * 0.15;
  ambT *= uCMisc.z; ambB *= uCMisc.z;
  vec3 L = vec3(0.0);
  float T = 1.0, tw = 0.0, ws = 0.0;
  float t = t0 + dt * jit;
  float texel = 1.0 / (uCDens.z * C_SHAPE_TEX);
  for (int i = 0; i < 192; i++) {
    if (float(i) >= st.x || T < 0.008 || t > t1) break;
    vec3 pe = ro + rd * t;
    float r = length(pe);
    vec3 p = vec3(pe.x + org.x, r - RG, pe.z + org.z);
    vec4 w = cWeather(p.xz);
    float c = cCover(w);
    if (c <= 0.0) { t += dt * 2.0; continue; }           // no cloud can form here: stride through
    {
      float lod = clamp(log2(max(t * st.w, dt * 0.3) / texel), 0.0, 4.0);
      float d = cDensity(p, w, c, lod, t < 45.0);
      if (d > 0.003) {
        float sig = d * uCDens.x;
        float muS = dot(pe / r, uSun.xyz);
        vec3 sunC = sunBelowHorizon(r, muS) ? vec3(0.0) : sampleTrans(r, muS) * uSun.w * uCMisc.w;
        float od = cLightOD(p, st.y, lod);
        float hf = clamp((p.y - uCLayer.x) / (uCLayer.y - uCLayer.x), 0.0, 1.0);
        float beer = ph0 * exp(-od) + 0.5 * ph1 * exp(-od * 0.35) + 0.25 * ph2 * exp(-od * 0.12);
        float powder = mix(1.0, 1.0 - exp(-od * 1.5 - sig * 0.06), pw);
        vec3 S = sunC * (beer * powder + 0.22 / (1.0 + 0.25 * od)) + mix(ambB, ambT, hf * hf);   // + deep multiple scattering
        float a = 1.0 - exp(-sig * dt);
        L += T * S * a;
        tw += t * T * a; ws += T * a;
        T *= 1.0 - a;
      }
    }
    t += dt;
  }
  T = max(T - 0.01, 0.0) / 0.99;                       // the march stops at T < 0.008: call that opaque (sun disc)
  vec3 sky = skyAtmo(normalize(vec3(rd.x, max(rd.y, 0.0005), rd.z)));
  vec3 Ta = cAirT(rd, ws > 0.0 ? tw / ws : t0, org.y);
  sky *= mix(1.0, uCMisc.z, 0.7);
  vec4 res = vec4(L * Ta + sky * (1.0 - Ta) * (1.0 - T), T);
  // rain shafts under the storm cells, between the camera and the cloud base
  if (uCMisc.y > 0.001 && org.y < uCLayer.x && rd.y < 0.3) {
    float a0 = 2.0, a1 = min(t0, 45.0);
    if (a1 > a0) {
      float dr = (a1 - a0) / 14.0, tr = a0 + dr * jit, Tr = 1.0, trw = 0.0, wr = 0.0;
      vec3 Lr = vec3(0.0);
      vec3 rc = mix(ambB, ambT, 0.3) * 0.8;
      for (int k = 0; k < 14; k++) {
        vec3 pe = ro + rd * tr;
        float r = length(pe);
        vec3 p = vec3(pe.x + org.x, r - RG, pe.z + org.z);
        if (p.y > uCLayer.x) break;
        vec4 w = cWeather(p.xz);
        float pr = smoothstep(0.45, 0.8, w.b) * cCover(w) * uCMisc.y;
        if (pr > 0.0) {
          float sk = textureLod(uCDetail, vec3((p.x + uCWind.x) * 0.9, p.y * 0.05 + uCMisc.x * 0.01, (p.z + uCWind.y) * 0.9), 0.0).r;
          float a = 1.0 - exp(-pr * (0.15 + 0.5 * sk * sk) * dr);
          Lr += Tr * rc * a; trw += tr * Tr * a; wr += Tr * a;
          Tr *= 1.0 - a;
        }
        tr += dr;
      }
      vec3 Tra = cAirT(rd, wr > 0.0 ? trw / wr : a0, org.y);
      res = vec4(Lr * Tra + sky * (1.0 - Tra) * (1.0 - Tr) + Tr * res.rgb, Tr * res.a);
    }
  }
  return res;
}
// transmittance of the clouds along a ray toward the sun (sun readback)
float cSunTrans(vec3 org, vec3 rd) {
  if (uCLayer.z <= 0.001 || rd.y < -0.05) return 1.0;
  vec3 ro = vec3(0.0, RG + org.y, 0.0);
  float t0, t1;
  if (!cShell(ro, rd, t0, t1)) return 1.0;
  t1 = min(t1, t0 + 60.0);
  float dt = (t1 - t0) / 48.0, od = 0.0;
  for (int i = 0; i < 48; i++) {
    vec3 pe = ro + rd * (t0 + (float(i) + 0.5) * dt);
    float r = length(pe);
    vec3 p = vec3(pe.x + org.x, r - RG, pe.z + org.z);
    vec4 w = cWeather(p.xz);
    float c = cCover(w);
    if (c > 0.0) od += cDensity(p, w, c, 0.0, true) * dt;
  }
  return exp(-od * uCDens.x);
}
`;
const SKY_IGN_GLSL = 'float ign(vec2 p) { return fract(52.9829189 * fract(dot(p, vec2(0.06711056, 0.00583715)))); }\n';

// ---- main-view march, temporal resolve, panorama
const SKY_MARCH_FRAG = ATMO_GLSL + NOISE_GLSL + SKY_FUNCS + CLOUD_GLSL + SKY_IGN_GLSL + /* glsl */`
uniform mat4 uInvVP;
uniform vec2 uRes, uJit;
uniform float uFrame;
uniform vec3 uOrg;
uniform vec4 uSteps;
layout(location = 0) out vec4 fragColor;
void main() {
  vec2 ndc = (gl_FragCoord.xy + uJit) / uRes * 2.0 - 1.0;
  vec4 pf = uInvVP * vec4(ndc, 1.0, 1.0);
  vec3 rd = normalize(pf.xyz / pf.w);
  float jit = fract(ign(gl_FragCoord.xy) + uFrame * 0.618034);
  fragColor = cMarch(uOrg, rd, jit, uSteps);
}`;
const SKY_RESOLVE_FRAG = /* glsl */`
uniform sampler2D uCur, uHist;
uniform mat4 uInvVP, uPrevVP;
uniform vec2 uRes;
uniform float uBlend;
layout(location = 0) out vec4 fragColor;
void main() {
  ivec2 ip = ivec2(gl_FragCoord.xy);
  ivec2 mx = ivec2(uRes) - 1;
  vec4 cur = texelFetch(uCur, ip, 0);
  vec4 lo = cur, hi = cur, sum = cur;
  for (int y = -1; y <= 1; y++) for (int x = -1; x <= 1; x++) {
    if (x == 0 && y == 0) continue;
    vec4 c = texelFetch(uCur, clamp(ip + ivec2(x, y), ivec2(0), mx), 0);
    lo = min(lo, c); hi = max(hi, c); sum += c;
  }
  vec2 ndc = gl_FragCoord.xy / uRes * 2.0 - 1.0;
  vec4 pf = uInvVP * vec4(ndc, 1.0, 1.0);
  vec3 rd = normalize(pf.xyz / pf.w);
  vec4 pc = uPrevVP * vec4(rd, 0.0);
  vec2 puv = pc.xy / max(pc.w, 1e-6) * 0.5 + 0.5;
  bool ok = uBlend < 1.0 && pc.w > 0.0 && all(greaterThan(puv, vec2(0.0))) && all(lessThan(puv, vec2(1.0)));
  if (!ok) { fragColor = mix(cur, sum / 9.0, 0.5); return; }
  vec4 h = texture(uHist, puv);
  vec4 ext = (hi - lo) * 0.35 + vec4(vec3(0.02), 0.02);
  h = clamp(h, lo - ext, hi + ext);
  fragColor = mix(h, cur, uBlend);
}`;
// one horizontal strip of the panorama (rows uY0..), blended with the previous panorama in the shader
const SKY_PANO_FRAG = ATMO_GLSL + NOISE_GLSL + SKY_FUNCS + CLOUD_GLSL + SKY_IGN_GLSL + /* glsl */`
uniform sampler2D uHist;
uniform vec2 uRes;
uniform float uFrame, uY0, uBlend;
uniform vec3 uOrg;
uniform vec4 uSteps;
layout(location = 0) out vec4 fragColor;
void main() {
  vec2 px = vec2(gl_FragCoord.x, gl_FragCoord.y + uY0);
  vec2 uv = px / uRes;
  float az = (uv.x - 0.5) * 2.0 * PI;
  float el = uv.y * uv.y * 0.5 * PI;
  vec3 rd = vec3(cos(el) * cos(az), sin(el), cos(el) * sin(az));
  float jit = fract(ign(px) + uFrame * 0.618034);
  vec4 c = cMarch(uOrg, rd, jit, uSteps);
  fragColor = uBlend < 1.0 ? mix(texelFetch(uHist, ivec2(px), 0), c, uBlend) : c;
}`;
const SKY_COPY_FRAG = /* glsl */`
uniform sampler2D uSrc;
uniform float uY0;
layout(location = 0) out vec4 fragColor;
void main() { fragColor = texelFetch(uSrc, ivec2(gl_FragCoord.x, gl_FragCoord.y - uY0), 0); }`;

const skyCloud = {
  w: 0, h: 0, div: 0, q: -1, frame: 0, hist: 0, histOK: false, panoW: 0, panoH: 0, panoStrip: 0, panoFresh: 0,
  march: null, histA: null, histB: null, pano: null,
  vp: new THREE.Matrix4(), prevVP: new THREE.Matrix4(), invVP: new THREE.Matrix4(), rot: new THREE.Matrix4(),
  driftX: 0, driftZ: 0,
};
const skyMarchMat = passMat(SKY_MARCH_FRAG, {
  ...skyU, uInvVP: { value: new THREE.Matrix4() }, uRes: { value: new THREE.Vector2() }, uJit: { value: new THREE.Vector2() },
  uFrame: { value: 0 }, uOrg: { value: new THREE.Vector3() }, uSteps: { value: new THREE.Vector4(64, 6, 100, 0.002) },
});
const skyResolveMat = passMat(SKY_RESOLVE_FRAG, {
  uCur: { value: null }, uHist: { value: null }, uInvVP: { value: new THREE.Matrix4() }, uPrevVP: { value: new THREE.Matrix4() },
  uRes: { value: new THREE.Vector2() }, uBlend: { value: 1 },
});
const skyPanoMat = passMat(SKY_PANO_FRAG, {
  ...skyU, uHist: { value: null }, uRes: { value: new THREE.Vector2() }, uFrame: { value: 0 }, uY0: { value: 0 }, uBlend: { value: 1 },
  uOrg: { value: new THREE.Vector3() }, uSteps: { value: new THREE.Vector4(40, 5, 90, 0.006) },
});
const skyCopyMat = passMat(SKY_COPY_FRAG, { uSrc: { value: null }, uY0: { value: 0 } });

// quality presets: [resolution divisor, primary steps, light steps, max distance km, panorama w, h, panorama steps]
const SKY_Q = [[4, 36, 4, 60, 512, 128, 28], [3, 44, 5, 80, 512, 128, 32], [2, 52, 5, 100, 1024, 256, 40], [2, 80, 6, 120, 1024, 256, 52]];
function skyCloudTargets() {
  const q = clamp(Math.round(params.quality ?? 2), 0, 3), Q = SKY_Q[q];
  const w = Math.max(16, Math.ceil(drawSize.x / Q[0])), h = Math.max(16, Math.ceil(drawSize.y / Q[0]));
  if (q !== skyCloud.q || w !== skyCloud.w || h !== skyCloud.h) {
    const mk = () => target(w, h);
    for (const k of ['march', 'histA', 'histB']) { if (skyCloud[k]) skyCloud[k].dispose(); skyCloud[k] = mk(); }
    skyCloud.march.texture.minFilter = skyCloud.march.texture.magFilter = THREE.NearestFilter;
    Object.assign(skyCloud, { w, h, div: Q[0], histOK: false });
  }
  if (Q[4] !== skyCloud.panoW) {
    if (skyCloud.pano) { skyCloud.pano.dispose(); skyCloud.panoStripRT.dispose(); }
    skyCloud.pano = target(Q[4], Q[5], { wrapS: THREE.RepeatWrapping });
    skyCloud.panoStripRT = target(Q[4], Q[5] / 4, { minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter });
    renderer.setRenderTarget(skyCloud.pano); renderer.setClearColor(0x000000, 1); renderer.clear(); renderer.setRenderTarget(null);
    Object.assign(skyCloud, { panoW: Q[4], panoH: Q[5], panoStrip: 0, panoFresh: 0 });
    skyU.uCloudPano.value = skyCloud.pano.texture;
  }
  skyCloud.q = q;
  return Q;
}
skyCloudTargets();
skyU.uCloudScr.value = skyCloud.histA.texture;
const SKY_HALTON = [[0.5, 0.333], [0.25, 0.667], [0.75, 0.111], [0.125, 0.444], [0.625, 0.778], [0.375, 0.222], [0.875, 0.556], [0.0625, 0.889]];

function skyCloudUniforms(dt) {
  const U = skyU;
  const base = params.cloudBase, top = base + Math.max(params.cloudThick, 0.2);
  U.uCLayer.value.set(base, top, params.volClouds ? params.clouds : 0, params.cloudType);
  U.uCDens.value.x = 60 * params.cloudDensity;
  U.uCDens.value.y = 0.35 * params.cloudDetail;
  // clouds drift with the gradient wind (~1.6x the surface wind + 3 m/s) toward the downwind direction
  const [tx, tz] = bearingVec(params.windFrom + 180);
  const v = (params.windSpeed * 1.6 + 3) * params.cloudDrift * 1e-3;     // km/s
  skyCloud.driftX -= tx * v * dt; skyCloud.driftZ -= tz * v * dt;
  skyCloud.driftX %= 40 * 55; skyCloud.driftZ %= 40 * 55;            // keep the offset small (weather tile x shape tile)
  U.uCWind.value.set(skyCloud.driftX, skyCloud.driftZ, -tx, -tz);
  U.uCMisc.value.x = (U.uCMisc.value.x + dt * params.cloudDrift) % 2e4;
  U.uCMisc.value.y = params.rainShafts;
  // skylight under the cloud deck: open sky in the gaps + diffuse transmission (two-stream ~1/(1 + 0.15 tau)) of the
  // vertical optical depth; scales the ambient of the clouds, the haze in front of them and the sea fog
  const cov = params.volClouds ? clamp(params.clouds, 0, 1) : 0;
  const tau = 60 * params.cloudDensity * Math.max(params.cloudThick, 0.2) * 0.3 * cov;
  U.uCMisc.value.z = (1 - cov) + cov / (1 + 0.15 * tau);
  U.uCloudOn.value = params.volClouds ? 1 : 0;
}
HOOKS.lateUpdate.push(dt => skyCloudUniforms(dt));     // after the weather blend of this frame

function skyRenderClouds() {
  const Q = skyCloudTargets();
  if (!params.volClouds) return;
  const C = skyCloud;
  C.frame++;
  camera.updateMatrixWorld();
  C.rot.copy(camera.matrixWorldInverse).setPosition(0, 0, 0);
  C.vp.multiplyMatrices(camera.projectionMatrix, C.rot);
  // 1. march (jittered)
  const mu = skyMarchMat.uniforms, hj = SKY_HALTON[C.frame % 8];
  mu.uInvVP.value.copy(C.vp).invert();
  mu.uRes.value.set(C.w, C.h);
  mu.uJit.value.set(hj[0] - 0.5, hj[1] - 0.5);
  mu.uFrame.value = C.frame % 997;
  mu.uOrg.value.set(camera.position.x * 1e-3, Math.max(camera.position.y, 0) * 1e-3 + 0.002, camera.position.z * 1e-3);
  const pixAng = 2 * Math.tan(camera.fov * D2R / 2) / C.h;
  mu.uSteps.value.set(Q[1], Q[2], Q[3], pixAng);
  runPass(skyMarchMat, C.march);
  // 2. temporal resolve into the other history buffer
  const src = C.hist ? C.histB : C.histA, dst = C.hist ? C.histA : C.histB;
  const ru = skyResolveMat.uniforms;
  ru.uCur.value = C.march.texture; ru.uHist.value = src.texture;
  ru.uInvVP.value.copy(mu.uInvVP.value);
  ru.uPrevVP.value.copy(C.prevVP);
  ru.uRes.value.set(C.w, C.h);
  ru.uBlend.value = C.histOK ? 0.1 : 1.0;
  runPass(skyResolveMat, dst);
  C.hist ^= 1; C.histOK = true;
  C.prevVP.copy(C.vp);
  skyU.uCloudScr.value = dst.texture;
  skyU.uCloudVP.value.copy(C.vp);
  skyU.uCloudCamXZ.value.set(camera.position.x, camera.position.z);
  // 3. one strip of the panorama per frame (4 strips per refresh); all of it after a reset
  const pu = skyPanoMat.uniforms, strips = 4, sh = C.panoH / strips;
  pu.uRes.value.set(C.panoW, C.panoH);
  pu.uFrame.value = C.frame % 997;
  pu.uOrg.value.set(controls.target.x * 1e-3, 0.002, controls.target.z * 1e-3);
  pu.uSteps.value.set(Q[6], Math.min(Q[2], 5), Q[3], 2 * Math.PI / C.panoW);
  pu.uHist.value = C.pano.texture;
  pu.uBlend.value = C.panoFresh < strips ? 1 : 0.35;
  skyCopyMat.uniforms.uSrc.value = C.panoStripRT.texture;
  for (let k = 0, n = C.panoFresh === 0 ? strips : 1; k < n; k++) {
    const y0 = C.panoStrip * sh;
    pu.uY0.value = y0;
    runPass(skyPanoMat, C.panoStripRT);
    skyCopyMat.uniforms.uY0.value = y0;
    C.pano.scissor.set(0, y0, C.panoW, sh);
    C.pano.scissorTest = true;
    runPass(skyCopyMat, C.pano);
    C.pano.scissorTest = false;
    C.panoStrip = (C.panoStrip + 1) % strips;
    C.panoFresh++;
  }
  renderer.setRenderTarget(null);
}
HOOKS.beforeRender.push(skyRenderClouds);
HOOKS.resize.push(() => { skyCloudTargets(); });
// anything that resets the view abruptly (weather jumps, toggles) can call this
function skyCloudReset() { skyCloud.histOK = false; skyCloud.panoFresh = 0; }

// ---- sky dome, reflection cube and sun readback
const SKY_CLOUD_DECL = /* glsl */`
uniform sampler2D uCloudScr, uCloudPano;
uniform mat4 uCloudVP;
uniform vec2 uCloudCamXZ;
uniform float uCloudOn;
uniform vec4 uLtn;
uniform vec4 uSeaFog;
vec2 cloudPanoUV(vec3 d) {
  vec2 hd = dot(d.xz, d.xz) > 1e-12 ? d.xz : vec2(1.0, 0.0);   // atan(0, 0) is undefined at the zenith
  float u = atan(hd.y, hd.x) * (0.5 / PI) + 0.5;
  float v = sqrt(clamp(asin(clamp(d.y, 0.0, 1.0)) / (0.5 * PI), 0.0, 1.0));
  return vec2(u, v);
}
vec4 volCloud(vec3 d) {
#ifdef ENV_PASS
  return textureLod(uCloudPano, cloudPanoUV(d), 0.0);
#else
  // main camera (and the sensor windows when they follow it): screen-space clouds; anything else: panorama
  vec4 c = uCloudVP * vec4(d, 0.0);
  vec2 suv = c.xy / max(c.w, 1e-6) * 0.5 + 0.5;
  bool scr = c.w > 0.0 && all(greaterThan(suv, vec2(0.0005))) && all(lessThan(suv, vec2(0.9995))) && distance(uCamXZ, uCloudCamXZ) < 0.5;
  return scr ? textureLod(uCloudScr, suv, 0.0) : textureLod(uCloudPano, cloudPanoUV(d), 0.0);
#endif
}
vec3 seaFogColor(vec3 dir) {
  vec3 amb = skyAtmo(vec3(0.0, 1.0, 0.0)) * 0.55 + skyAtmo(normalize(vec3(dir.x, 0.08, dir.z) + vec3(1e-4, 0.0, 0.0))) * 0.55;
  amb = mix(amb, vec3(dot(amb, vec3(0.2126, 0.7152, 0.0722))), 0.55) * uSeaFog.w;   // w: skylight under the clouds
  vec3 sunMid = uSunE / sqrt(max(uSeaFog.z, 1e-3));
  return amb + sunMid * (0.025 + 0.45 * phaseHG(dot(dir, uSun.xyz), 0.7)) * clamp(uSun.y * 4.0 + 0.2, 0.0, 1.0);
}
`;
const SKY_CLOUD_HOOK = /* glsl */`
  float volT = 1.0;
  if (uCloudOn > 0.5) {
    vec4 vc = volCloud(d2);
    volT = vc.a;
    cl = vec4(cl.rgb * vc.a + vc.rgb, 1.0 - (1.0 - cl.a) * vc.a);
    sunT *= vc.a;
  }`;
const SKY_FINAL_HOOK = /* glsl */`
#ifndef ENV_PASS
  if (uLtn.w > 0.0 && dir.y > -0.002) {
    float lk = max(dot(d2, uLtn.xyz), 0.0);
    L += vec3(0.78, 0.83, 1.0) * uLtn.w * (1.0 - volT) * (0.06 + 0.8 * pow(lk, 8.0) + 5.0 * pow(lk, 90.0));
  }
#endif
  if (uSeaFog.x > 0.0) {
    float fy = dir.y;
    float fod = fy > 1e-3 ? uSeaFog.x * uSeaFog.y / fy * (1.0 - exp(-min(3.0e4 * fy / uSeaFog.y, 60.0))) : uSeaFog.x * 3.0e4;
    L = mix(L, seaFogColor(dir), 1.0 - exp(-fod));
  }`;
// core hook markers may carry a trailing note on the same line: end the inserted code with a comment opener
const skyInject = (m, hook, code, stage) => injectShader(m, hook, code + '\n  //', stage);
for (const m of [skyMat, skyEnv.material]) {
  skyInject(m, 'sky-frag-decl', SKY_CLOUD_DECL);
  skyInject(m, 'sky-frag-clouds', SKY_CLOUD_HOOK);
  skyInject(m, 'sky-frag-final', SKY_FINAL_HOOK);
}
skyInject(sunEMat, 'sune-frag-decl', CLOUD_GLSL + 'uniform float uCloudOn;\nuniform vec4 uSeaFog;\n');
skyInject(sunEMat, 'sune-frag-vis', /* glsl */`
  if (uCloudOn > 0.5) {
    vec3 corg = vec3(uCamXZ.x * 1e-3, uCamAlt, uCamXZ.y * 1e-3);
    float cv = 0.0;
    for (int k = 0; k < 3; k++) {
      float ca = float(k) * 2.0944;
      cv += cSunTrans(corg, normalize(sd + (tx * cos(ca) + ty * sin(ca)) * SUN_ANG_R * 3.0));
    }
    vis *= cv / 3.0;
  }
  if (uSeaFog.x > 0.0) vis *= exp(-uSeaFog.x * uSeaFog.y / max(sd.y, 0.03));`);

// ---- thermal: cloud base temperature from the volumetric layer (water clouds are black in LWIR)
HOOKS.lateUpdate.push(() => {
  if (!params.volClouds) return;
  const fogK = skyU.uSeaFog.value.x > 0 ? 1 - Math.exp(-skyU.uSeaFog.value.x * 400) : 0;
  THERM.uCloudFrac.value = clamp(Math.max(params.clouds * 0.95 + params.veil * 0.25, fogK), 0, 1);
  THERM.uTcloud.value = params.airTempC + 273.15 - 6.5 * (fogK > 0.5 ? 0.05 : params.cloudBase);
});

// ---- GUI
const fClouds = gui.addFolder('Volumetric clouds');
const skyCloudChanged = () => { skyDirty = true; skyCloudReset(); };
fClouds.add(params, 'volClouds').name('Volumetric clouds (off = flat layers)').onChange(skyCloudChanged);
fClouds.add(params, 'clouds', 0, 1, 0.01).name('Coverage').onChange(() => { skyDirty = true; }).listen();
fClouds.add(params, 'cloudType', 0, 1, 0.01).name('Type: stratus → cumulus').listen();
fClouds.add(params, 'cloudBase', 0.2, 4, 0.01).name('Cloud base (km)').listen();
fClouds.add(params, 'cloudThick', 0.3, 7, 0.01).name('Thickness (km)').listen();
fClouds.add(params, 'cloudDensity', 0.1, 4, 0.01).name('Density ×').listen();
fClouds.add(params, 'cloudDetail', 0, 2, 0.01).name('Edge erosion ×');
fClouds.add(params, 'cloudDrift', 0, 4, 0.01).name('Drift / evolution ×');
fClouds.add(params, 'rainShafts', 0, 1, 0.01).name('Rain shafts').listen();
fClouds.close();

// after the first updateSky(): fill the panorama and refresh the reflection cube so it shows the clouds at once
HOOKS.ready.push(() => { skyRenderClouds(); refreshEnv(); });
HOOKS.ready.push(() => { window.sim.sky = { envRT, skyCloud, skyShapeRT, skyDetailRT, skyWeatherTex, skyMarchMat, skyPanoMat, skyResolveMat, CLOUD_U }; });
