
// ============================================================================
// Atmosphere: transmittance, multiple scattering and sky-view LUTs
// ============================================================================
const TRANS_FRAG = ATMO_GLSL + /* glsl */`
uniform float uHaze;
layout(location = 0) out vec4 fragColor;
void main() {
  vec2 rm = transUVToRMu(gl_FragCoord.xy / vec2(TRANS_W, TRANS_H));
  vec3 ro = vec3(0.0, rm.x, 0.0);
  vec3 rd = vec3(sqrt(max(0.0, 1.0 - rm.y * rm.y)), rm.y, 0.0);
  float tMax = raySphere(ro, rd, RT).y;
  vec3 od = vec3(0.0);
  float dt = tMax / 64.0;
  for (int i = 0; i < 64; i++) {
    vec3 p = ro + rd * ((float(i) + 0.5) * dt);
    od += medium(length(p) - RG, uHaze).ext * dt;
  }
  fragColor = vec4(exp(-od), 1.0);
}`;

const MS_FRAG = ATMO_GLSL + /* glsl */`
uniform float uHaze;
uniform sampler2D uTrans;
layout(location = 0) out vec4 fragColor;
vec3 sampleTrans(float r, float mu) { return texture(uTrans, transRMuToUV(r, mu)).rgb; }
void main() {
  vec2 uv = gl_FragCoord.xy / MS_SIZE;
  uv = vec2(fromSubUvsToUnit(uv.x, MS_SIZE), fromSubUvsToUnit(uv.y, MS_SIZE));
  float cosSun = uv.x * 2.0 - 1.0;
  float r = RG + max(uv.y * (RT - RG), 0.01);
  vec3 ro = vec3(0.0, r, 0.0);
  vec3 sun = vec3(sqrt(max(0.0, 1.0 - cosSun * cosSun)), cosSun, 0.0);
  vec3 Lsum = vec3(0.0), Fsum = vec3(0.0);
  for (int a = 0; a < 8; a++) {
    for (int b = 0; b < 8; b++) {
      float th = acos(1.0 - 2.0 * (float(b) + 0.5) / 8.0);
      float ph = 2.0 * PI * (float(a) + 0.5) / 8.0;
      vec3 rd = vec3(sin(th) * cos(ph), cos(th), sin(th) * sin(ph));
      vec2 tTop = raySphere(ro, rd, RT);
      vec2 tGr = raySphere(ro, rd, RG);
      float tMax = tTop.y;
      bool hitG = tGr.x > 0.0;
      if (hitG) tMax = tGr.x;
      float dt = tMax / 20.0;
      vec3 T = vec3(1.0), L = vec3(0.0), Fms = vec3(0.0);
      for (int s = 0; s < 20; s++) {
        vec3 p = ro + rd * ((float(s) + 0.5) * dt);
        float pr = length(p);
        Medium m = medium(pr - RG, uHaze);
        vec3 scat = m.rayS + vec3(m.mieS);
        vec3 Ts = exp(-m.ext * dt);
        float smu = dot(p, sun) / pr;
        vec3 sunT = sunBelowHorizon(pr, smu) ? vec3(0.0) : sampleTrans(pr, smu);
        vec3 ext = max(m.ext, vec3(1e-9));
        vec3 S = scat * sunT / (4.0 * PI);
        L += T * (S - S * Ts) / ext;
        Fms += T * (scat - scat * Ts) / ext;
        T *= Ts;
      }
      if (hitG) {
        vec3 ng = normalize(ro + rd * tMax);
        float smu = dot(ng, sun);
        L += T * sampleTrans(RG, smu) * max(smu, 0.0) * 0.06 / PI;
      }
      Lsum += L; Fsum += Fms;
    }
  }
  fragColor = vec4((Lsum / 64.0) / (1.0 - Fsum / 64.0), 1.0);
}`;

const SKYVIEW_FRAG = ATMO_GLSL + /* glsl */`
uniform float uHaze;
uniform float uCamAlt;
uniform float uSunY;
uniform sampler2D uTrans;
uniform sampler2D uMS;
layout(location = 0) out vec4 fragColor;
vec3 sampleTrans(float r, float mu) { return texture(uTrans, transRMuToUV(r, mu)).rgb; }
void main() {
  vec2 uv = gl_FragCoord.xy / vec2(SKY_W, SKY_H);
  uv = vec2(fromSubUvsToUnit(uv.x, SKY_W), fromSubUvsToUnit(uv.y, SKY_H));
  float r = RG + uCamAlt;
  float cosBeta = rhoOf(r) / r;
  float beta = acos(clamp(cosBeta, -1.0, 1.0));
  float zha = PI - beta;
  float vza;
  if (uv.y < 0.5) {
    float coord = 1.0 - 2.0 * uv.y;
    vza = zha * (1.0 - coord * coord);
  } else {
    float coord = uv.y * 2.0 - 1.0;
    vza = zha + beta * coord * coord;
  }
  float lvc = -(uv.x * uv.x * 2.0 - 1.0);
  float lvs = sqrt(max(0.0, 1.0 - lvc * lvc));
  vec3 rd = vec3(sin(vza) * lvc, cos(vza), sin(vza) * lvs);
  vec3 sun = vec3(sqrt(max(0.0, 1.0 - uSunY * uSunY)), uSunY, 0.0);
  vec3 ro = vec3(0.0, r, 0.0);
  vec2 tTop = raySphere(ro, rd, RT);
  vec2 tGr = raySphere(ro, rd, RG);
  float tMax = tTop.y;
  if (tGr.x > 0.0) tMax = tGr.x;
  float cosT = dot(rd, sun);
  float pR = phaseRayleigh(cosT), pM = phaseMie(cosT, 0.84);
  vec3 L = vec3(0.0), T = vec3(1.0);
  float tPrev = 0.0;
  for (int i = 0; i < 48; i++) {
    float fa = (float(i) + 1.0) / 48.0;
    float tn = tMax * fa * fa;
    float dt = tn - tPrev;
    float t = tPrev + dt * 0.3;
    tPrev = tn;
    vec3 p = ro + rd * t;
    float pr = length(p);
    float h = pr - RG;
    Medium m = medium(h, uHaze);
    float pmu = dot(p, sun) / pr;
    vec3 sunT = sunBelowHorizon(pr, pmu) ? vec3(0.0) : sampleTrans(pr, pmu);
    vec2 msuv = vec2(fromUnitToSubUvs(pmu * 0.5 + 0.5, MS_SIZE), fromUnitToSubUvs(clamp(h / (RT - RG), 0.0, 1.0), MS_SIZE));
    vec3 ms = texture(uMS, msuv).rgb;
    vec3 S = sunT * (m.rayS * pR + vec3(m.mieS * pM)) + ms * (m.rayS + vec3(m.mieS));
    vec3 Ts = exp(-m.ext * dt);
    vec3 ext = max(m.ext, vec3(1e-9));
    L += T * (S - S * Ts) / ext;
    T *= Ts;
  }
  fragColor = vec4(L, 1.0);
}`;

const transRT = target(256, 64), msRT = target(32, 32), skyViewRT = target(256, 144);
const transMat = passMat(TRANS_FRAG, { uHaze: { value: 1 } });
const msMat = passMat(MS_FRAG, { uHaze: { value: 1 }, uTrans: { value: transRT.texture } });
const skyViewMat = passMat(SKYVIEW_FRAG, {
  uHaze: { value: 1 }, uCamAlt: { value: 0.002 }, uSunY: { value: 0.1 },
  uTrans: { value: transRT.texture }, uMS: { value: msRT.texture },
});

// ---- sky dome (main view) and environment cube (reflections, image-based lighting)
const skyU = {
  uTrans: { value: transRT.texture }, uSkyView: { value: skyViewRT.texture },
  uSun: { value: new THREE.Vector4(0, 0.1, -1, 14) }, uCamAlt: { value: 0.002 },
  uCloud: { value: new THREE.Vector4(1.5, 0.4, 2.2, 0) }, uVeil: { value: new THREE.Vector2(0.35, 0.9) },
  uCamXZ: { value: new THREE.Vector2() }, uHaze: { value: 3 }, uPixAng: { value: 0.001 },
  uNight: { value: 0 }, uWaterCol: { value: new THREE.Vector3() }, uSunE: { value: new THREE.Vector3() },
  ...THERM,
};
const SKY_VERT = /* glsl */`
out vec3 vDir;
void main() {
  vec4 wp = modelMatrix * vec4(position, 1.0);
  vDir = wp.xyz - cameraPosition;
  gl_Position = projectionMatrix * viewMatrix * wp;
  gl_Position.z = gl_Position.w * 0.999999;
}`;
const SKY_FRAG = ATMO_GLSL + NOISE_GLSL + SKY_FUNCS + THERMAL_UNIFORMS_GLSL + THERMAL_GLSL + /* glsl */`
uniform float uPixAng;
uniform float uNight;
uniform vec3 uWaterCol;
uniform vec3 uSunE;
in vec3 vDir;
// @hook:sky-frag-decl
layout(location = 0) out vec4 fragColor;
vec3 sunDisk(vec3 dir, float pix) {
  float c = dot(dir, uSun.xyz);
  float ang = sqrt(max(2.0 * (1.0 - c), 0.0));
  float edge = 1.0 - smoothstep(SUN_ANG_R - pix, SUN_ANG_R + pix, ang);
  if (edge <= 0.0) return vec3(0.0);
  float x = min(ang / SUN_ANG_R, 1.0);
  float mu = sqrt(max(1.0 - x * x, 0.0));
  vec3 limb = vec3(1.0) - vec3(0.397, 0.503, 0.652) * (1.0 - mu);
  float rad = uSun.w / (PI * SUN_ANG_R * SUN_ANG_R);
  return min(rad * limb * edge * sampleTrans(camR(), max(dir.y, -0.01)), vec3(6.0e4));
}
float stars(vec3 dir) {
  vec3 sd = dir * 260.0;
  vec3 cell = floor(sd);
  vec3 f = fract(sd) - 0.5;
  uint h = pcgHash(uint(int(cell.x) + 4096) * 73856093u ^ pcgHash(uint(int(cell.y) + 4096) * 19349663u ^ uint(int(cell.z) + 4096) * 83492791u));
  float r = float(h & 65535u) / 65535.0;
  float b = float((h >> 16u) & 255u) / 255.0;
  return step(0.9965, r) * smoothstep(0.3, 0.05, length(f)) * (0.3 + 2.5 * b * b);
}
void main() {
  vec3 dir = normalize(vDir);
#ifdef ENV_PASS
  float pixAng = 1.5708 / 256.0 * 1.5;
#else
  float pixAng = uPixAng;
#endif
  vec3 d2 = dir;
  d2.y = max(d2.y, 0.0005);
  d2 = normalize(d2);
  float el = 0.0;
  if (uCloud.y > 0.001) { CloudHit hl = hitLayer(d2, uCloud.x); el = lowEdge(hl.q, hl.t * pixAng / hl.mu, 7); }
  CloudHit hh = hitLayer(d2, 4.5);
  float eh = highEdge(hh.q, hh.t * pixAng / hh.mu, 6);
  vec2 aa = vec2(fwidth(el), fwidth(eh));
  float sunT;
  vec4 cl = cloudLayers(d2, pixAng, aa, sunT);
  // @hook:sky-frag-clouds   (cl = premultiplied cloud radiance + hidden fraction, sunT = sun-disc visibility; may be modified)
#ifndef ENV_PASS
  if (uThermal > 0.5) {
    // LWIR sky: cold clear sky warming towards the horizon, clouds radiate near their base temperature
    float Ls = dir.y >= 0.0 ? skyRadianceLWIR(d2.y, uTair, uRH, cl.a, uTcloud) : planckBand(uTair - 1.0);
    fragColor = vec4(Ls, 0.0, 0.0, 1.0);
    return;
  }
#endif
  vec3 L = skyAtmo(d2) * (1.0 - cl.a) + cl.rgb;
#ifndef ENV_PASS
  L += sunDisk(dir, pixAng * 0.75) * sunT;
  L += vec3(0.85, 0.9, 1.0) * stars(dir) * uNight * (1.0 - cl.a) * smoothstep(0.0, 0.08, dir.y);
#endif
  if (dir.y < 0.0) {
    vec3 hz = skyAtmo(normalize(vec3(dir.x, 0.004, dir.z)));
#ifdef ENV_PASS
    // below the horizon in the reflection cube: what a reflected ray would see -> far, hazy ocean
    vec3 body = uWaterCol * (uSunE * max(uSun.y, 0.0) + skyAtmo(vec3(0.0, 1.0, 0.0)) * 2.0);
    L = mix(hz * 0.75, body, clamp((-dir.y - 0.05) * 6.0, 0.0, 1.0));
#else
    // beyond the ocean mesh edge: fully fogged sea, matching the ocean's aerial perspective
    L = hz;
#endif
  }
  // @hook:sky-frag-final   (L = sky radiance of this pixel; ENV_PASS defined in the reflection cube)
  fragColor = vec4(L, 1.0);
}`;
const skyGeo = new THREE.SphereGeometry(1, 64, 32);
const skyMat = new THREE.ShaderMaterial({ glslVersion: THREE.GLSL3, uniforms: skyU, vertexShader: SKY_VERT, fragmentShader: SKY_FRAG, side: THREE.BackSide, depthWrite: false, depthTest: false });
const skyDome = new THREE.Mesh(skyGeo, skyMat);
skyDome.scale.setScalar(30000); skyDome.frustumCulled = false; skyDome.renderOrder = -10;
scene.add(skyDome);
const envScene = new THREE.Scene();
const skyEnv = new THREE.Mesh(skyGeo, new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3, uniforms: skyU, vertexShader: SKY_VERT, fragmentShader: SKY_FRAG,
  side: THREE.BackSide, depthWrite: false, depthTest: false, defines: { ENV_PASS: 1 },
}));
skyEnv.scale.setScalar(10000); skyEnv.frustumCulled = false;
envScene.add(skyEnv);
const envRT = new THREE.WebGLCubeRenderTarget(256, { type: THREE.HalfFloatType, generateMipmaps: true, minFilter: THREE.LinearMipmapLinearFilter });
const cubeCam = new THREE.CubeCamera(1, 20000, envRT);
const pmrem = new THREE.PMREMGenerator(renderer);
let pmremRT = null;

// Sunlight reaching the sea (atmospheric transmittance x cloud cover over the solar disc), read back for the lights
const SUNE_FRAG = ATMO_GLSL + NOISE_GLSL + SKY_FUNCS + /* glsl */`
layout(location = 0) out vec4 fragColor;
// @hook:sune-frag-decl
void main() {
  vec3 sd = uSun.xyz;
  vec3 tx = normalize(cross(sd, vec3(0.0, 1.0, 0.0)) + vec3(1e-5, 0.0, 0.0));
  vec3 ty = cross(tx, sd);
  float acc = 0.0;
  for (int k = 0; k < 7; k++) {
    float a = float(k) * 0.8975979;
    vec2 o = k == 0 ? vec2(0.0) : vec2(cos(a), sin(a)) * 0.65;
    vec3 dir = normalize(sd + (tx * o.x + ty * o.y) * SUN_ANG_R);
    float t;
    cloudLayers(dir, 0.0, vec2(-1.0), t);
    acc += t;
  }
  float vis = acc / 7.0;
  // @hook:sune-frag-vis   (vis = sun visibility through the clouds at the orbit target; may be modified)
  vec3 E = sunBelowHorizon(camR(), sd.y) ? vec3(0.0) : sampleTrans(camR(), sd.y) * uSun.w * vis;
  fragColor = vec4(E, vis);
}`;
const sunERT = target(1, 1, { type: THREE.FloatType, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter });
const sunEMat = passMat(SUNE_FRAG, skyU);
const sunEBuf = new Float32Array(4);

