// ============================================================================
// Weather system (feature module "sky"): presets with smooth transitions, sea fog, rain, lightning
// ============================================================================
// A preset sets clouds (coverage, type, base, thickness, density), haze and aerial perspective, sea-fog visibility,
// wind and waves, exposure, rain and lightning; switching blends every value over params.weatherBlend seconds.
//   - sea fog: an exponential fog layer (scale height fogH) in the sky dome and reflection cube, the ocean's aerial
//     perspective floor (oceanU.uFogExt.w) and the sunlight readback;
//   - rain: camera-centred streak particles (wrapped around whichever camera renders, so the drone view gets its
//     own) and procedural drop rings + splash roughness on the water (ocean-frag-slopes, uniforms only);
//   - lightning: flash sequences that light the clouds (sky dome), the sea (ocean-frag-combine) and the people
//     (a hemisphere light), plus a jagged bolt below the cloud base.
Object.assign(params, { weather: 'Fair cumulus', weatherBlend: 6, rain: 0, lightning: 0, seaFogKm: 0 });

const SKY_PRESETS = {
  'Clear': { clouds: 0.07, cloudType: 0.9, cloudBase: 1.4, cloudThick: 1.5, cloudDensity: 0.9, veil: 0.06, haze: 1.6, fog: 0.8,
    windSpeed: 5, waveHeight: 0.8, exposure: 1.25, rain: 0, rainShafts: 0, lightning: 0, seaFogKm: 0, fogH: 150, humidity: 0.65, light: 1 },
  'Fair cumulus': { clouds: 0.42, cloudType: 0.85, cloudBase: 1.2, cloudThick: 2.2, cloudDensity: 1.0, veil: 0.18, haze: 3.0, fog: 1.0,
    windSpeed: 7, waveHeight: 1.0, exposure: 1.35, rain: 0, rainShafts: 0, lightning: 0, seaFogKm: 0, fogH: 150, humidity: 0.75, light: 1 },
  'Overcast': { clouds: 0.98, cloudType: 0.15, cloudBase: 0.8, cloudThick: 1.5, cloudDensity: 1.3, veil: 0.6, haze: 4.5, fog: 1.6,
    windSpeed: 9, waveHeight: 1.0, exposure: 1.6, rain: 0, rainShafts: 0, lightning: 0, seaFogKm: 0, fogH: 150, humidity: 0.85, light: 1 },
  'Storm': { clouds: 0.94, cloudType: 0.55, cloudBase: 0.55, cloudThick: 4.5, cloudDensity: 2.4, veil: 0.8, haze: 6, fog: 2.5,
    windSpeed: 15, waveHeight: 1.1, exposure: 2.0, rain: 0.9, rainShafts: 0.9, lightning: 1, seaFogKm: 3, fogH: 500, humidity: 0.97, light: 0.8 },
  'Marine fog': { clouds: 0.9, cloudType: 0.0, cloudBase: 0.4, cloudThick: 0.6, cloudDensity: 0.8, veil: 0.3, haze: 7, fog: 2,
    windSpeed: 3, waveHeight: 0.55, exposure: 1.3, rain: 0, rainShafts: 0, lightning: 0, seaFogKm: 0.45, fogH: 140, humidity: 1.0, light: 1 },
  'Rain': { clouds: 0.97, cloudType: 0.3, cloudBase: 0.7, cloudThick: 2.6, cloudDensity: 1.7, veil: 0.7, haze: 5, fog: 2.0,
    windSpeed: 11, waveHeight: 1.0, exposure: 1.8, rain: 0.6, rainShafts: 0.5, lightning: 0, seaFogKm: 6, fogH: 500, humidity: 0.95, light: 0.9 },
};
const skyWx = { from: null, to: null, t: 1, skyT: 0, waveT: 0, sigma: 0, fogH: 150, light: 1 };
const skyFogSigma = km => km > 0 ? 3.912 / (km * 1000) : 0;      // Koschmieder: visibility -> extinction (1/m)
function skyWeatherSnapshot() {
  const s = {};
  for (const k of Object.keys(SKY_PRESETS.Clear)) s[k] = params[k];
  s.sigma = skyWx.sigma; s.fogH = skyWx.fogH; s.light = skyWx.light;
  return s;
}
function skySetWeather(name) {
  const P = SKY_PRESETS[name];
  if (!P) return;
  skyWx.from = skyWeatherSnapshot();
  skyWx.to = { ...P, sigma: skyFogSigma(P.seaFogKm) };
  skyWx.t = params.weatherBlend > 0 ? 0 : 0.999999;
  if (!(params.weatherBlend > 0)) skyCloudReset();     // instant change: drop the cloud history
}
function skyWeatherStep(dtReal) {
  if (skyWx.t >= 1) return;
  skyWx.t = Math.min(1, skyWx.t + (params.weatherBlend > 0 ? dtReal / params.weatherBlend : 1));
  const k = smoothstep(0, 1, skyWx.t), F = skyWx.from, T = skyWx.to;
  for (const key of Object.keys(T)) {
    if (key === 'sigma' || key === 'fogH' || key === 'light' || key === 'seaFogKm') continue;
    if (params.seaLock && (key === 'windSpeed' || key === 'waveHeight')) continue;   // a chosen sea condition keeps the sea
    params[key] = F[key] + (T[key] - F[key]) * k;
  }
  // fog blends in extinction (visibility 0 = no fog)
  skyWx.sigma = F.sigma + (T.sigma - F.sigma) * k;
  skyWx.fogH = F.fogH + (T.fogH - F.fogH) * k;
  skyWx.light = F.light + (T.light - F.light) * k;
  params.seaFogKm = skyWx.sigma > 1e-6 ? 3.912 / (skyWx.sigma * 1000) : 0;
  // the expensive parts (LUTs + reflection cube, wave spectrum) are refreshed a few times per second
  skyWx.skyT -= dtReal; skyWx.waveT -= dtReal;
  const done = skyWx.t >= 1;
  if (skyWx.skyT <= 0 || done) { lutDirty = true; skyDirty = true; skyWx.skyT = 0.2; }
  if (skyWx.waveT <= 0 || done) { needSpectrum = true; skyWx.waveT = 0.5; }
  if (done) gui.controllersRecursive().forEach(c => c.updateDisplay());
}

// ---- sea fog
function skyFogUpdate() {
  if (skyWx.t >= 1) skyWx.sigma = skyFogSigma(params.seaFogKm);
  const sf = skyU.uSeaFog.value, sig = skyWx.sigma;
  sf.set(sig, skyWx.fogH, sig > 0 ? Math.exp(-sig * skyWx.fogH / Math.max(lightDir.y, 0.03)) : 1, skyU.uCMisc.value.z);
  const fe = oceanU.uFogExt.value;
  fe.w = Math.max(1.1e-4 * params.fog, sig);
}

// ---- rain streaks
const SKY_RAIN_MAX = [4000, 8000, 14000, 22000];
const skyRainGeo = new THREE.InstancedBufferGeometry();
skyRainGeo.setAttribute('aCorner', new THREE.Float32BufferAttribute([-1, 0, 1, 0, -1, 1, 1, 1], 2));
skyRainGeo.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(12), 3));
skyRainGeo.setIndex([0, 2, 1, 1, 2, 3]);
{
  const n = SKY_RAIN_MAX[3], a = new Float32Array(n * 4);
  for (let i = 0; i < n * 4; i++) a[i] = Math.random();
  skyRainGeo.setAttribute('aSeed', new THREE.InstancedBufferAttribute(a, 4));
}
skyRainGeo.instanceCount = 0;
const skyRainMat = new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3, transparent: true, depthWrite: false, side: THREE.DoubleSide,   // quads flip with the streak direction
  uniforms: {
    uTime: { value: 0 }, uBox: { value: new THREE.Vector3(30, 16, 30) }, uVel: { value: new THREE.Vector3(0, -8.5, 0) },
    uStreak: { value: 1 / 30 }, uRes: { value: new THREE.Vector2(1, 1) }, uAlpha: { value: 0.3 }, uEnv: { value: envRT.texture },
    uThermal: THERM.uThermal,
  },
  vertexShader: /* glsl */`
in vec2 aCorner;
in vec4 aSeed;
uniform float uTime, uStreak, uAlpha;
uniform vec3 uBox, uVel;
uniform vec2 uRes;
uniform samplerCube uEnv;
out float vA;
out float vSide;
out vec3 vCol;
void main() {
  vec3 vel = uVel * vec3(1.0, 0.8 + 0.4 * aSeed.w, 1.0);
  vec3 p = aSeed.xyz * uBox + vel * uTime;
  vec3 rel = mod(p - cameraPosition + 0.5 * uBox, uBox) - 0.5 * uBox;   // wrapped around the rendering camera
  vec3 head = cameraPosition + rel;
  vec3 tail = head - vel * uStreak;
  mat4 vp = projectionMatrix * viewMatrix;
  vec4 ch = vp * vec4(head, 1.0), ct = vp * vec4(tail, 1.0);
  vec4 c = aCorner.y > 0.5 ? ct : ch;
  vec2 dir = (ch.xy / ch.w - ct.xy / ct.w) * uRes;
  float len = length(dir);
  dir = len > 1e-4 ? dir / len : vec2(0.0, 1.0);
  float wpx = max(1.0, 0.0025 * uRes.y * projectionMatrix[1][1] * 0.5 / max(ch.w, 0.05));
  c.xy += vec2(-dir.y, dir.x) * aCorner.x * wpx / uRes * c.w;
  gl_Position = c;
  vec3 e = abs(rel) / uBox * 2.0;
  float edge = max(max(e.x, e.y), e.z);
  vA = uAlpha * (1.0 - smoothstep(0.7, 1.0, edge)) * smoothstep(0.25, 1.0, length(rel)) * step(0.0, min(ch.w, ct.w)) / wpx;
  vSide = aCorner.x;
  vCol = textureLod(uEnv, vec3(0.0, 1.0, 0.0), 7.0).rgb * 0.65 + textureLod(uEnv, normalize(vec3(rel.x, 0.05, rel.z) + vec3(1e-4, 0.0, 0.0)), 5.0).rgb * 0.55;
}`,
  fragmentShader: /* glsl */`
uniform float uThermal;
in float vA;
in float vSide;
in vec3 vCol;
layout(location = 0) out vec4 fragColor;
void main() {
  if (uThermal > 0.5) discard;           // raindrops sit at air temperature: no LWIR contrast worth drawing
  fragColor = vec4(vCol, vA * (1.0 - vSide * vSide));
}`,
});
const skyRain = new THREE.Mesh(skyRainGeo, skyRainMat);
skyRain.frustumCulled = false; skyRain.renderOrder = 5;
scene.add(skyRain);

// ---- drop rings and splash roughness on the water (procedural: no samplers)
oceanU.uRainAmt = { value: 0 };
oceanU.uLtnSea = { value: 0 };
const skyOceanInject = (hook, code) => injectShader(ocean.material, hook, code + '\n  //');
skyOceanInject('ocean-frag-decl', /* glsl */`
uniform float uRainAmt, uLtnSea;
vec4 skyRainHash(vec2 c, float l) {
  vec4 p = fract(vec4(c.xyxy) * vec4(0.1031, 0.1030, 0.0973, 0.1099) + l * 0.1234);
  p += dot(p, p.wzxy + 33.33);
  return fract((p.xxyz + p.yzzw) * p.zywx);
}`);
skyOceanInject('ocean-frag-slopes', /* glsl */`
  float skyRainSpl = 0.0;                                       // splash crowns, added at ocean-frag-combine
  if (uRainAmt > 0.001) {
    float rDist = length(vRel);
    float rFade = 1.0 - smoothstep(10.0, 35.0, rDist);
    if (rFade > 0.0) {
      vec2 rs = vec2(0.0);
      for (int l = 0; l < 4; l++) {
        float cs = 0.42 + 0.11 * float(l);                      // cell size (m); one drop per cell at a time
        vec2 q = xz / cs + vec2(float(l) * 17.31, float(l) * 7.13);
        vec2 ci = floor(q);
        vec4 h = skyRainHash(ci, float(l));
        if (h.x > uRainAmt * 0.9 + 0.08) continue;
        float period = 0.45 + 0.8 * h.y;
        float cyc = uTime / period + h.z;
        float age = fract(cyc);
        vec4 h2 = skyRainHash(ci + floor(cyc) * 7.0, float(l) + 3.0);   // a new impact point every cycle
        vec2 dp = (q - ci - 0.28 - 0.44 * h2.xy) * cs;
        float r = length(dp);
        float R = age * 0.21 * cs;
        float w = 0.010 + 0.022 * age;
        float A = 0.009 * (1.0 - age) * (1.0 - age);
        float x = (r - R) / w, x2 = (r - R * 0.55) / (w * 0.8);
        float dh = A * (-2.0 * x / w) * exp(-x * x) - 0.45 * A * (-2.0 * x2 / (w * 0.8)) * exp(-x2 * x2);
        rs += dp / max(r, 1e-4) * dh;
        skyRainSpl += smoothstep(0.14, 0.0, age) * exp(-r * r * 2500.0);
      }
      s += rs * rFade;
      skyRainSpl *= rFade;
    }
    cov.x += 0.0025 * uRainAmt; cov.y += 0.0025 * uRainAmt;   // splash roughness everywhere
  }`);
skyOceanInject('ocean-frag-combine', /* glsl */`
  if (uLtnSea > 0.0) col += vec3(0.78, 0.83, 1.0) * uLtnSea * (0.02 + 0.9 * Fr);
  col += skyRainSpl * 0.5 * (skyHemi + sunE * 0.05);`);

// ---- lightning
const skyLtnLight = new THREE.HemisphereLight(0xc8d4ff, 0x203040, 0);
skyLtnLight.layers.enable(PEOPLE_LAYER);
scene.add(skyLtnLight);
const skyBoltMat = new THREE.ShaderMaterial({
  transparent: true, depthWrite: false, blending: THREE.AdditiveBlending,
  uniforms: { uI: { value: 0 }, uThermal: THERM.uThermal },
  vertexShader: 'attribute float aW; varying float vW; void main() { vW = aW; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }',
  fragmentShader: 'uniform float uI, uThermal; varying float vW; void main() { if (uThermal > 0.5) discard; gl_FragColor = vec4(vec3(0.85, 0.9, 1.0) * uI * vW, 1.0); }',
});
const skyBolt = new THREE.LineSegments(new THREE.BufferGeometry(), skyBoltMat);
skyBolt.frustumCulled = false; skyBolt.visible = false;
scene.add(skyBolt);
const skyLtn = { next: 4, t: -1, pulses: [], dir: new THREE.Vector3(0, 1, 0), pos: new THREE.Vector3() };
function skyBuildBolt(top, bottom) {
  const pts = [], w = [];
  const seg = (a, b, depth, wt) => {
    if (depth === 0) { pts.push(a.x, a.y, a.z, b.x, b.y, b.z); w.push(wt, wt); return; }
    const m = a.clone().lerp(b, 0.5), L = a.distanceTo(b);
    m.x += (Math.random() - 0.5) * L * 0.28; m.z += (Math.random() - 0.5) * L * 0.28; m.y += (Math.random() - 0.5) * L * 0.08;
    seg(a, m, depth - 1, wt); seg(m, b, depth - 1, wt);
    if (depth >= 4 && Math.random() < 0.35) {        // side branch
      const e = m.clone().add(new THREE.Vector3(rand(-1, 1) * L * 0.4, -L * rand(0.2, 0.45), rand(-1, 1) * L * 0.4));
      seg(m, e, depth - 2, wt * 0.35);
    }
  };
  seg(top, bottom, 7, 1);
  const g = skyBolt.geometry;
  g.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  g.setAttribute('aW', new THREE.Float32BufferAttribute(w, 1));
  g.computeBoundingSphere();
}
function skyLightningStep(dtReal) {
  const L = skyLtn;
  if ((params.lightning > 0.01 || L.force) && params.volClouds) {
    L.next -= dtReal;
    if (L.next <= 0 || L.force) {
      L.force = false;
      L.next = rand(4, 14) / Math.max(params.lightning, 0.05);
      L.t = 0;
      L.pulses = [];
      let at = 0;
      for (let i = 0, n = 2 + Math.floor(Math.random() * 3); i < n; i++) { L.pulses.push([at, rand(0.4, 1)]); at += rand(0.05, 0.16); }
      // somewhere in front of the camera, 3-12 km away, at the cloud base
      const fwd = new THREE.Vector3(); camera.getWorldDirection(fwd);
      const az = Math.atan2(fwd.z, fwd.x) + rand(-1, 1) * 0.9, d = rand(3000, 12000), base = params.cloudBase * 1000;
      L.pos.set(camera.position.x + Math.cos(az) * d, base, camera.position.z + Math.sin(az) * d);
      L.dir.copy(L.pos).sub(camera.position).normalize();
      L.bolt = Math.random() < 0.6;
      if (L.bolt) skyBuildBolt(new THREE.Vector3(L.pos.x, base + 150, L.pos.z), new THREE.Vector3(L.pos.x + rand(-1, 1) * 600, 0, L.pos.z + rand(-1, 1) * 600));
    }
  }
  let I = 0;
  if (L.t >= 0) {
    L.t += dtReal;
    for (const [t0, a] of L.pulses) { const x = L.t - t0; if (x > 0) I += a * Math.exp(-x * 28) * (1 - Math.exp(-x * 400)); }
    if (L.t > 1.5) L.t = -1;
  }
  const dist = L.pos.distanceTo(camera.position), fogT = Math.exp(-dist * Math.max(skyWx.sigma, 1.1e-4 * params.fog) * 0.5);
  skyU.uLtn.value.set(L.dir.x, L.dir.y, L.dir.z, I * 6 * (0.4 + 0.6 * fogT));
  oceanU.uLtnSea.value = I * 1.2 * (6000 / Math.max(dist, 2000));
  skyLtnLight.intensity = I * 2.5 * (6000 / Math.max(dist, 2000));
  skyBolt.visible = !!L.bolt && I > 0.02;
  skyBoltMat.uniforms.uI.value = I * 400 * fogT;
}

// ---- per frame
HOOKS.update.push((dt, dtReal) => {
  skyWeatherStep(dtReal);
  // rain
  const q = clamp(Math.round(params.quality ?? 2), 0, 3);
  skyRainGeo.instanceCount = Math.round(SKY_RAIN_MAX[q] * clamp(params.rain, 0, 1));
  skyRain.visible = skyRainGeo.instanceCount > 0;
  const ru = skyRainMat.uniforms;
  ru.uTime.value = simTime % 3600;
  ru.uVel.value.set(sea.wind.x * 0.9, -8.5, sea.wind.z * 0.9);
  ru.uAlpha.value = 0.35 + 0.25 * params.rain;
  ru.uRes.value.copy(drawSize);
  oceanU.uRainAmt.value = clamp(params.rain, 0, 1);
  skyLightningStep(dtReal);
});
HOOKS.lateUpdate.push(() => {
  skyFogUpdate();
  // overall light level of the preset (storm light is also cut by the clouds themselves)
  skyU.uCMisc.value.w = skyWx.light;
});

// ---- GUI
const fWx = gui.addFolder('Weather');
fWx.add(params, 'weather', Object.keys(SKY_PRESETS)).name('Preset').onChange(skySetWeather);
fWx.add(params, 'weatherBlend', 0, 30, 0.5).name('Transition (s)');
fWx.add(params, 'rain', 0, 1, 0.01).name('Rain').listen();
fWx.add(params, 'lightning', 0, 2, 0.01).name('Lightning rate').listen();
fWx.add(params, 'seaFogKm', 0, 10, 0.05).name('Sea fog visibility (km, 0 off)').listen();
fWx.add({ flash: () => { skyLtn.force = true; } }, 'flash').name('Trigger lightning');
HOOKS.ready.push(() => { if (window.sim.sky) Object.assign(window.sim.sky, { skyWx, skySetWeather, skyLtn, skyRain, composer }); });
