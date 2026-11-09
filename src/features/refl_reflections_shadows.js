// ============================================================================
// Reflections and shadows (feature module, prefix refl_)
//  1. screen-space reflections of the people on the water (traced against the people depth buffer refrRT)
//  2. sun shadow maps for the people (PCF soft, light frustum fitted to the visible people every frame)
//  3. analytic body shadows and contact darkening on the water (sphere + capsule proxies from the bones;
//     no extra samplers), plus warm-body reflections in the LWIR view
//  4. wet skin: thin water film (clear coat) on skin, hair and jacket via HOOKS.personShader
// ============================================================================
Object.assign(params, {
  reflOn: true, reflSSR: 1.0, reflShadows: true, reflWaterShadow: 1.0, reflContactAO: 0.6, reflThermal: true, reflWetSkin: 0.7,
});
const REFL_MAXP = MAX_PEOPLE;
// per person: [head centre, r] [upper chest centre, r] [lower chest centre, r] [waterline plane n, w]
Object.assign(oceanU, {
  uReflProj: { value: new THREE.Matrix4() },
  uReflP: { value: Array.from({ length: REFL_MAXP * 4 }, () => new THREE.Vector4()) },
  uReflCfg: { value: new THREE.Vector4() },    // people count, SSR strength, march steps, refine steps
  uReflCfg2: { value: new THREE.Vector4() },   // water shadow, contact AO, IR reflections, proxy fallback
  uReflCfg3: { value: new THREE.Vector4() },   // blur taps, -, SSR valid for this view, pixel angle (rad)
});

// ---------------------------------------------------------------------------- GLSL helpers
// the trailing '//' turns the rest of the marker line (its "(variables ...)" note) back into a comment
const reflInject = (hook, code) => injectShader(ocean.material, hook, code + '\n//');
reflInject('ocean-frag-decl', /* glsl */`
#define REFL_MAXP ${REFL_MAXP}
uniform mat4 uReflProj;
uniform vec4 uReflP[REFL_MAXP * 4];
uniform vec4 uReflCfg, uReflCfg2, uReflCfg3;
// closest approach of the ray ro + s rd (s >= 0) to a sphere: (clearance to the surface, s)
vec2 reflSph(vec3 ro, vec3 rd, vec4 sph) {
  float s = max(dot(sph.xyz - ro, rd), 0.0);
  return vec2(length(ro + rd * s - sph.xyz) - sph.w, s);
}
// ... to a capsule a-b whose radius goes linearly from ra to rb (Ericson, ray vs segment)
vec2 reflCap(vec3 ro, vec3 rd, vec4 a, vec4 b) {
  vec3 d2 = b.xyz - a.xyz, r = ro - a.xyz;
  float e = dot(d2, d2), f = dot(d2, r), c = dot(rd, r), bb = dot(rd, d2);
  float den = max(e - bb * bb, 1e-6);
  float s = max((bb * f - c * e) / den, 0.0);
  float t = (bb * s + f) / e;
  if (t < 0.0) { t = 0.0; s = max(-c, 0.0); }
  else if (t > 1.0) { t = 1.0; s = max(bb - c, 0.0); }
  return vec2(length(ro + rd * s - a.xyz - d2 * t) - mix(a.w, b.w, t), s);
}
float reflSceneZ(vec2 uv) {
  float d = textureLod(uRefrDepth, uv, 0.0).r;
  return uNear * uFar / (uFar - d * (uFar - uNear));
}
vec2 reflProject(vec3 pV) { vec4 c = uReflProj * vec4(pV, 1.0); return c.xy / c.w * 0.5 + 0.5; }
// LWIR sky radiance along the reflected ray, with the warm heads and lifejackets the ray runs into
float reflSkyLWIR(float e, float Ta, float rh, float cf, float Tc, vec3 ro, vec3 rd) {
  float Ls = skyRadianceLWIR(e, Ta, rh, cf, Tc);
  if (uReflCfg2.z < 0.5 || rd.y <= 0.0) return Ls;
  float fp = max(length(ro - cameraPosition) * uReflCfg3.w, 0.003);
  float L = Ls;
  for (int i = 0; i < REFL_MAXP; i++) {
    if (float(i) >= uReflCfg.x) break;
    vec4 hd = uReflP[i * 4], nk = uReflP[i * 4 + 1], bl = uReflP[i * 4 + 2], pl = uReflP[i * 4 + 3];
    vec2 q = reflSph(ro, rd, vec4(nk.xyz, 0.9));
    if (q.x > q.y * 0.05 + fp) continue;
    vec2 h = reflSph(ro, rd, hd), c = reflCap(ro, rd, nk, bl);
    bool headFirst = h.y > c.y;                 // composite back to front: the nearer part covers the farther one
    for (int k = 0; k < 2; k++) {
      bool isHead = (k == 0) == headFirst;
      vec2 o = isHead ? h : c;
      float hw = dot(pl.xyz, ro + rd * o.y) + pl.w;                 // height of that point above the waterline
      float cov = (1.0 - smoothstep(-1.0, 1.0, o.x / (o.y * 0.03 + fp))) * smoothstep(-0.02, 0.04, hw);
      if (cov <= 0.0) continue;
      int m = isHead ? LWIR_MAT_FACE : LWIR_MAT_LIFEJACKET;
      float T = personSurfaceTemp(m, max(hw, 0.0), 0.0, uTwater, uTair, uWindSpd, uSunIrr);
      float em = materialEmissivity(m);
      L = mix(L, em * planckBand(T) + (1.0 - em) * Ls, cov);
    }
  }
  return L;
}`);

// ---------------------------------------------------------------------------- shadows + contact AO on the water
// spec (sun glitter) loses the sun sharply; the sun-lit scatter/SSS/foam terms (all built from sunE later) softly
reflInject('ocean-frag-specular', /* glsl */`
  {
    vec3 rPw = cameraPosition + vRel;
    float rFp = max(dist * uReflCfg3.w, 0.002);
    float visS = 1.0, visB = 1.0, aoK = 1.0;
    if (uReflCfg2.x + uReflCfg2.y > 0.0) {
      for (int i = 0; i < REFL_MAXP; i++) {
        if (float(i) >= uReflCfg.x) break;
        vec4 hd = uReflP[i * 4], nk = uReflP[i * 4 + 1], bl = uReflP[i * 4 + 2];
        vec3 dn = nk.xyz - rPw;
        float dd = dot(dn, dn);
        if (uReflCfg2.x > 0.0 && Ld.y > 0.0) {
          vec2 q = reflSph(rPw, Ld, vec4(nk.xyz, 0.75));
          if (q.x < q.y * 0.2 + rFp + 0.1) {
            vec2 h = reflSph(rPw, Ld, hd), c = reflCap(rPw, Ld, nk, bl);
            visS = min(min(visS, smoothstep(-1.0, 1.0, h.x / (h.y * 0.012 + rFp))), smoothstep(-1.0, 1.0, c.x / (c.y * 0.012 + rFp)));
            visB = min(min(visB, smoothstep(-1.0, 1.0, h.x / (h.y * 0.16 + rFp + 0.04))),
                       smoothstep(-1.0, 1.0, c.x / (c.y * 0.16 + rFp + 0.04)));
          }
        }
        if (uReflCfg2.y > 0.0 && dd < 4.0) {
          // sky occlusion of the water next to the body (cosine-weighted sphere occlusion)
          vec3 ba = bl.xyz - nk.xyz;
          float t = clamp(dot(rPw - nk.xyz, ba) / dot(ba, ba), 0.0, 1.0);
          vec3 D = nk.xyz + ba * t - rPw;
          float l2 = max(dot(D, D), 1e-4), rr = mix(nk.w, bl.w, t);
          aoK *= 1.0 - clamp(max(D.y, 0.0) * inversesqrt(l2) * rr * rr / l2, 0.0, 1.0);
          vec3 Dh = hd.xyz - rPw;
          float lh = max(dot(Dh, Dh), 1e-4);
          aoK *= 1.0 - clamp(max(Dh.y, 0.0) * inversesqrt(lh) * hd.w * hd.w / lh, 0.0, 1.0);
        }
      }
    }
    spec *= mix(1.0, visS, uReflCfg2.x);
    sunE *= mix(1.0, visB, uReflCfg2.x * 0.55);
    float ao = mix(1.0, aoK, uReflCfg2.y);
    skyUp *= ao; skyHemi *= ao; skyN *= mix(1.0, ao, 0.5);
  }`);

// ---------------------------------------------------------------------------- screen-space reflections of the people
reflInject('ocean-frag-reflection', /* glsl */`
  if (uReflCfg.x > 0.5 && R.y > 0.0) {
    vec3 rPw = cameraPosition + vRel;
    float rFp = max(dist * uReflCfg3.w, 0.002);
    // nearest person whose reach (1 m around the chest) the reflected ray enters, and how much the body proxies
    // cover the ray: soft (roughness-blurred) and core (ray through the middle of a proxy)
    float tA = 1e9, tB = 0.0, occ = 0.0, occCore = 0.0;
    int pA = -1;
    for (int i = 0; i < REFL_MAXP; i++) {
      if (float(i) >= uReflCfg.x) break;
      vec4 nk = uReflP[i * 4 + 1];
      vec3 oc = rPw - nk.xyz;
      float b = dot(oc, R), h = b * b - dot(oc, oc) + 1.0;
      if (h <= 0.0) continue;
      float sq = sqrt(h);
      if (-b + sq <= 0.0) continue;
      float t0 = max(-b - sq, 0.03);
      if (t0 < tA) { tA = t0; tB = -b + sq; pA = i; }
      vec4 hd = uReflP[i * 4], bl = uReflP[i * 4 + 2], pl = uReflP[i * 4 + 3];
      vec2 qh = reflSph(rPw, R, hd), qc = reflCap(rPw, R, nk, bl);
      float ah = smoothstep(-0.02, 0.05, dot(pl.xyz, rPw + R * qh.y) + pl.w);   // that part is above the waterline
      float ac = smoothstep(-0.02, 0.05, dot(pl.xyz, rPw + R * qc.y) + pl.w);
      float wb = sig;
      occ = max(occ, max((1.0 - smoothstep(-1.0, 1.0, qh.x / (qh.y * wb + rFp))) * ah,
                         (1.0 - smoothstep(-1.0, 1.0, qc.x / (qc.y * wb + rFp))) * ac));
      occCore = max(occCore, max((1.0 - smoothstep(-0.6, -0.3, qh.x / hd.w)) * ah, (1.0 - smoothstep(-0.6, -0.3, qc.x / bl.w)) * ac));
    }
    vec4 ssr = vec4(0.0);
    float ssrW = 0.0, unknown = 1.0;
    if (pA >= 0 && uReflCfg.y > 0.0 && uReflCfg3.z > 0.5) {
      vec3 roV = (viewMatrix * vec4(rPw, 1.0)).xyz;
      vec3 rdV = mat3(viewMatrix) * R;
      float tMax = tB;
      if (rdV.z > 1e-4) tMax = min(tMax, (-roV.z - 2.0 * uNear) / rdV.z);   // stay in front of the camera
      if (tMax > tA) {
        float nS = uReflCfg.z;
        float dt = (tMax - tA) / nS;
        float thick = 0.3 + 1.5 * dt * abs(rdV.z);
        float tLo = tA, tHit = -1.0;
        unknown = 0.0;
        for (int k = 0; k <= 40; k++) {
          if (float(k) > nS) break;
          float t = tA + dt * float(k);
          vec3 pV = roV + rdV * t;
          vec2 uv = reflProject(pV);
          if (uv.x < 0.0 || uv.y < 0.0 || uv.x > 1.0 || uv.y > 1.0) { unknown = 1.0; break; }   // left the screen
          float dz = -pV.z - reflSceneZ(uv);
          if (dz > 0.0 && dz < thick) { tHit = t; break; }
          tLo = t;
        }
        if (tHit > 0.0) {
          float tHi = tHit;
          for (int k = 0; k < 8; k++) {
            if (float(k) >= uReflCfg.w) break;
            float tm = 0.5 * (tLo + tHi);
            vec3 pV = roV + rdV * tm;
            if (-pV.z - reflSceneZ(reflProject(pV)) > 0.0) tHi = tm; else tLo = tm;
          }
          vec3 pV = roV + rdV * tHi;
          vec2 uv = reflProject(pV);
          float sz = reflSceneZ(uv);
          // the surface stored in the depth buffer (not the ray point) must lie above that person's waterline
          vec3 sW = cameraPosition + transpose(mat3(viewMatrix)) * (pV * (sz / max(-pV.z, 1e-4)));
          vec4 pl = uReflP[pA * 4 + 3];
          vec2 e2 = min(uv, 1.0 - uv);
          float edge = smoothstep(0.0, 0.05, min(e2.x, e2.y));
          ssrW = edge * smoothstep(-0.01, 0.06, dot(pl.xyz, sW) + pl.w)
               * (1.0 - smoothstep(150.0, 500.0, dist)) * (1.0 - smoothstep(6.0, 14.0, tHi))
               * (1.0 - smoothstep(0.3, 0.8, sig)) * uReflCfg.y;
          unknown = 1.0 - edge;
          // roughness blur: the reflected rays spread by about 2 sig over the ray length
          float br = clamp(tHi * sig * 0.5 * uReflProj[1][1] / max(sz, 0.1), 0.0, 0.04);
          vec2 ax = vec2(br * uResolution.y / uResolution.x, br);
          ssr = textureLod(uRefr, uv, 0.0);
          if (uReflCfg3.x > 1.5) {
            ssr += textureLod(uRefr, uv + vec2( 0.6,  0.8) * ax, 0.0) + textureLod(uRefr, uv + vec2(-0.8,  0.6) * ax, 0.0)
                 + textureLod(uRefr, uv + vec2(-0.6, -0.8) * ax, 0.0) + textureLod(uRefr, uv + vec2( 0.8, -0.6) * ax, 0.0);
            if (uReflCfg3.x > 5.5) {
              ssr += textureLod(uRefr, uv + vec2( 0.0,  0.45) * ax, 0.0) + textureLod(uRefr, uv + vec2(-0.45, 0.0) * ax, 0.0)
                   + textureLod(uRefr, uv + vec2( 0.0, -0.45) * ax, 0.0) + textureLod(uRefr, uv + vec2( 0.45, 0.0) * ax, 0.0);
              ssr *= 1.0 / 9.0;
            } else ssr *= 0.2;
          }
        }
      }
    }
    float covS = clamp(ssr.a * ssrW, 0.0, 1.0);
    refl = refl * (1.0 - covS) + ssr.rgb * ssrW;
    // what the trace could not resolve (off screen, no trace in this view) or could not see (the hidden side,
    // ray through the middle of a body without a depth hit): a dim, shaded body instead of bright sky
    float fb = mix(occCore, occ, unknown) * (1.0 - covS) * uReflCfg2.w;
    refl = mix(refl, skyHemi * 0.12 + sunE * max(Ld.y, 0.0) * 0.01, fb);
  }`);

// LWIR branch: the sky radiance it reflects also contains the warm people (macro scoped to the thermal block)
reflInject('ocean-frag-foam', '#define skyRadianceLWIR(e_, a_, b_, c_, d_) reflSkyLWIR(e_, a_, b_, c_, d_, cameraPosition + vRel, reflect(-V, nrm))');
reflInject('ocean-frag-combine', '#undef skyRadianceLWIR');

// the projection, SSR validity and pixel footprint follow whichever camera draws the ocean (main view or sensors)
{
  const prevOBR = ocean.onBeforeRender;
  ocean.onBeforeRender = function (r, s, cam, g, m, grp) {
    prevOBR.call(this, r, s, cam, g, m, grp);
    oceanU.uReflProj.value.copy(cam.projectionMatrix);
    const tex = oceanU.uRefr.value, res = oceanU.uResolution.value;
    const ok = THERM.uThermal.value < 0.5 && tex && tex.image && tex.image.width === res.x && tex.image.height === res.y;
    oceanU.uReflCfg3.value.z = ok ? 1 : 0;
    oceanU.uReflCfg3.value.w = 2 * Math.tan((cam.fov || 50) * D2R / 2) / res.y;
  };
}

// ---------------------------------------------------------------------------- per-person proxies from the bones
// head sphere + one tapered capsule for chest, collar and the jacket lobes (which sit ~0.12-0.18 m in front of
// the chest bone, from 0.27 m below to 0.17 m above it), in the inclined body frame
const _rA = new THREE.Vector3(), _rB = new THREE.Vector3(), _rC = new THREE.Vector3(), _rD = new THREE.Vector3(),
  _rUp = new THREE.Vector3(), _rL = new THREE.Vector3(), _rF = new THREE.Vector3();
function reflProxies(P, out, o) {
  if (!P.reflB) {
    const b = n => P.rig.byName(n);
    P.reflB = { head: b('Head'), top: b('HeadTop_End') || b('Head'), neck: b('Neck'), chest: b('Spine2') || b('Spine1'),
      belly: b('Spine1') || b('Spine'), la: b('LeftArm'), ra: b('RightArm') };
  }
  const B = P.reflB;
  B.neck.getWorldPosition(_rA); B.belly.getWorldPosition(_rB);
  _rUp.subVectors(_rA, _rB).normalize();
  B.la.getWorldPosition(_rL); B.ra.getWorldPosition(_rC);
  _rL.sub(_rC).normalize();
  _rF.crossVectors(_rL, _rUp).normalize();                                  // chest-forward
  B.head.getWorldPosition(_rC); B.top.getWorldPosition(_rD);
  _rC.lerp(_rD, 0.45).addScaledVector(_rF, 0.01);
  out[o].set(_rC.x, _rC.y, _rC.z, 0.1);                                    // head
  B.chest.getWorldPosition(_rA);
  _rC.copy(_rA).addScaledVector(_rF, 0.04).addScaledVector(_rUp, 0.09);
  out[o + 1].set(_rC.x, _rC.y, _rC.z, 0.15);                               // upper chest, collar, top of the lobes
  _rC.copy(_rA).addScaledVector(_rF, 0.04).addScaledVector(_rUp, -0.2);
  out[o + 2].set(_rC.x, _rC.y, _rC.z, 0.17);                               // lower chest + lobes
  out[o + 3].copy(P.wetU.plane.value);                                     // waterline plane
}

// ---------------------------------------------------------------------------- sun shadow maps
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.shadowMap.autoUpdate = false;          // drawn once per frame (see beforeRender below), not once per view
const reflSh = { w: 0, h: 0, size: 0 };
const _lx = new THREE.Vector3(), _ly = new THREE.Vector3(), _lz = new THREE.Vector3(), _pm = new THREE.Matrix4(),
  _frA = new THREE.Frustum(), _frB = new THREE.Frustum(), _sph = new THREE.Sphere();
function reflFitShadow() {
  const q = params.quality ?? 2;
  const size = [1024, 2048, 2048, 4096][q] || 2048;
  const sh = sunLight.shadow;
  if (size !== reflSh.size) {
    reflSh.size = size;
    sh.mapSize.set(size, size);
    if (sh.map) { sh.map.dispose(); sh.map = null; }
  }
  const L = _lz.copy(lightDir).normalize();
  _lx.set(0, 1, 0).cross(L);
  if (_lx.lengthSq() < 1e-6) _lx.set(1, 0, 0);
  _lx.normalize();
  _ly.crossVectors(L, _lx);
  // casters: people that are (or whose shadow could be) inside the main or the sensor view
  camera.updateMatrixWorld();
  _frA.setFromProjectionMatrix(_pm.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse));
  if (params.sensors) _frB.setFromProjectionMatrix(_pm.multiplyMatrices(sensorCam.projectionMatrix, sensorCam.matrixWorldInverse));
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity, z0 = Infinity, z1 = -Infinity;
  const R = 1.25;
  for (const P of people) {
    _sph.center.copy(P.pos); _sph.radius = R + 2.5;
    if (!_frA.intersectsSphere(_sph) && !(params.sensors && _frB.intersectsSphere(_sph))) continue;
    const x = _lx.dot(P.pos), y = _ly.dot(P.pos), z = L.dot(P.pos);
    x0 = Math.min(x0, x - R); x1 = Math.max(x1, x + R);
    y0 = Math.min(y0, y - R); y1 = Math.max(y1, y + R);
    z0 = Math.min(z0, z - R); z1 = Math.max(z1, z + R);
  }
  if (x0 === Infinity) return false;
  // stable extents (hysteresis, 0.5 m steps) and texel-snapped centre
  const fit = (need, cur) => (need > cur || need < cur * 0.55) ? Math.ceil(need * 1.15 / 0.5) * 0.5 : cur;
  reflSh.w = fit(x1 - x0, reflSh.w); reflSh.h = fit(y1 - y0, reflSh.h);
  const tx = reflSh.w / size, ty = reflSh.h / size;
  const cx = Math.round((x0 + x1) / 2 / tx) * tx, cy = Math.round((y0 + y1) / 2 / ty) * ty, cz = z1 + 1;
  sunLight.position.set(0, 0, 0).addScaledVector(_lx, cx).addScaledVector(_ly, cy).addScaledVector(L, cz);
  sunLight.target.position.copy(sunLight.position).sub(L);
  const cam = sh.camera;
  cam.left = -reflSh.w / 2; cam.right = reflSh.w / 2; cam.top = reflSh.h / 2; cam.bottom = -reflSh.h / 2;
  cam.near = 0.05; cam.far = Math.ceil((cz - z0 + 1) / 4) * 4;
  cam.updateProjectionMatrix();
  // depth bias ~ half a texel along the light, normal offset ~ one texel
  const tex = Math.max(tx, ty);
  sh.bias = -0.6 * tex / cam.far;
  sh.normalBias = 1.0 * tex;
  return true;
}

// ---------------------------------------------------------------------------- wet skin (thin water film)
// a clear coat (F0 0.02, smooth) over skin, hair, straps and tape, weighted by the core wetness term
const reflWetU = { value: params.reflWetSkin };
HOOKS.personShader.push((sh, mat) => {
  sh.uniforms.uReflWet = reflWetU;
  const coat = !mat.isMeshPhysicalMaterial || !(mat.clearcoat > 0);
  if (coat) { sh.uniforms.clearcoat = { value: 0 }; sh.uniforms.clearcoatRoughness = { value: 0 }; }
  sh.fragmentShader = (coat ? '#ifndef USE_CLEARCOAT\n#define USE_CLEARCOAT\n#define REFL_WET_COAT\n#endif\n' : '') +
    'uniform float uReflWet;\n' + sh.fragmentShader.replace('#include <lights_physical_fragment>', `#include <lights_physical_fragment>
#ifdef REFL_WET_COAT
  material.clearcoat = uReflWet * wetness;
  material.clearcoatRoughness = min(0.1 + 0.15 * (1.0 - wetness) + geometryRoughness, 1.0);
  material.clearcoatF0 = vec3(0.02);
#endif`);
});

// ---------------------------------------------------------------------------- per-frame state
function reflQuality() {
  const q = params.quality ?? 2;
  return { steps: [10, 14, 20, 28][q] ?? 20, refine: [3, 4, 5, 6][q] ?? 5, taps: [1, 5, 5, 9][q] ?? 5 };
}
HOOKS.update.push(() => {
  const on = params.reflOn;
  for (const P of people) {
    if (P.reflInit) continue;
    P.root.traverse(o => { if (o.isMesh && o.parent !== P.probeGroup) { o.castShadow = true; o.receiveShadow = true; } });
    P.reflInit = true;
  }
  const arr = oceanU.uReflP.value;
  const n = Math.min(people.length, REFL_MAXP);
  for (let i = 0; i < n; i++) reflProxies(people[i], arr, i * 4);
  const Q = reflQuality();
  oceanU.uReflCfg.value.set(on ? n : 0, params.reflSSR, Q.steps, Q.refine);
  oceanU.uReflCfg2.value.set(params.reflWaterShadow, params.reflContactAO, params.reflThermal ? 1 : 0, 0.55);
  oceanU.uReflCfg3.value.x = Q.taps;
  reflWetU.value = on ? params.reflWetSkin : 0;
});
HOOKS.lateUpdate.push(() => {
  sunLight.castShadow = params.reflOn && params.reflShadows && lightDir.y > 0.01;
  if (sunLight.castShadow && people.length) reflFitShadow();     // keeps the last frustum when nobody is in view
});
// shadow maps are drawn by the first render of the scene each frame (the people pass) and reused by all later views
HOOKS.beforeRender.push(() => { renderer.shadowMap.needsUpdate = true; });

// ---------------------------------------------------------------------------- GUI
{
  const f = gui.addFolder('Reflections & shadows');
  f.add(params, 'reflOn').name('Enabled');
  f.add(params, 'reflSSR', 0, 1.5, 0.01).name('People reflections');
  f.add(params, 'reflShadows').name('Sun shadow maps');
  f.add(params, 'reflWaterShadow', 0, 1, 0.01).name('Body shadows on water');
  f.add(params, 'reflContactAO', 0, 1, 0.01).name('Contact darkening');
  f.add(params, 'reflThermal').name('IR body reflections');
  f.add(params, 'reflWetSkin', 0, 1, 0.01).name('Wet skin film');
  f.close();
}
HOOKS.ready.push(() => { window.sim.refl = { fit: reflFitShadow, sh: reflSh, sunLight }; });
