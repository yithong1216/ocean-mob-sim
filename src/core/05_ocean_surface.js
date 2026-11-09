// ============================================================================
// Ocean surface
// ============================================================================
function makeOceanGeometry(rings = 380, segs = 512, rMin = 0.05, rMax = 20000) {
  const pos = [0, 0, 0], idx = [];
  for (let i = 0; i < rings; i++) {
    const r = rMin * Math.pow(rMax / rMin, i / (rings - 1));
    for (let j = 0; j < segs; j++) { const a = j / segs * Math.PI * 2; pos.push(r * Math.cos(a), 0, r * Math.sin(a)); }
  }
  const v = (i, j) => 1 + i * segs + (j % segs);
  for (let j = 0; j < segs; j++) idx.push(0, v(0, j + 1), v(0, j));
  for (let i = 0; i < rings - 1; i++) for (let j = 0; j < segs; j++) {
    const a = v(i, j), b = v(i, j + 1), c = v(i + 1, j), d = v(i + 1, j + 1);
    idx.push(a, b, c, b, d, c);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  return { geo: g, lodK: Math.pow(rMax / rMin, 1 / (rings - 1)) - 1 };
}
const OCEAN_VERT = CASC_SAMPLERS(['uDisp']) + /* glsl */`
#define NC ${NC}
uniform float uCascL[NC];
uniform vec2 uCenter;
uniform float uLodK, uLodMin;
out vec3 vRel;
out vec2 vXZ;
out float vHeight;
out float vViewZ;
// @hook:ocean-vert-decl
void main() {
  vec2 local = position.xz;
  vec2 xz = uCenter + local;
  float spacing = max(uLodMin, length(local) * uLodK);
  vec3 disp = vec3(0.0);
  ${forC(c => `{ float lod = log2(max(spacing / (uCascL[${c}] / ${N}.0), 1.0)) + 0.5;
    if (lod < 8.0) disp += textureLod(uDisp${c}, xz / uCascL[${c}], lod).xyz; }`)}
  // @hook:ocean-vert-displace   (xz = lagrangian position, disp = summed FFT displacement; may be modified)
  vec3 world = vec3(xz.x + disp.x, disp.y, xz.y + disp.z);
  vRel = world - cameraPosition;
  vXZ = xz;
  vHeight = disp.y;
  vec4 mv = viewMatrix * vec4(world, 1.0);
  vViewZ = -mv.z;
  gl_Position = projectionMatrix * mv;
}`;
const OCEAN_FRAG = CASC_SAMPLERS(['uDeriv', 'uMom']) + /* glsl */`
#define NC ${NC}
uniform float uCascL[NC];
uniform samplerCube uEnv;
uniform sampler2D uFoamTex, uFoamWorld, uRefr, uRefrDepth;
uniform vec3 uSunDir, uSunE;
uniform vec3 uWind;            // xy direction, z speed
uniform float uTime;
uniform vec4 uWaterCol;        // body scatter colour, unresolved slope variance
uniform vec4 uSss;             // sss colour, strength
uniform vec4 uFoamP;           // J threshold, J gain, intensity
uniform vec4 uFogExt;          // extinction per metre rgb, floor
uniform vec2 uFoamCenter;
uniform float uFoamSize;
uniform vec2 uResolution;
uniform float uNear, uFar;
uniform vec3 uAbsorb;          // water absorption per metre (for submerged bodies)
in vec3 vRel;
in vec2 vXZ;
in float vHeight;
in float vViewZ;
layout(location = 0) out vec4 fragColor;
const float PI = 3.14159265358979;
const float ENV_MAXMIP = 8.0;
const float ENV_TEXELS_PER_RAD = 256.0 / 1.5708;
${THERMAL_UNIFORMS_GLSL}${THERMAL_GLSL}
// @hook:ocean-frag-decl
float fresnelWater(float c) { float x = 1.0 - clamp(c, 0.0, 1.0); float x2 = x * x; return 0.02 + 0.98 * x2 * x2 * x; }
float lambdaSmith(vec3 w, vec3 cov) {
  float xz2 = max(dot(w.xz, w.xz), 1e-8);
  float il = inversesqrt(xz2);
  vec2 az = w.xz * il;
  float sig2 = az.x * az.x * cov.x + az.y * az.y * cov.y + 2.0 * az.x * az.y * cov.z;
  float a = max(w.y * il, 1e-3) / sqrt(2.0 * max(sig2, 1e-7));
  if (a >= 1.6) return 0.0;
  return (1.0 - 1.259 * a + 0.396 * a * a) / (3.535 * a + 2.181 * a * a);
}
float foamPattern(vec2 xz, float t) {
  float a = texture(uFoamTex, xz * 0.23 + vec2(0.013, 0.0) * t).r;
  float b = texture(uFoamTex, xz * 0.71 + vec2(0.37, 0.71)).g;
  return a * 0.78 + b * 0.22;
}
void main() {
  vec2 xz = vXZ;
  // @hook:ocean-frag-start
  vec2 s = vec2(0.0);
  float jxx = 0.0, jzz = 0.0, jxz = 0.0;
  vec3 cov = vec3(0.0);
  // wind gusts (cat's paws): patches where the short ripples are stronger or calmer
  float gustN = texture(uFoamTex, xz * 0.011 + uWind.xy * (uTime * 0.0004)).b;
  gustN = 0.6 * gustN + 0.4 * texture(uFoamTex, mat2(0.8, -0.6, 0.6, 0.8) * xz * 0.0047).b;   // break up the 91 m tiling
  float gust = mix(0.35, 1.2, smoothstep(0.3, 0.72, gustN));
  ${forC(c => `{ float gk = ${c >= 4 ? 'gust' : c >= 3 ? 'mix(1.0, gust, 0.5)' : '1.0'};
    vec2 uv = xz / uCascL[${c}]; vec4 dv = texture(uDeriv${c}, uv) * gk; vec4 mv = texture(uMom${c}, uv);
    s += dv.xy; jxx += dv.z; jzz += dv.w; jxz += mv.w * gk; vec3 m3 = mv.xyz * (gk * gk);
    cov += vec3(max(m3.x - dv.x * dv.x, 0.0), max(m3.y - dv.y * dv.y, 0.0), m3.z - dv.x * dv.y); }`)}
  // @hook:ocean-frag-slopes   (add extra slope s, jxx/jzz/jxz and slope covariance cov before the normal is built)
  float dist = length(vRel);
  vec3 V = -vRel / dist;
  vec3 Ld = uSunDir;

  // slope statistics (LEAN) + unresolved roughness growing with distance
  float unres = uWaterCol.w + 0.0035 * smoothstep(30.0, 500.0, dist);
  cov.x += unres; cov.y += unres;
  float jx = max(1.0 + jxx, 0.15), jz = max(1.0 + jzz, 0.15);
  vec2 sl = vec2(s.x / jx, s.y / jz);
  vec3 nrm = normalize(vec3(-sl.x, 1.0, -sl.y));
  // specular anti-aliasing: slope variation across the pixel widens the lobe
  vec2 dsx = dFdx(sl), dsy = dFdy(sl);
  vec3 aa = vec3(dsx.x * dsx.x + dsy.x * dsy.x, dsx.y * dsx.y + dsy.y * dsy.y, dsx.x * dsx.y + dsy.x * dsy.y) * 0.5;
  cov += aa * min(1.0, 0.03 / max(max(aa.x, aa.y), 1e-8));
  float J = (1.0 + jxx) * (1.0 + jzz) - jxz * jxz;
  // @hook:ocean-frag-normal   (nrm, sl, cov, J final)

  vec3 sunE = uSunE;
  vec3 skyUp = textureLod(uEnv, vec3(0.0, 1.0, 0.0), ENV_MAXMIP).rgb;
  vec3 skyHemi = skyUp * 0.4 + 0.15 * (
      textureLod(uEnv, vec3(0.94, 0.34, 0.0), ENV_MAXMIP - 1.0).rgb + textureLod(uEnv, vec3(-0.94, 0.34, 0.0), ENV_MAXMIP - 1.0).rgb +
      textureLod(uEnv, vec3(0.0, 0.34, 0.94), ENV_MAXMIP - 1.0).rgb + textureLod(uEnv, vec3(0.0, 0.34, -0.94), ENV_MAXMIP - 1.0).rgb);
  vec3 skyN = textureLod(uEnv, nrm, ENV_MAXMIP - 1.0).rgb;

  // ---- sun glitter: anisotropic Gaussian slope distribution
  vec3 Hv = normalize(V + Ld);
  vec2 hs = -Hv.xz / max(Hv.y, 1e-3);
  vec2 dsl = hs - sl;
  float det = max(cov.x * cov.y - cov.z * cov.z, 1e-10);
  float e = (dsl.x * dsl.x * cov.y + dsl.y * dsl.y * cov.x - 2.0 * dsl.x * dsl.y * cov.z) / det;
  float pdf = exp(-0.5 * e) / (2.0 * PI * sqrt(det));
  float hy2 = Hv.y * Hv.y;
  float Dm = pdf / max(hy2 * hy2, 1e-6);
  float Gs = 1.0 / (1.0 + lambdaSmith(V, cov) + lambdaSmith(Ld, cov));
  float Fh = fresnelWater(dot(V, Hv));
  vec3 spec = min(sunE * (Fh * Dm * Gs / (4.0 * max(V.y, 0.02))) * step(0.0, Ld.y), vec3(6.0e4));
  // @hook:ocean-frag-specular   (spec = sun glitter radiance)

  // ---- sky reflection (visible facets tilt towards the viewer at grazing angles)
  float sig = sqrt(max(cov.x + cov.y, 1e-6) * 0.5);
  vec3 vh = normalize(vec3(V.x, 0.0, V.z) + vec3(1e-5, 0.0, 0.0));
  float sigV = sqrt(max(vh.x * vh.x * cov.x + vh.z * vh.z * cov.y + 2.0 * vh.x * vh.z * cov.z, 1e-6));
  float aV = max(V.y, 1e-3) / max(sqrt(1.0 - V.y * V.y), 1e-3) / (1.4142 * sigV);
  float tilt = 0.32 * sigV * (1.0 - smoothstep(0.0, 2.5, aV));
  vec3 nV = normalize(nrm + vh * tilt);
  vec3 R = reflect(-V, nV);
  R.y = abs(R.y) * 0.4 + max(R.y, 0.0) * 0.6;
  R = normalize(R);
  float envLod = clamp(log2(max(2.0 * sig * ENV_TEXELS_PER_RAD, 1.0)), 0.0, 5.0);   // coarser mips mix in the below-horizon fill
  vec3 refl = textureLod(uEnv, R, envLod).rgb;
  vec3 nF = normalize(mix(nrm, vec3(0.0, 1.0, 0.0), 0.45));
  float Fr = fresnelWater(max(dot(nF, V), 0.0));
  // @hook:ocean-frag-reflection   (refl = reflected radiance, Fr = Fresnel weight)

  // ---- water body: upwelling scatter + backlit subsurface glow
  vec3 ambient = skyUp * 1.3 + skyN * 0.5;
  vec3 body = uWaterCol.rgb * (skyHemi * 1.6 + sunE * max(Ld.y, 0.0) * 0.8);
  float crest = clamp((vHeight - 0.05) / 0.9, 0.0, 1.0);
  float toSun = max(dot(Ld, -V), 0.0);
  float thin = pow(clamp(0.5 - 0.5 * dot(Ld, nrm), 0.0, 1.0), 3.0);
  body += uSss.rgb * sunE * (uSss.w * crest * crest * pow(toSun, 4.0) * thin * exp(-dist / 600.0));

  // ---- bodies under the surface, seen through the water with depth-dependent absorption
  vec2 suv0 = gl_FragCoord.xy / uResolution;
  vec2 cr = normalize(vec2(viewMatrix[0][0], viewMatrix[2][0]));      // camera right, horizontal even looking down
  vec2 off = vec2(dot(nrm.xz, cr), dot(nrm.xz, vec2(cr.y, -cr.x))) * 0.035 * clamp(6.0 / dist, 0.0, 1.0);
  vec4 rc = texture(uRefr, suv0 + off);
  if (rc.a > 0.01) {
    float sd = texture(uRefrDepth, suv0 + off).r;
    float sceneZ = uNear * uFar / (uFar - sd * (uFar - uNear));
    if (sceneZ > vViewZ) {
      vec3 surfP = cameraPosition + vRel;
      vec3 sceneP = cameraPosition + vRel * (sceneZ / vViewZ);
      float d = max(surfP.y - sceneP.y, 0.0);
      float hd = length(sceneP.xz - surfP.xz);
      // only what a ray refracted at this surface point can reach (Snell's window, ~48.6 deg)
      // hd is measured along the unrefracted ray, so the cut-off sits near 60-70 deg from vertical
      float reach = smoothstep(d * 2.75 + 0.3, d * 1.6 + 0.08, hd);
      float path = sqrt(d * d + hd * hd);
      vec3 T = exp(-uAbsorb * (path * 2.0 + 0.25)) * reach;
      // forward scattering: deeper parts look softer
      vec4 rc2 = texture(uRefr, suv0 + off * 2.5 + vec2(0.003, 0.002) * clamp(6.0 / dist, 0.0, 1.0));
      vec3 seen = mix(rc.rgb, rc2.rgb, clamp(path * 0.45, 0.0, 0.65) * rc2.a);
      body = mix(body, seen * T + body * (1.0 - T), rc.a);
    }
  }

  // ---- foam: persistent world-space map near the camera target, instantaneous crest foam beyond
  float w0 = 1.0 - smoothstep(0.22 * uFoamSize, 0.48 * uFoamSize, length(xz - uFoamCenter));   // round, soft edge
  vec4 fwl = texture(uFoamWorld, (xz - uFoamCenter) / uFoamSize + 0.5) * w0;
  float fresh = clamp((uFoamP.x - J) * uFoamP.y, 0.0, 1.0);
  float cover = clamp((max(fwl.r, fresh * (1.0 - w0 * 0.7)) + fwl.g * 0.6) * uFoamP.z, 0.0, 1.5);
  vec2 wd = uWind.xy;
  vec2 ws = vec2(dot(xz, wd), dot(xz, vec2(-wd.y, wd.x)));
  float streak = texture(uFoamTex, ws * vec2(0.035, 0.11)).a;
  float freshK = clamp(fwl.g * 2.0 + fresh, 0.0, 1.0);
  cover *= mix(smoothstep(0.25, 0.75, streak) * 1.5, 1.0, freshK);
  float pat = foamPattern(xz, uTime);
  float fw = fwidth(pat) + 0.03;
  float soft = 0.04 + 0.14 * (1.0 - clamp(cover, 0.0, 1.0));
  float fm = smoothstep(1.0 - cover - fw - soft, 1.0 - cover + fw + soft, pat);
  float foamMask = clamp(fm * smoothstep(0.0, 0.5, cover), 0.0, 1.0) * (0.68 + 0.32 * smoothstep(0.3, 0.9, cover));
  foamMask = max(foamMask, smoothstep(0.35, 0.9, fwl.g) * smoothstep(0.25, 0.6, pat + 0.25) * 0.95);
  // @hook:ocean-frag-foam   (cover, foamMask)

  if (uThermal > 0.5) {
    // LWIR: emission at the skin temperature + reflected sky, weighted by the angular emissivity.
    // Water is opaque at 10 um, so nothing below the surface (the refraction buffer) is seen.
    float eW = waterEmissivity(max(dot(nrm, V), 0.0));
    vec3 Rt = reflect(-V, nrm);
    float Lsky = skyRadianceLWIR(max(Rt.y, 0.0), uTair, uRH, uCloudFrac, uTcloud);
    float Tskin = waterSkinTemp(uTwater, uWindSpd);                // cool skin layer (Donlon 2002)
    float Lw = eW * planckBand(Tskin) + (1.0 - eW) * Lsky;
    float eF = foamEmissivity(max(dot(nrm, V), 0.0));
    float Lf = eF * planckBand(uTwater + 0.05) + (1.0 - eF) * Lsky; // foam: near-blackbody, mixed water
    float Lt = mix(Lw, Lf, foamMask);
    float tau = pathTransmittanceLWIR(dist, uRH, uTair);
    fragColor = vec4(Lt * tau + planckBand(uTair) * (1.0 - tau), 0.0, 0.0, 1.0);
    return;
  }
  body += uSss.rgb * (sunE * 0.05 + ambient * 0.35) * clamp(cover, 0.0, 1.0);
  body += vec3(0.10, 0.32, 0.30) * skyUp * fwl.g * 0.35;
  float bub = texture(uFoamTex, xz * 1.9 + vec2(0.21, 0.53)).g;
  vec3 foamAlb = vec3(0.82, 0.87, 0.92) * (0.78 + 0.3 * bub);
  vec3 foamAmb = skyHemi + skyN * 0.3;
  vec3 foamAmbN = mix(foamAmb, vec3(dot(foamAmb, vec3(0.2126, 0.7152, 0.0722))), 0.85);
  vec3 foamLit = foamAlb * (sunE * max(dot(nrm, Ld), 0.0) * 0.318 * 0.5 + foamAmbN * 1.55);

  vec3 col = body * (1.0 - Fr) + refl * Fr + spec;
  col = mix(col, foamLit, foamMask * 0.95);
  // @hook:ocean-frag-combine   (col = surface radiance before aerial perspective)

  // ---- aerial perspective
  // haze thins with height: average density along the ray (scale height ~1.2 km)
  float hr = max(cameraPosition.y, 0.0) / 1200.0;
  float dens = hr > 1e-3 ? (1.0 - exp(-hr)) / hr : 1.0;
  vec3 T = exp(-max(uFogExt.rgb, vec3(uFogExt.w)) * dist * dens);
  T *= 1.0 - smoothstep(12000.0, 19000.0, length(vRel.xz));   // fully fogged before the mesh edge
  // in-scattered light along the view ray: sample the sky mirrored about the horizon, so the scattering
  // angle to the sun stays close to that of the real (downward) ray; near the horizon this is the horizon sky
  vec3 fogCol = textureLod(uEnv, normalize(vec3(-V.x, max(V.y, 0.002), -V.z)), 1.0).rgb;
  col = col * T + fogCol * (1.0 - T);
  // @hook:ocean-frag-final
  fragColor = vec4(col, 1.0);
}`;
const oceanGeo = makeOceanGeometry();
const oceanU = {
  uCascL: { value: CASC_L }, uCenter: { value: new THREE.Vector2() }, uLodK: { value: oceanGeo.lodK }, uLodMin: { value: 0.03 },
  uEnv: { value: envRT.texture }, uFoamTex: { value: FOAM_TEX }, uFoamWorld: { value: foamA.texture },
  uRefr: { value: refrRT.texture }, uRefrDepth: { value: refrRT.depthTexture },
  uSunDir: { value: new THREE.Vector3(0, 1, 0) }, uSunE: { value: new THREE.Vector3(10, 10, 10) },
  uWind: { value: new THREE.Vector3(1, 0, 7) }, uTime: { value: 0 },
  uWaterCol: { value: new THREE.Vector4(0.0256, 0.128, 0.154, 0.0024) }, uSss: { value: new THREE.Vector4(0.035, 0.12, 0.085, 2.6) },
  uFoamP: { value: new THREE.Vector4(0.48, 2.5, 1, 1) }, uFogExt: { value: new THREE.Vector4(1e-5, 1e-5, 1e-5, 1.1e-4) },
  uFoamCenter: { value: foamCenter }, uFoamSize: { value: FOAM_SIZE },
  uResolution: { value: drawSize.clone() }, uNear: { value: camera.near }, uFar: { value: camera.far },
  uAbsorb: { value: new THREE.Vector3(0.42, 0.085, 0.11) },
  ...THERM,
  ...Object.fromEntries(CASC_L.flatMap((_, c) => [[`uDisp${c}`, { value: DISP[c] }], [`uDeriv${c}`, { value: DERIV[c] }], [`uMom${c}`, { value: MOM[c] }]])),
};
const ocean = new THREE.Mesh(oceanGeo.geo, new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3, uniforms: oceanU, vertexShader: OCEAN_VERT, fragmentShader: OCEAN_FRAG,
}));
ocean.frustumCulled = false;
scene.add(ocean);

