// ============================================================================
// Underwater view: camera below the surface, the surface seen from below (Snell's window, total internal
// reflection, foam undersides), water-volume pass (absorption, in-scattering, god rays), caustics on bodies,
// marine snow, bubbles, and a half-over / half-under split view with a meniscus line.
// ============================================================================
import { Pass, FullScreenQuad } from 'three/addons/postprocessing/Pass.js';

// ---- water optics shared by the ocean underside and the volume pass
const UW_OPTICS_GLSL = /* glsl */`
uniform vec4 uUwPlane, uUwInfo;
uniform vec3 uUwSigT, uUwSigS, uUwKd, uUwSunW, uUwSunE;
uniform float uUwTime;
// forward-peaked particle phase function (Henyey-Greenstein g = 0.85) with a little isotropic backscatter
float uwPhase(float c) {
  const float g = 0.85;
  float hg = (1.0 - g * g) / (12.566371 * pow(max(1.0 + g * g - 2.0 * g * c, 1e-4), 1.5));
  return mix(hg, 0.0795775, 0.15);
}
// diffuse irradiance in the water just below the surface: sky dome + multiply scattered sunlight
vec3 uwAmbient(vec3 skyHemi) { return 2.6 * skyHemi + 0.3 * uUwSunE * uUwSunW.y; }
// single-scattered radiance along a ray from depth zc (m) in direction rd over a length D,
// with the downwelling light decaying as exp(-Kd z): closed form of the in-scatter integral
vec3 uwInscatter(float zc, vec3 rd, float D, vec3 Eamb) {
  vec3 k = max(uUwSigT - uUwKd * rd.y, vec3(1e-3));
  vec3 I = exp(-uUwKd * zc) * (1.0 - exp(-k * D)) / k;
  return uUwSigS * (uUwSunE * uwPhase(dot(rd, uUwSunW)) + Eamb * 0.0795775) * I;
}
// is this camera below (or straddling) the surface the uniforms describe? (sensor cameras far away: no)
float uwCamHeight() { return dot(uUwPlane.xyz, cameraPosition) + uUwPlane.w; }
bool uwCamNear() { return uUwInfo.x > 0.5 && distance(cameraPosition.xz, uUwInfo.yz) < 3.0; }
`;

// ---- the camera may go below the surface: polar limit + double-sided ocean
function uwApplyCameraLimits() {
  controls.maxPolarAngle = params.uwAllowUnder ? Math.PI * 0.97 : Math.PI * 0.5;
  const side = params.uwAllowUnder ? THREE.DoubleSide : THREE.FrontSide;
  if (ocean.material.side !== side) { ocean.material.side = side; ocean.material.needsUpdate = true; }
  if (!params.uwAllowUnder) params.uwSplit = false;
}

// ---- the ocean surface seen from below
Object.assign(oceanU, uwU);
injectShader(ocean.material, 'ocean-frag-decl', UW_OPTICS_GLSL);
injectShader(ocean.material, 'ocean-frag-foam', /* glsl */`
  // ---- underwater: this fragment is the underside of the surface (uw_underwater.js)
  if (uwCamNear() && (uwCamHeight() < -uUwInfo.w || (uwCamHeight() < uUwInfo.w && !gl_FrontFacing))) {
    if (uThermal > 0.5) { fragColor = vec4(planckBand(uTwater), 0.0, 0.0, 1.0); return; }   // LWIR: opaque water
    vec3 uwD = -V;                                 // the view ray, travelling up through the water
    vec3 uwN = nrm;
    float uwCi = dot(uwD, uwN);
    if (uwCi < 0.05) { uwN = normalize(uwN + uwD * (0.05 - uwCi)); uwCi = dot(uwD, uwN); }
    // Fresnel water -> air with dispersion (red..blue): total internal reflection beyond ~48.6 deg,
    // softened by the unresolved slopes; the slightly different critical angles fringe the window edge
    const vec3 UW_ETA = vec3(1.3305, 1.3340, 1.3390);
    vec3 uwS2 = UW_ETA * UW_ETA * (1.0 - uwCi * uwCi);
    float uwSoft = 0.01 + 1.5 * sig;
    vec3 uwTir = smoothstep(1.0 - uwSoft, 1.0 + 0.15 * uwSoft, uwS2);
    vec3 uwCt = sqrt(max(1.0 - uwS2, 0.0));
    vec3 uwRs = (UW_ETA * uwCi - uwCt) / (UW_ETA * uwCi + uwCt);
    vec3 uwRp = (uwCi - UW_ETA * uwCt) / (uwCi + UW_ETA * uwCt);
    vec3 uwF = mix(0.5 * (uwRs * uwRs + uwRp * uwRp), vec3(1.0), uwTir);
    // Snell's window: the whole sky squeezed into a ~97 deg cone; radiance gains n^2 entering the water
    vec3 uwT = refract(uwD, -uwN, UW_ETA.y);
    if (dot(uwT, uwT) < 1e-6) uwT = vec3(uwD.x, 0.0, uwD.z) + vec3(1e-4, 0.0, 0.0);
    uwT.y = max(uwT.y, 0.003);
    uwT = normalize(uwT);
    float uwLod = clamp(log2(max(sig * ENV_TEXELS_PER_RAD * 0.8 / max(uwCt.y, 0.06), 1.0)), 0.0, 6.0);
    vec3 uwCol = (1.0 - uwF) * textureLod(uEnv, uwT, uwLod).rgb * 1.78;
    // the sun refracted through the wave facets (microfacet BTDF, Walter et al. 2007)
    vec3 uwH = -(Ld + 1.334 * V);
    float uwHl = length(uwH);
    uwH /= uwHl;
    float uwIh = dot(Ld, uwH), uwOh = dot(V, uwH);
    if (uwH.y > 0.05 && uwIh > 0.0 && uwOh < 0.0 && Ld.y > 0.0) {
      vec2 uwDs = -uwH.xz / uwH.y - sl;
      float uwE = (uwDs.x * uwDs.x * cov.y + uwDs.y * uwDs.y * cov.x - 2.0 * uwDs.x * uwDs.y * cov.z) / det;
      float uwH2 = uwH.y * uwH.y;
      float uwDm = exp(-0.5 * uwE) / (2.0 * PI * sqrt(det)) / (uwH2 * uwH2);
      float uwFt = 1.0 - fresnelWater(uwIh);
      uwCol += min(sunE * (uwIh * -uwOh * 1.78 * uwFt * uwDm / (max(-V.y, 0.05) * uwHl * uwHl)), vec3(6.0e4));
    }
    // outside the window: the surface mirrors the deep water below
    vec3 uwR = reflect(uwD, uwN);
    uwR.y = min(uwR.y, -0.02);
    uwR = normalize(uwR);
    vec3 uwAmb = uwAmbient(skyHemi);
    uwCol += uwF * uwInscatter(0.0, uwR, 1.0e4, uwAmb);
    // dye patch above: fluorescein absorbs the blue and glows green
    float uwDy = uwDyeCover(xz);
    uwCol = mix(uwCol, uwCol * vec3(0.62, 1.0, 0.32) + vec3(0.004, 0.016, 0.0) * uwAmb, uwDy * 0.6);
    // foam and bubble clouds seen from below: bright, diffusely transmitted light that blocks the window
    vec3 uwFoamL = vec3(0.78, 0.85, 0.88) * (uUwSunE * uUwSunW.y * 0.09 + skyHemi * 0.5);
    uwCol = mix(uwCol, uwFoamL, clamp(foamMask * 1.15, 0.0, 1.0) * 0.92);
    uwCol += vec3(0.55, 0.7, 0.72) * uwAmb * 0.025 * fwl.g;
    fragColor = vec4(uwCol, 1.0);
    return;
  }
  //`);
uwApplyCameraLimits();

// ---- caustics and depth-dependent light on bodies and gear (every makeWet material)
const UW_CAUSTIC_GLSL = /* glsl */`
vec2 uwHash2(vec2 p) {
  p = vec2(dot(p, vec2(127.1, 311.7)), dot(p, vec2(269.5, 183.3)));
  return fract(sin(p) * 43758.5453);
}
// distance to the nearest cell border (F2 - F1) of drifting Voronoi sites: thin bright lines at the borders
float uwCell(vec2 p, float t) {
  vec2 i = floor(p), f = fract(p);
  float d1 = 8.0, d2 = 8.0;
  for (int y = -1; y <= 1; y++) for (int x = -1; x <= 1; x++) {
    vec2 g = vec2(float(x), float(y));
    vec2 o = 0.5 + 0.42 * sin(t + 6.2831 * uwHash2(i + g));
    vec2 r = g + o - f;
    float d = dot(r, r);
    if (d < d1) { d2 = d1; d1 = d; } else if (d < d2) d2 = d;
  }
  return sqrt(d2) - sqrt(d1);
}
// light focused by the surface waves on a point 'depth' metres down; mean ~1 (energy is redistributed)
float uwCaustic(vec2 p, float t, float depth) {
  float k = mix(12.0, 3.5, clamp(depth / 5.0, 0.0, 1.0));     // deeper: broader, softer lines
  float a = uwCell(p * 4.6, t * 1.6);
  float b = uwCell(mat2(0.8, -0.6, 0.6, 0.8) * p * 6.3 + 3.7, t * 2.1 + 2.0);
  return 0.25 + (exp(-a * k) * 0.65 + exp(-b * k * 1.2) * 0.45) * 0.38 * k;
}
`;
const uwCausU = { value: new THREE.Vector4(0, 1, 1, 0) };   // time, strength, enabled
HOOKS.personShader.push(sh => {
  Object.assign(sh.uniforms, { uUwPlane: uwU.uUwPlane, uUwInfo: uwU.uUwInfo, uUwKd: uwU.uUwKd, uUwSunW: uwU.uUwSunW, uUwSunT: uwU.uUwSunT, uUwCaus: uwCausU });
  sh.fragmentShader = 'uniform vec4 uUwPlane, uUwInfo, uUwCaus;\nuniform vec3 uUwKd, uUwSunW;\nuniform float uUwSunT;\n' + UW_CAUSTIC_GLSL + sh.fragmentShader
    .replace('#include <lights_fragment_end>', `#include <lights_fragment_end>
  {
    float uwDep = -(dot(uWaterPlane.xyz, vWetPos) + uWaterPlane.w);
    if (uwDep > 0.0) {
      float uwIn = smoothstep(0.0, 0.06, uwDep);
#if NUM_DIR_LIGHTS > 0
      // below the waterline the sun arrives refracted (steeper) and reduced by the Fresnel transmission
      vec3 uwLv = normalize((viewMatrix * vec4(uUwSunW, 0.0)).xyz);
      vec3 uwDirect = directionalLights[0].color * (uUwSunT * max(dot(normal, uwLv), 0.0)) * BRDF_Lambert(material.diffuseColor);
      reflectedLight.directDiffuse = mix(reflectedLight.directDiffuse, uwDirect, uwIn);
      reflectedLight.directSpecular *= 1.0 - 0.8 * uwIn;
#endif
      // the sky light comes through Snell's window, not from the horizon; no sky reflections under water
      reflectedLight.indirectDiffuse *= 1.0 - 0.35 * uwIn;
      reflectedLight.indirectSpecular *= 1.0 - 0.75 * uwIn;
      // caustics: project the point to the surface along the refracted sun ray and look up the focusing pattern
      if (uUwCaus.z > 0.5) {
        vec2 uwP = vWetPos.xz + uUwSunW.xz / max(uUwSunW.y, 0.3) * uwDep;
        float uwC = uwCaustic(uwP, uUwCaus.x, uwDep);
        float uwK = uUwCaus.y * smoothstep(0.0, 0.25, uwDep) * exp(-uwDep * 0.12);
        reflectedLight.directDiffuse *= mix(1.0, uwC, uwK);
      }
    }
  }`)
    .replace('#include <dithering_fragment>', `{
    // seen from under water: the light reaching a submerged point has crossed 'depth' metres of water
    float uwDep = -(dot(uWaterPlane.xyz, vWetPos) + uWaterPlane.w);
    float uwCs = dot(uUwPlane.xyz, cameraPosition) + uUwPlane.w;
    if (uUwInfo.x > 0.5 && distance(cameraPosition.xz, uUwInfo.yz) < 3.0 && uwCs < uUwInfo.w && uwDep > 0.0)
      gl_FragColor.rgb *= exp(-uUwKd * uwDep);
  }
#include <dithering_fragment>`);
});

// ---- water-volume pass (inserted right after the RenderPass)
// Needs the scene depth: a DepthTexture is attached to both composer ping-pong targets (MSAA depth is resolved
// into it by three.js), and the pass reads readBuffer.depthTexture.
for (const rt of [composer.renderTarget1, composer.renderTarget2]) {
  if (!rt.depthTexture) { rt.depthTexture = new THREE.DepthTexture(rt.width, rt.height); rt.dispose(); }
}
const UW_OCC = 16;
const UW_FS_VERT = 'out vec2 vUv; void main() { vUv = uv; gl_Position = vec4(position.xy, 0.0, 1.0); }';
const UW_POST_FRAG = CASC_SAMPLERS(['uDisp']) + NOISE_GLSL + UW_OPTICS_GLSL + /* glsl */`
#define NC ${NC}
#define UW_OCC ${UW_OCC}
uniform sampler2D tDiffuse, tDepth;
uniform samplerCube uEnv;
uniform float uCascL[NC], uDispLod[NC];
uniform mat4 uProjInv, uCamWorld;
uniform vec3 uCamPos;
uniform vec2 uWind;
uniform float uNear, uFar, uMode, uShaft, uExpo, uFrame, uMarch;
uniform int uSteps, uOccN;
uniform vec4 uOcc[UW_OCC];
in vec2 vUv;
layout(location = 0) out vec4 fragColor;
float ign(vec2 p) { return fract(52.9829189 * fract(dot(p, vec2(0.06711056, 0.00583715)))); }
// the rendered surface: same displacement sum and mip levels as the ocean vertex shader near the camera
vec3 uwDispAt(vec2 xz) {
  vec3 d = vec3(0.0);
  ${forC(c => `d += textureLod(uDisp${c}, xz / uCascL[${c}], uDispLod[${c}]).xyz;`)}
  return d;
}
float uwWaterHeight(vec2 p) {
  vec2 x0 = p;
  for (int i = 0; i < 3; i++) x0 = p - uwDispAt(x0).xz;   // invert the horizontal (choppy) displacement
  return uwDispAt(x0).y;
}
// light shafts: brightness at P is the focusing of the surface where the refracted sun ray through P enters
float uwShaft(vec3 P, float z) {
  vec2 ps = P.xz + uUwSunW.xz * (z / max(uUwSunW.y, 0.3));
  // two drifting ridged noise layers multiplied: a network of thin focused lines, like the caustics above
  vec2 q = ps * 1.3 + uWind * (uUwTime * 0.12);
  float n1 = gnoise(q + vec2(uUwTime * 0.21, 0.0));
  float n2 = gnoise(mat2(0.8, -0.6, 0.6, 0.8) * q * 1.9 + vec2(-uUwTime * 0.17, uUwTime * 0.13));
  float c = clamp((1.0 - abs(n1) * 2.2) * (1.0 - abs(n2) * 2.2), 0.0, 1.0);
  float S = mix(1.0, c * c * c * 6.0, uShaft * exp(-z * 0.04));
  // shadow columns below people, the raft and the buoy (penumbra widens with depth)
  for (int i = 0; i < UW_OCC; i++) {
    if (i >= uOccN) break;
    vec4 o = uOcc[i];
    float r = o.z + z * 0.06;
    S *= 1.0 - o.w * smoothstep(r, o.z * 0.35, length(ps - o.xy)) / (1.0 + z * 0.08);
  }
  return S;
}
void main() {
  vec3 col = texture(tDiffuse, vUv).rgb;
  vec2 ndc = vUv * 2.0 - 1.0;
  vec4 vn = uProjInv * vec4(ndc, -1.0, 1.0);
  vn.xyz /= vn.w;                                   // this pixel's point on the near plane (view space)
  vec3 dirV = normalize(vn.xyz);
  vec3 rd = normalize(mat3(uCamWorld) * dirV);
  vec3 pN = (uCamWorld * vec4(vn.xyz, 1.0)).xyz;
  // height of the ray origin above the water: exact surface when the near plane cuts it, else the camera plane
  float s;
  if (uMode > 1.5) s = pN.y - uwWaterHeight(pN.xz) + 0.004 * sin(ndc.x * 23.0 + uUwTime * 2.3) * sin(ndc.x * 7.0 - uUwTime * 1.1);
  else s = dot(uUwPlane.xyz, pN) + uUwPlane.w;
  float sPix = s / max(fwidth(s), 1e-7);          // signed distance to the waterline on the lens, in pixels
  vec3 skyHemi = textureLod(uEnv, vec3(0.0, 1.0, 0.0), 8.0).rgb * 0.4 + 0.15 * (
      textureLod(uEnv, vec3(0.94, 0.34, 0.0), 7.0).rgb + textureLod(uEnv, vec3(-0.94, 0.34, 0.0), 7.0).rgb +
      textureLod(uEnv, vec3(0.0, 0.34, 0.94), 7.0).rgb + textureLod(uEnv, vec3(0.0, 0.34, -0.94), 7.0).rgb);
  vec3 Eamb = uwAmbient(skyHemi);
  if (s < 0.0) {
    float dz = texture(tDepth, vUv).r;
    float D = dz >= 0.999999 ? 1.0e4 : uNear * uFar / (uFar - dz * (uFar - uNear)) / max(-dirV.z, 1e-4);
    float zc = -s;
    vec3 k = max(uUwSigT - uUwKd * rd.y, vec3(1e-3));
    vec3 e0 = exp(-uUwKd * zc);
    // ambient part: closed form over the whole ray
    vec3 Lamb = uUwSigS * Eamb * 0.0795775 * e0 * (1.0 - exp(-k * D)) / k;
    // sun part: shafts marched over the first uMarch metres, closed form beyond
    float Lm = min(D, uMarch);
    float dt = Lm / float(uSteps);
    float j = ign(gl_FragCoord.xy + uFrame * 5.588238);
    vec3 acc = vec3(0.0);
    for (int i = 0; i < 48; i++) {
      if (i >= uSteps) break;
      float t = (float(i) + j) * dt;
      float z = max(zc - rd.y * t, 0.0);
      acc += exp(-k * t) * uwShaft(uCamPos + rd * t, z);
    }
    vec3 Lsun = uUwSigS * uUwSunE * uwPhase(dot(rd, uUwSunW)) * e0 * (acc * dt + (exp(-k * Lm) - exp(-k * D)) / k);
    vec3 uwc = col * exp(-uUwSigT * D) + Lamb + Lsun;
    col = uwc * uExpo;
  }
  if (uMode > 1.5) {
    // meniscus: the water surface touching the lens is a dark line with light trapped just under it,
    // and the film of water draining off the upper half dims and softens the image a little
    float w = sPix;
    col *= 1.0 - 0.9 * exp(-w * w / 5.0);
    col += vec3(0.45, 0.75, 0.8) * Eamb * 0.03 * exp(-(w + 4.0) * (w + 4.0) / 6.0);
    col *= 1.0 - 0.18 * exp(-max(w, 0.0) / 14.0);
  }
  fragColor = vec4(col, 1.0);
}`;
const uwPostU = {
  tDiffuse: { value: null }, tDepth: { value: null }, uEnv: { value: envRT.texture },
  uCascL: { value: CASC_L }, uDispLod: { value: CASC_L.map(L => Math.log2(Math.max(0.03 / (L / N), 1)) + 0.5) },
  uProjInv: { value: new THREE.Matrix4() }, uCamWorld: { value: new THREE.Matrix4() }, uCamPos: { value: new THREE.Vector3() },
  uWind: { value: new THREE.Vector2(1, 0) },
  uNear: { value: 0.1 }, uFar: { value: 60000 }, uMode: { value: 1 }, uShaft: { value: 1 }, uExpo: { value: 1.6 },
  uFrame: { value: 0 }, uMarch: { value: 30 }, uSteps: { value: 16 }, uOccN: { value: 0 },
  uOcc: { value: Array.from({ length: UW_OCC }, () => new THREE.Vector4()) },
  ...uwU,
  ...Object.fromEntries(CASC_L.map((_, c) => [`uDisp${c}`, { value: DISP[c] }])),
};
const uwPostMat = new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3, vertexShader: UW_FS_VERT, fragmentShader: UW_POST_FRAG, uniforms: uwPostU, depthTest: false, depthWrite: false,
});

// ---- marine snow and bubbles: drawn by the pass after the volume (soft depth test against the scene depth)
const UW_PARTICLE_COMMON = UW_OPTICS_GLSL + /* glsl */`
uniform sampler2D tDepth;
uniform samplerCube uEnv;
uniform vec2 uRes;
uniform float uNear, uFar, uPx, uSurfY;
float uwViewZ(float d) { return uNear * uFar / (uFar - d * (uFar - uNear)); }
vec3 uwAmbientV() {
  vec3 hemi = textureLod(uEnv, vec3(0.0, 1.0, 0.0), 8.0).rgb * 0.7 + textureLod(uEnv, vec3(0.7, 0.3, 0.0), 7.0).rgb * 0.3;
  return uwAmbient(hemi);
}
`;
const uwSnowMat = new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3, transparent: true, depthTest: false, depthWrite: false, blending: THREE.AdditiveBlending,
  uniforms: { ...uwU, tDepth: { value: null }, uEnv: { value: envRT.texture }, uRes: { value: new THREE.Vector2(1, 1) },
    uNear: { value: 0.1 }, uFar: { value: 60000 }, uPx: { value: 500 }, uSurfY: { value: 0 }, uBox: { value: 18 }, uDrift: { value: new THREE.Vector3() } },
  vertexShader: UW_PARTICLE_COMMON + /* glsl */`
in vec4 aSeed;
uniform float uBox;
uniform vec3 uDrift;
out vec3 vCol;
out float vZ, vA;
void main() {
  // particles fill a box that wraps around the camera; they drift with the water and sink slowly
  vec3 p = aSeed.xyz * uBox + uDrift * (0.6 + 0.8 * aSeed.w) + vec3(0.0, -0.012, 0.0) * uUwTime * (0.4 + aSeed.w);
  p += 0.05 * vec3(sin(uUwTime * 0.31 + aSeed.w * 40.0), sin(uUwTime * 0.23 + aSeed.x * 31.0), cos(uUwTime * 0.27 + aSeed.y * 23.0));
  p = cameraPosition + mod(p - cameraPosition + 0.5 * uBox, uBox) - 0.5 * uBox;
  vec4 mv = viewMatrix * vec4(p, 1.0);
  gl_Position = projectionMatrix * mv;
  float d = -mv.z;
  float depth = uSurfY - p.y;
  float size = 0.003 + 0.011 * aSeed.w * aSeed.w * aSeed.w;
  float px = size * uPx / max(d, 0.05);
  gl_PointSize = clamp(px, 1.6, 40.0);
  float cover = min(px * px / (gl_PointSize * gl_PointSize), 1.0);       // sub-pixel motes: dimmer, not smaller
  float blur = 1.0 / (1.0 + 0.04 * gl_PointSize);                         // close, out-of-focus motes are faint discs
  vec3 V = normalize(p - cameraPosition);
  vec3 E = uUwSunE * (0.15 + 3.0 * uwPhase(dot(V, uUwSunW))) + uwAmbientV() * 0.35;
  vCol = E * exp(-uUwKd * max(depth, 0.0)) * exp(-uUwSigT * d) * (0.35 + 0.65 * aSeed.w);
  vA = cover * blur * smoothstep(0.06, 0.4, d) * (1.0 - smoothstep(uBox * 0.3, uBox * 0.48, d)) * smoothstep(0.1, 0.6, depth);
  vZ = d;
}`,
  fragmentShader: UW_PARTICLE_COMMON + /* glsl */`
in vec3 vCol;
in float vZ, vA;
layout(location = 0) out vec4 fragColor;
void main() {
  vec2 c = gl_PointCoord - 0.5;
  float r2 = dot(c, c) * 4.0;
  if (r2 > 1.0 || vA < 1e-3) discard;
  float sceneZ = uwViewZ(texture(tDepth, gl_FragCoord.xy / uRes).r);
  float a = vA * (1.0 - r2) * clamp((sceneZ - vZ) * 5.0, 0.0, 1.0);
  fragColor = vec4(vCol * a, 1.0);
}`,
});
const uwSnowGeo = new THREE.BufferGeometry();
{
  const n = 7000, seed = new Float32Array(n * 4);
  for (let i = 0; i < n * 4; i++) seed[i] = Math.random();
  uwSnowGeo.setAttribute('aSeed', new THREE.BufferAttribute(seed, 4));
  uwSnowGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(n * 3), 3));
}
const uwSnow = new THREE.Points(uwSnowGeo, uwSnowMat);
uwSnow.frustumCulled = false;

const UW_MAXB = 600;
const uwBubPos = new Float32Array(UW_MAXB * 3), uwBubR = new Float32Array(UW_MAXB);
const uwBubGeo = new THREE.BufferGeometry();
uwBubGeo.setAttribute('position', new THREE.BufferAttribute(uwBubPos, 3).setUsage(THREE.DynamicDrawUsage));
uwBubGeo.setAttribute('aR', new THREE.BufferAttribute(uwBubR, 1).setUsage(THREE.DynamicDrawUsage));
const uwBubMat = new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3, transparent: true, depthTest: false, depthWrite: false, blending: THREE.AdditiveBlending,
  uniforms: { ...uwU, tDepth: { value: null }, uEnv: { value: envRT.texture }, uRes: { value: new THREE.Vector2(1, 1) },
    uNear: { value: 0.1 }, uFar: { value: 60000 }, uPx: { value: 500 }, uSurfY: { value: 0 } },
  vertexShader: UW_PARTICLE_COMMON + /* glsl */`
in float aR;
out vec3 vCol;
out float vZ, vA;
void main() {
  vec4 mv = viewMatrix * vec4(position, 1.0);
  gl_Position = projectionMatrix * mv;
  float d = -mv.z;
  float px = 2.0 * aR * uPx / max(d, 0.05);
  gl_PointSize = clamp(px, 1.5, 64.0);
  float depth = max(uSurfY - position.y, 0.0);
  // a bubble mirrors the bright surface above it (total reflection at its rim)
  vec3 E = uwAmbientV() * 0.5 + uUwSunE * uUwSunW.y * 0.25;
  vCol = E * exp(-uUwKd * depth) * exp(-uUwSigT * d);
  vA = (aR > 0.0 ? 1.0 : 0.0) * min(px * px / (gl_PointSize * gl_PointSize), 1.0) * smoothstep(0.05, 0.25, d);
  vZ = d;
}`,
  fragmentShader: UW_PARTICLE_COMMON + /* glsl */`
in vec3 vCol;
in float vZ, vA;
layout(location = 0) out vec4 fragColor;
void main() {
  vec2 c = (gl_PointCoord - 0.5) * 2.0;
  float r = length(c);
  if (r > 1.0 || vA < 1e-3) discard;
  float sceneZ = uwViewZ(texture(tDepth, gl_FragCoord.xy / uRes).r);
  float rim = smoothstep(0.55, 0.92, r) * (1.0 - smoothstep(0.92, 1.0, r));
  float glint = exp(-dot(c - vec2(-0.3, 0.38), c - vec2(-0.3, 0.38)) * 40.0);
  float a = (rim * 0.9 + glint * 1.6 + 0.06) * vA * clamp((sceneZ - vZ) * 8.0, 0.0, 1.0);
  fragColor = vec4(vCol * a, 1.0);
}`,
});
const uwBubbles = new THREE.Points(uwBubGeo, uwBubMat);
uwBubbles.frustumCulled = false;
const uwParticleScene = new THREE.Scene();
uwParticleScene.add(uwSnow, uwBubbles);

class UwVolumePass extends Pass {
  constructor() { super(); this.quad = new FullScreenQuad(uwPostMat); this.needsSwap = true; }
  render(r, writeBuffer, readBuffer) {
    uwPostU.tDiffuse.value = readBuffer.texture;
    uwPostU.tDepth.value = uwSnowMat.uniforms.tDepth.value = uwBubMat.uniforms.tDepth.value = readBuffer.depthTexture;
    r.setRenderTarget(this.renderToScreen ? null : writeBuffer);
    this.quad.render(r);
    if (UW.mode !== 0 && (params.uwSnow || params.uwBubbles)) {
      uwSnow.visible = params.uwSnow;
      uwBubbles.visible = params.uwBubbles && uwBubCount > 0;
      const ac = r.autoClear;
      r.autoClear = false;
      r.render(uwParticleScene, camera);
      r.autoClear = ac;
    }
  }
  dispose() { this.quad.dispose(); }
}
const uwPass = new UwVolumePass();
uwPass.enabled = false;
composer.insertPass(uwPass, 1);

// ---- bubbles: released from the submerged parts of the people, rising and wobbling to the surface
const uwBub = Array.from({ length: UW_MAXB }, () => ({ on: false, x: 0, y: 0, z: 0, r: 0, v: 0, ph: 0, w: 0, owner: 0 }));
let uwBubCount = 0, uwBubNext = 0;
const _uwV = new THREE.Vector3();
function uwSpawnBubble(x, y, z, r, owner) {
  const b = uwBub[uwBubNext];
  uwBubNext = (uwBubNext + 1) % UW_MAXB;
  Object.assign(b, { on: true, x, y, z, r, owner, ph: Math.random() * 6.28, w: rand(6, 14) });
  b.v = clamp(0.14 + 25 * r, 0.16, 0.34);    // terminal rise speed of 1.5-9 mm bubbles (~0.2-0.3 m/s)
}
function uwUpdateBubbles(dt) {
  if (dt > 0 && params.uwBubbles) {
    people.forEach((P, pi) => {
      // a slow trickle of air from soaked clothing and the treading legs, the odd burst from the jacket
      const rate = 3 + 6 * Math.min(Math.hypot(P.vel.x, P.vel.y, P.vel.z) * 2, 1);
      if (Math.random() < rate * dt) {
        const k = 3 + Math.floor(Math.random() * 6);           // spine, hips, legs
        const pr = P.probes[k];
        if (pr && P.depth[k] > 0.15) {
          _uwV.copy(pr.local).applyQuaternion(P.q).add(P.pos);
          const burst = Math.random() < 0.1, n = burst ? 8 : 2;
          for (let i = 0; i < n; i++) uwSpawnBubble(_uwV.x + rand(-0.1, 0.1), _uwV.y + rand(-0.05, 0.05), _uwV.z + rand(-0.1, 0.1), rand(0.0015, burst ? 0.009 : 0.005), pi);
        }
      }
    });
  }
  uwBubCount = 0;
  for (let i = 0; i < UW_MAXB; i++) {
    const b = uwBub[i];
    if (b.on) {
      b.ph += b.w * dt;
      b.y += b.v * dt;
      b.x += (sea.current.x + Math.cos(b.ph) * b.v * 0.35) * dt;
      b.z += (sea.current.z + Math.sin(b.ph) * b.v * 0.35) * dt;
      const P = people[b.owner];
      const pl = P ? P.wetU.plane.value : null;
      const above = pl ? pl.x * b.x + pl.y * b.y + pl.z * b.z + pl.w : b.y;
      if (above > 0) b.on = false;
    }
    uwBubPos[i * 3] = b.x; uwBubPos[i * 3 + 1] = b.y; uwBubPos[i * 3 + 2] = b.z;
    uwBubR[i] = b.on ? b.r : 0;
    if (b.on) uwBubCount++;
  }
  uwBubGeo.attributes.position.needsUpdate = true;
  uwBubGeo.attributes.aR.needsUpdate = true;
}

// ---- per frame
const uwSnowDrift = new THREE.Vector3();
let uwFrame = 0;
const _uwOcc = [];
function uwCollectOccluders() {
  // the nearest bodies cast shadow columns into the shafts; floating gear (uw_gear.js) adds its own
  _uwOcc.length = 0;
  const c = camera.position;
  const ppl = people.map(P => P).sort((a, b) => (a.pos.x - c.x) ** 2 + (a.pos.z - c.z) ** 2 - (b.pos.x - c.x) ** 2 - (b.pos.z - c.z) ** 2);
  for (const P of ppl.slice(0, 10)) _uwOcc.push([P.pos.x, P.pos.z, 0.32, 0.75]);
  if (typeof uwGearOccluders === 'function') uwGearOccluders(_uwOcc);
  const o = uwPostU.uOcc.value;
  const n = Math.min(_uwOcc.length, UW_OCC);
  for (let i = 0; i < n; i++) o[i].set(..._uwOcc[i]);
  uwPostU.uOccN.value = n;
}
HOOKS.update.push(dt => uwUpdateBubbles(dt));
HOOKS.lateUpdate.push((dt, dtReal) => {
  uwApplyCameraLimits();
  uwUpdatePlane();
  if (params.uwAllowUnder) {
    if (params.uwSplit) {
      // the camera floats on the surface like a half-submerged dome port: it rides the waves
      camera.position.y = UW.camH + 0.004;
      camera.lookAt(controls.target);
    }
    camera.position.y = Math.max(camera.position.y, -80);
    const near = params.uwSplit ? 0.22 : 0.1;
    if (camera.near !== near) { camera.near = near; camera.updateProjectionMatrix(); }
    camera.updateMatrixWorld();
    uwUpdatePlane();
    // the drone EO/IR cameras stay in the air when they follow the main view
    if (params.sensorSource === 0 && sensorCam.position.y < UW.camH + 1.0) {
      sensorCam.position.y = UW.camH + 1.0;
      sensorCam.lookAt(controls.target);
      sensorCam.updateMatrixWorld();
    }
  } else if (camera.near !== 0.1) { camera.near = 0.1; camera.updateProjectionMatrix(); }
  oceanU.uNear.value = camera.near;
  uwUpdateOptics();
  uwUpdateSun();
  uwU.uUwTime.value = simTime;
  uwCausU.value.set(simTime, params.uwCaustics, params.uwCaustics > 0 ? 1 : 0, 0);
  uwPass.enabled = UW.mode !== 0;
  if (!uwPass.enabled) return;
  const q = params.quality ?? 2;
  const u = uwPostU;
  u.uProjInv.value.copy(camera.projectionMatrixInverse);
  u.uCamWorld.value.copy(camera.matrixWorld);
  u.uCamPos.value.copy(camera.position);
  u.uNear.value = camera.near; u.uFar.value = camera.far;
  u.uMode.value = UW.mode;
  u.uShaft.value = params.uwGodRays;
  u.uSteps.value = params.uwGodRays > 0 ? [8, 12, 20, 32][q] : 1;
  u.uMarch.value = [18, 24, 32, 40][q];
  u.uFrame.value = uwFrame = (uwFrame + 1) % 64;
  u.uWind.value.set(sea.toward[0], sea.toward[1]);
  // simple eye adaptation: looking up into Snell's window is far brighter than looking into the blue
  const fwdY = -camera.matrixWorld.elements[9];
  u.uExpo.value = 1.15 - 0.55 * smoothstep(-0.1, 0.7, fwdY);
  uwCollectOccluders();
  // particles
  const px = drawSize.y / (2 * Math.tan(camera.fov * D2R / 2));
  uwSnowDrift.x += sea.current.x * dt * 0.5; uwSnowDrift.z += sea.current.z * dt * 0.5;
  uwSnowGeo.setDrawRange(0, [1500, 3000, 5000, 7000][q]);
  for (const m of [uwSnowMat, uwBubMat]) {
    const mu = m.uniforms;
    mu.uRes.value.copy(drawSize); mu.uNear.value = camera.near; mu.uFar.value = camera.far;
    mu.uPx.value = px; mu.uSurfY.value = UW.camH;
  }
  uwSnowMat.uniforms.uDrift.value.copy(uwSnowDrift);
});

// ---- GUI
uwGui.add(params, 'uwAllowUnder').name('Camera can dive').onChange(uwApplyCameraLimits);
uwGui.add(params, 'uwSplit').name('Float at surface (split view)').listen();
uwGui.add(params, 'uwClarity', 0.3, 3, 0.01).name('Underwater clarity ×');
uwGui.add(params, 'uwGodRays', 0, 2, 0.01).name('God rays');
uwGui.add(params, 'uwCaustics', 0, 2, 0.01).name('Caustics on bodies');
uwGui.add(params, 'uwSnow').name('Marine snow');
uwGui.add(params, 'uwBubbles').name('Bubbles');
function uwView(kind) {
  params.uwAllowUnder = true;
  params.uwSplit = kind === 'split';
  uwApplyCameraLimits();
  const c = controls.target, a = Math.atan2(lightDir.z, lightDir.x) + Math.PI + 0.45;
  if (kind === 'below') camera.position.set(c.x + Math.cos(a) * 7, -4.5, c.z + Math.sin(a) * 7);
  else camera.position.set(c.x + Math.cos(a) * 9, UW.camH, c.z + Math.sin(a) * 9);
  camera.lookAt(c);
}
uwGui.add({ f: () => uwView('below') }, 'f').name('View from below');
uwGui.add({ f: () => uwView('split') }, 'f').name('Half over / half under');
HOOKS.ready.push(() => { window.sim.uw = { UW, uwU, uwPostU, uwPass, uwView }; });   // console access for testing
