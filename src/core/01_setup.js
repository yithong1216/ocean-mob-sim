// ============================================================================
// Constants and parameters
// ============================================================================
const G = 9.81, RHO = 1025, RHO_AIR = 1.225;
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const N = 256;                                         // FFT size per cascade
const CASC_L = [1103.0, 211.3, 39.7, 7.43, 1.393];     // cascade patch sizes (m)
const NC = CASC_L.length;
const LOOP = 600;                                      // spectrum time loop (s)
const FOAM_RES = 1024, FOAM_SIZE = 128;                // world foam map: 12.5 cm texels
const MAX_PEOPLE = 20, PROBES_PER = 12, MAX_PROBES = 256, CAM_PROBE = 255;
const PEOPLE_LAYER = 1;
const PHYS_DT = 1 / 120;
const MODEL_BASE = 'https://cdn.jsdelivr.net/gh/mrdoob/three.js@r169/examples/models/gltf/';

const params = {
  timeOfDay: 18.1, haze: 3.0, clouds: 0.4, veil: 0.35, exposure: 1.35, bloom: 0.1, fog: 1.0,
  windSpeed: 7, windFrom: 300, fetchKm: 90, waveHeight: 1.0, steepness: 1.25, choppiness: 0.5,
  swellHeight: 1.6, swellFrom: 275, swellLength: 120, foam: 1.0,
  waterColor: '#2c646d', scatterColor: '#356152', turbidity: 1.0,
  people: 6, followGroup: true, showProbes: false,
  timeScale: 1, paused: false, autoRotate: false,
  // drone EO/IR gimbal (both windows share one co-registered pose)
  sensors: true, sensorSource: 0, droneAlt: 45, droneRange: 85, sensorFov: 13, droneOrbit: 1.5, palette: 0, irNoise: 1,
  waterTempC: 18, airTempC: 20, humidity: 0.75,
};

const clamp = (x, a, b) => Math.min(b, Math.max(a, x));
const smoothstep = (a, b, x) => { const t = clamp((x - a) / (b - a), 0, 1); return t * t * (3 - 2 * t); };
const bearingVec = deg => [Math.sin(deg * D2R), -Math.cos(deg * D2R)];   // compass -> (x, z); -z = north
const rand = (a, b) => a + Math.random() * (b - a);
const pick = a => a[Math.floor(Math.random() * a.length)];

// ============================================================================
// Extension points for the feature modules in src/features/ (see README.md)
// ============================================================================
// Feature files are concatenated after src/core/00..09 and before src/core/99_main.js, so they can use every
// core object at load time, register callbacks here, and edit shader strings via the "// @hook:..." markers.
let cameraDriver = null;        // fn(dt, dtReal) that poses the main camera instead of OrbitControls (route planner / flight)
const HOOKS = {
  update: [],        // fn(dt, dtReal)          after the core simulation/people update, before camera + uniform sync
  lateUpdate: [],    // fn(dt, dtReal)          after camera + uniform sync (camera pose is final for this frame)
  beforeRender: [],  // fn()                    before the main view is rendered (extra passes, shadow maps ...)
  afterRender: [],   // fn()                    after the main view and the sensor windows
  resize: [],        // fn(width, height)       drawing-buffer size in pixels
  personShader: [],  // fn(shader, material, thermalClass)  inside every person/lifejacket onBeforeCompile
  ready: [],         // fn()                    once, after startup (window.sim exists)
};
// insert GLSL on the lines after a "// @hook:name" marker line of a ShaderMaterial (vertex or fragment)
function injectShader(material, hook, code, stage = 'fragment') {
  const key = stage === 'vertex' ? 'vertexShader' : 'fragmentShader';
  const src = material[key], at = src.indexOf('// @hook:' + hook);
  if (at < 0) throw new Error('shader hook not found: ' + hook);
  const eol = src.indexOf('\n', at);
  const end = eol < 0 ? src.length : eol;
  material[key] = src.slice(0, end) + '\n' + code + '\n' + src.slice(end);
  material.needsUpdate = true;
}

// ============================================================================
// Renderer, camera, post-processing
// ============================================================================
// The ocean binds 5 vertex + 15 fragment samplers: within each stage limit, but three.js checks the sum against
// the per-stage limit (and warns every frame). Texture units are one combined pool, so report the combined limit.
const glCanvas = document.createElement('canvas');
const glCtx = glCanvas.getContext('webgl2', { antialias: false, powerPreference: 'high-performance' });
{
  const getParameter = glCtx.getParameter.bind(glCtx);
  glCtx.getParameter = p => p === glCtx.MAX_TEXTURE_IMAGE_UNITS ? Math.min(getParameter(glCtx.MAX_COMBINED_TEXTURE_IMAGE_UNITS), 32) : getParameter(p);
}
const renderer = new THREE.WebGLRenderer({ canvas: glCanvas, context: glCtx, antialias: false });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.toneMapping = THREE.NoToneMapping;      // tone mapping happens in the final composite pass
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.1, 60000);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true; controls.dampingFactor = 0.08;
controls.maxPolarAngle = Math.PI * 0.5; controls.minDistance = 1.5; controls.maxDistance = 800;
controls.autoRotateSpeed = 0.4;

const drawSize = new THREE.Vector2();
renderer.getDrawingBufferSize(drawSize);
// the scene depth is resolved into a DepthTexture for the post effects; each composer target needs its own one
// (RenderTarget.clone() would share a single GPU depth texture between the two)
const composer = new EffectComposer(renderer, new THREE.WebGLRenderTarget(drawSize.x, drawSize.y,
  { type: THREE.HalfFloatType, samples: 4, depthTexture: new THREE.DepthTexture(drawSize.x, drawSize.y) }));
composer.renderTarget2.depthTexture = new THREE.DepthTexture(drawSize.x, drawSize.y);
composer.addPass(new RenderPass(scene, camera));
// basic bloom + composite; src/features/cine_post.js replaces the bloom and extends the composite when present
const bloomPass = new UnrealBloomPass(new THREE.Vector2(drawSize.x, drawSize.y), params.bloom * 2, 0.6, 4.0);
// feed the bloom a luminance-capped image so the sun disc and glints glow without washing out the frame
bloomPass.materialHighPassFilter.fragmentShader = bloomPass.materialHighPassFilter.fragmentShader.replace(
  'gl_FragColor = mix( outputColor, texel, alpha );',
  'gl_FragColor = mix( outputColor, vec4( texel.rgb * min( 1.0, 30.0 / max( v, 1e-4 ) ), texel.a ), alpha );');
bloomPass.materialHighPassFilter.needsUpdate = true;
composer.addPass(bloomPass);
// final composite: exposure, vignette, slight cool white balance, highlight desaturation, AgX, dither
const compositePass = new ShaderPass({
  uniforms: { tDiffuse: { value: null }, uExposure: { value: params.exposure }, uFrame: { value: 0 }, uAspect: { value: 1 } },
  vertexShader: 'varying vec2 vUv; void main() { vUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }',
  fragmentShader: /* glsl */`
uniform sampler2D tDiffuse;
uniform float uExposure, uFrame, uAspect;
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
  float l = dot(v, vec3(0.2126, 0.7152, 0.0722));
  return l + sat * (pow(max(v, vec3(0.0)), vec3(power)) - l);
}
float ign(vec2 p) { return fract(52.9829189 * fract(dot(p, vec2(0.06711056, 0.00583715)))); }
void main() {
  vec3 c = texture2D(tDiffuse, vUv).rgb * uExposure;
  vec2 d = (vUv - 0.5) * vec2(uAspect, 1.0);
  c *= 1.0 - 0.10 * dot(d, d);
  c *= vec3(0.81, 1.0, 1.19);
  float lum = dot(c, vec3(0.2126, 0.7152, 0.0722));
  c = mix(c, vec3(lum), smoothstep(0.7, 6.0, lum) * 0.55);
  vec3 o = agxEotf(agxLook(agx(c), 1.05, 1.06));
  o += (ign(gl_FragCoord.xy + uFrame * 5.588238) + ign(gl_FragCoord.xy * 1.37 + 17.0 + uFrame * 3.1) - 1.0) / 255.0;
  gl_FragColor = vec4(clamp(o, 0.0, 1.0), 1.0);
}`,
});
composer.addPass(compositePass);

// Submerged bodies are rendered first into their own target so the water can show them through the surface
const refrRT = new THREE.WebGLRenderTarget(drawSize.x, drawSize.y, {
  type: THREE.HalfFloatType, depthTexture: new THREE.DepthTexture(drawSize.x, drawSize.y),
});

// ---- full-screen GPU passes
const fsCam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
const fsScene = new THREE.Scene();
const fsQuad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2));
fsQuad.frustumCulled = false; fsScene.add(fsQuad);
function runPass(mat, target) { fsQuad.material = mat; renderer.setRenderTarget(target); renderer.render(fsScene, fsCam); }
const FS_VERT = 'void main() { gl_Position = vec4(position.xy, 0.0, 1.0); }';
const passMat = (frag, uniforms, defines = {}) => new THREE.ShaderMaterial({
  glslVersion: THREE.GLSL3, vertexShader: FS_VERT, fragmentShader: frag, uniforms, defines, depthTest: false, depthWrite: false,
});
const target = (w, h, o = {}) => new THREE.WebGLRenderTarget(w, h, {
  type: THREE.HalfFloatType, depthBuffer: false, minFilter: THREE.LinearFilter, magFilter: THREE.LinearFilter, generateMipmaps: false, ...o,
});

