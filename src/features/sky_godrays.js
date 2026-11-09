// ============================================================================
// Crepuscular rays / god rays (feature module "sky")
// ============================================================================
// Screen-space light shafts: a bright-pass of the HDR frame restricted to the sky (sun disc, sunlit cloud edges and
// the gaps between clouds; people and the sea occlude it), radially blurred toward the sun at 1/2..1/4 resolution in
// two passes, then added back tinted by the sunlight. Inserted right after the RenderPass, so bloom, the cinematic
// post chain and tone mapping all see the shafts.
import { Pass } from 'three/addons/postprocessing/Pass.js';

Object.assign(params, { godRays: true, godRayStrength: 1.0 });

const SKY_GR_MASK_FRAG = /* glsl */`
uniform sampler2D tScene, tCloud;
uniform mat4 uInvVP;
uniform vec2 uRes, uSrcTexel;
uniform vec3 uSunDir;
uniform float uLvl, uCloudOn;
uniform vec4 uSeaFog;
layout(location = 0) out vec4 fragColor;
void main() {
  vec2 uv = gl_FragCoord.xy / uRes;
  vec3 c = 0.25 * (texture(tScene, uv + uSrcTexel * vec2(-0.7, -0.7)).rgb + texture(tScene, uv + uSrcTexel * vec2(0.7, -0.7)).rgb
                 + texture(tScene, uv + uSrcTexel * vec2(-0.7, 0.7)).rgb + texture(tScene, uv + uSrcTexel * vec2(0.7, 0.7)).rgb);
  vec4 pf = uInvVP * vec4(uv * 2.0 - 1.0, 1.0, 1.0);
  vec3 rd = normalize(pf.xyz / pf.w);
  float sky = smoothstep(-0.002, 0.015, rd.y);
  // light gets through where the sky is open: cloud transmittance, sea fog, and no dark silhouette (people) in front
  float T = uCloudOn > 0.5 ? texture(tCloud, uv).a : 1.0;
  float fy = max(rd.y, 1e-3);
  T *= exp(-uSeaFog.x * uSeaFog.y / fy);
  float l = dot(c, vec3(0.2126, 0.7152, 0.0722));
  float occ = smoothstep(0.08 * uLvl, 0.35 * uLvl, l);
  float ca = max(dot(rd, uSunDir), 0.0);
  float w = 0.35 * pow(ca, 16.0) + pow(ca, 300.0);
  fragColor = vec4(vec3(T * sky * occ * w), 1.0);
}`;
const SKY_GR_BLUR_FRAG = /* glsl */`
uniform sampler2D tMask;
uniform vec2 uRes, uSunUV;
uniform float uLen, uDecay, uFrame;
layout(location = 0) out vec4 fragColor;
float ign(vec2 p) { return fract(52.9829189 * fract(dot(p, vec2(0.06711056, 0.00583715)))); }
void main() {
  vec2 uv = gl_FragCoord.xy / uRes;
  const int N = 40;
  vec2 delta = (uv - uSunUV) * uLen / float(N);
  vec2 p = uv - delta * ign(gl_FragCoord.xy + uFrame * 5.588);
  vec3 acc = vec3(0.0);
  float dec = 1.0, ws = 0.0;
  for (int i = 0; i < N; i++) {
    acc += texture(tMask, p).rgb * dec;
    ws += dec; dec *= uDecay; p -= delta;
  }
  fragColor = vec4(acc / ws, 1.0);
}`;
const SKY_GR_COMP_FRAG = /* glsl */`
uniform sampler2D tScene, tRays;
uniform mat4 uInvVP;
uniform vec2 uRes, uSunUV;
uniform vec3 uTint;
uniform float uAspect, uStrength;
layout(location = 0) out vec4 fragColor;
void main() {
  vec2 uv = gl_FragCoord.xy / uRes;
  vec4 c = texture(tScene, uv);
  vec4 pf = uInvVP * vec4(uv * 2.0 - 1.0, 1.0, 1.0);
  float y = normalize(pf.xyz / pf.w).y;
  float d = length((uv - uSunUV) * vec2(uAspect, 1.0));
  vec3 r = texture(tRays, uv).rgb * uTint * uStrength / (1.0 + d * d * 2.0) * mix(0.35, 1.0, smoothstep(-0.08, 0.02, y));
  fragColor = vec4(c.rgb + r, c.a);
}`;

class SkyGodRaysPass extends Pass {
  constructor() {
    super();
    this.needsSwap = true;
    this.rtA = target(16, 16); this.rtB = target(16, 16);
    this.mask = passMat(SKY_GR_MASK_FRAG, {
      tScene: { value: null }, tCloud: skyU.uCloudScr, uCloudOn: skyU.uCloudOn, uSeaFog: skyU.uSeaFog,
      uInvVP: { value: new THREE.Matrix4() }, uRes: { value: new THREE.Vector2() }, uSrcTexel: { value: new THREE.Vector2() },
      uSunUV: { value: new THREE.Vector2() }, uAspect: { value: 1 }, uSunDir: { value: new THREE.Vector3() }, uLvl: { value: 1 },
    });
    this.Esun = new THREE.Vector3(1, 1, 1);
    this.blur = passMat(SKY_GR_BLUR_FRAG, {
      tMask: { value: null }, uRes: { value: new THREE.Vector2() }, uSunUV: this.mask.uniforms.uSunUV,
      uLen: { value: 1 }, uDecay: { value: 0.97 }, uFrame: { value: 0 },
    });
    this.comp = passMat(SKY_GR_COMP_FRAG, {
      tScene: { value: null }, tRays: { value: null }, uRes: { value: new THREE.Vector2() }, uSunUV: this.mask.uniforms.uSunUV,
      uTint: { value: new THREE.Vector3(1, 1, 1) }, uAspect: this.mask.uniforms.uAspect, uStrength: { value: 0 },
      uInvVP: this.mask.uniforms.uInvVP,
    });
    this.vp = new THREE.Matrix4(); this.sun = new THREE.Vector4(); this.frame = 0; this.div = 0;
  }
  setSize(w, h) {
    const div = (params.quality ?? 2) >= 2 ? 2 : 4;
    this.div = div; this.w = w; this.h = h;
    this.rtA.setSize(Math.max(8, Math.ceil(w / div)), Math.max(8, Math.ceil(h / div)));
    this.rtB.setSize(this.rtA.width, this.rtA.height);
  }
  // sun position on screen and how strongly the shafts should show this frame (0 = skip the pass)
  update() {
    camera.updateMatrixWorld();
    this.vp.copy(camera.matrixWorldInverse).setPosition(0, 0, 0).premultiply(camera.projectionMatrix);
    const s = this.sun.set(lightDir.x, lightDir.y, lightDir.z, 0).applyMatrix4(this.vp);
    if (s.w <= 1e-4) return 0;
    const u = s.x / s.w * 0.5 + 0.5, v = s.y / s.w * 0.5 + 0.5;
    this.mask.uniforms.uSunUV.value.set(u, v);
    const off = Math.max(Math.abs(u - 0.5), Math.abs(v - 0.5)) - 0.5;      // > 0: sun outside the frame
    // sunlight above the clouds (the readback divided by its cloud visibility)
    if (sunEBuf[3] > 1e-3) this.Esun.set(sunEBuf[0], sunEBuf[1], sunEBuf[2]).divideScalar(sunEBuf[3]);
    const el = smoothstep(-0.03, 0.04, lightDir.y);
    return params.godRayStrength * (1 - smoothstep(0, 0.6, off)) * el * (1 - 0.9 * nightF);
  }
  render(renderer, writeBuffer, readBuffer) {
    const k = this.update();
    if ((params.quality ?? 2) >= 2 ? this.div !== 2 : this.div !== 4) this.setSize(this.w, this.h);
    const out = this.renderToScreen ? null : writeBuffer;
    this.needsSwap = k > 1e-3 || this.renderToScreen;
    if (!this.needsSwap) return;             // nothing to add this frame: leave the frame untouched
    this.frame++;
    const mu = this.mask.uniforms, W = this.rtA.width, H = this.rtA.height;
    mu.tScene.value = readBuffer.texture;
    mu.uInvVP.value.copy(this.vp).invert();
    mu.uRes.value.set(W, H);
    mu.uSrcTexel.value.set(1 / this.w, 1 / this.h);
    mu.uAspect.value = this.w / this.h;
    mu.uSunDir.value.copy(lightDir);
    mu.uLvl.value = Math.max(skyU.uSun.value.w / 14, 1e-3);
    runPass(this.mask, this.rtA);
    const bu = this.blur.uniforms;
    bu.uRes.value.set(W, H); bu.uFrame.value = this.frame % 64;
    bu.tMask.value = this.rtA.texture; bu.uLen.value = 1.0; bu.uDecay.value = 0.975;
    runPass(this.blur, this.rtB);
    bu.tMask.value = this.rtB.texture; bu.uLen.value = 0.12; bu.uDecay.value = 1.0;
    runPass(this.blur, this.rtA);
    const cu = this.comp.uniforms;
    cu.tScene.value = readBuffer.texture; cu.tRays.value = this.rtA.texture;
    cu.uRes.value.set(this.w, this.h);
    // in-scattered sunlight along the shafts: grows with the haze (Mie) and the sun's strength above the clouds
    cu.uTint.value.copy(this.Esun);
    cu.uStrength.value = 0.12 * k * (0.5 + 0.12 * params.haze);
    runPass(this.comp, out);
  }
  dispose() { this.rtA.dispose(); this.rtB.dispose(); this.mask.dispose(); this.blur.dispose(); this.comp.dispose(); }
}
const skyGodRays = new SkyGodRaysPass();
skyGodRays.setSize(drawSize.x, drawSize.y);
skyGodRays.enabled = params.godRays;
composer.insertPass(skyGodRays, 1);

const fRays = gui.addFolder('God rays');
fRays.add(params, 'godRays').name('Crepuscular rays').onChange(v => { skyGodRays.enabled = v; });
fRays.add(params, 'godRayStrength', 0, 3, 0.01).name('Strength');
fRays.close();
