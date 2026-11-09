// ============================================================================
// Sky, sun and moon state
// ============================================================================
const LAT = 32 * D2R, DECL = 12 * D2R;
const sunDir = new THREE.Vector3(), lightDir = new THREE.Vector3();
let skyDirty = true, lutDirty = true, envTimer = 0, nightF = 0, sunElDeg = 0;
const sunLight = new THREE.DirectionalLight(0xffffff, 3);
sunLight.layers.enable(PEOPLE_LAYER);       // lights are filtered by camera layers too (underwater pass)
scene.add(sunLight, sunLight.target);

function updateSky() {
  const H = (params.timeOfDay - 12) * 15 * D2R;
  const sinEl = Math.sin(LAT) * Math.sin(DECL) + Math.cos(LAT) * Math.cos(DECL) * Math.cos(H);
  const el = Math.asin(sinEl);
  const b = Math.atan2(Math.sin(H), Math.cos(H) * Math.sin(LAT) - Math.tan(DECL) * Math.cos(LAT)) + Math.PI;
  sunDir.set(Math.cos(el) * Math.sin(b), sinEl, -Math.cos(el) * Math.cos(b));
  sunElDeg = el * R2D;
  nightF = 1 - smoothstep(-0.12, -0.02, sinEl);
  // below about -3 deg the moon (opposite azimuth, 38 deg up) lights the scene instead
  const useMoon = sunElDeg < -3;
  if (useMoon) lightDir.set(-sunDir.x, 0, -sunDir.z).normalize().multiplyScalar(Math.cos(38 * D2R)).setY(Math.sin(38 * D2R));
  else lightDir.copy(sunDir);
  const W = useMoon ? 14 * 0.004 : 14;
  skyU.uSun.value.set(lightDir.x, lightDir.y, lightDir.z, W);
  skyU.uHaze.value = params.haze;
  skyU.uCloud.value.y = params.volClouds ? 0 : params.clouds;   // the flat low layer gives way to the volumetric clouds
  skyU.uVeil.value.x = params.veil;
  skyU.uNight.value = nightF;
  skyU.uWaterCol.value.set(oceanU.uWaterCol.value.x, oceanU.uWaterCol.value.y, oceanU.uWaterCol.value.z);
  if (lutDirty) {
    transMat.uniforms.uHaze.value = msMat.uniforms.uHaze.value = skyViewMat.uniforms.uHaze.value = params.haze;
    runPass(transMat, transRT);
    runPass(msMat, msRT);
    lutDirty = false;
  }
  skyViewMat.uniforms.uSunY.value = lightDir.y;
  runPass(skyViewMat, skyViewRT);
  compositePass.uniforms.uExposure.value = params.exposure * (1 + 3.0 * nightF);   // eye adaptation at night only
  const fe = oceanU.uFogExt.value;
  fe.set((5.802e-3 + 4.4e-3 * params.haze) * 1e-3 * params.fog, (13.558e-3 + 4.4e-3 * params.haze) * 1e-3 * params.fog,
    (33.1e-3 + 4.4e-3 * params.haze) * 1e-3 * params.fog, 1.1e-4 * params.fog);
  skyDirty = false;
  refreshEnv();
}
// sunlight at the sea surface + environment cube + image-based lighting for the people
function refreshEnv() {
  // cloud shadow on the sun and the reflection cube are taken at the orbit target, not the (moving) camera
  const camXZ = skyU.uCamXZ.value.clone();
  skyU.uCamXZ.value.set(controls.target.x, controls.target.z);
  runPass(sunEMat, sunERT);
  renderer.readRenderTargetPixels(sunERT, 0, 0, 1, 1, sunEBuf);
  const E = oceanU.uSunE.value.set(sunEBuf[0], sunEBuf[1], sunEBuf[2]);
  skyU.uSunE.value.copy(E);
  oceanU.uSunDir.value.copy(lightDir);
  const Emax = Math.max(E.x, E.y, E.z, 1e-6);
  sunLight.color.setRGB(E.x / Emax, E.y / Emax, E.z / Emax, THREE.LinearSRGBColorSpace);
  sunLight.intensity = Emax;
  sunLight.position.copy(lightDir).multiplyScalar(100);
  renderer.setRenderTarget(null);
  cubeCam.update(renderer, envScene);
  skyU.uCamXZ.value.copy(camXZ);
  if (pmremRT) pmremRT.dispose();
  pmremRT = pmrem.fromCubemap(envRT.texture);
  scene.environment = pmremRT.texture;
  envTimer = 0.5;
}

