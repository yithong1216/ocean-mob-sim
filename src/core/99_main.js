// ============================================================================
// Main loop
// ============================================================================
const statsEl = document.getElementById('stats');
const BFT = [0.5, 1.6, 3.4, 5.5, 8, 10.8, 13.9, 17.2, 20.8, 24.5, 28.5, 32.7];
const COMPASS = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
let simTime = 0, physAcc = 0, hudT = 0;
function updateHud() {
  const U = params.windSpeed, bft = BFT.findIndex(v => U < v);
  const meanSub = people.length ? people.reduce((s, P) => s + P.sub, 0) / people.length : 0;
  const c = groupCentroid(new THREE.Vector3());
  const b = bft < 0 ? 12 : bft, k = s => `<span class="k">${i18n(s)}</span>`;
  statsEl.innerHTML =
    `${k('Sea state')} ${typeof seaStateText === 'function' ? seaStateText() + ' · ' : ''}Hs ${sea.Hs.toFixed(2)} m · Tp ${sea.Tp.toFixed(1)} s · λp ${sea.lp.toFixed(0)} m · ${LANG === 'zh' ? `蒲福 ${b} 级` : `Bft ${b}`}<br>` +
    `${k('Wind')} ${U.toFixed(1)} m/s ${i18n('from')} ${params.windFrom.toFixed(0)}° (${i18n(COMPASS[Math.round(params.windFrom / 45) % 8], 'compass')}) · ${k('drift')} ${Math.hypot(sea.current.x, sea.current.z).toFixed(2)} m/s<br>` +
    `${k('Sun')} ${sunElDeg.toFixed(1)}° · ${k('sim t')} ${simTime.toFixed(0)} s<br>` +
    `${k('In water')} ${people.length}${modelsReady ? '' : i18n(' (loading models…)')} · ${i18n('mean submerged')} ${(meanSub * 100).toFixed(0)}% · ${i18n('group')} (${c.x.toFixed(0)}, ${c.z.toFixed(0)}) m`;
}

const clock = new THREE.Clock();
const centroid = new THREE.Vector3(), prevTarget = new THREE.Vector3();
function update(dtReal) {
  if (skyDirty) updateSky();
  const dt = params.paused ? 0 : dtReal * params.timeScale;
  simTime += dt;
  oceanU.uFoamP.value.set(0.36 + 0.12 * params.foam, 2.5, 1, 1);

  simulateWaves(simTime);
  writeQueries();
  runProbes(simTime);

  // physics substeps on the sampled surface
  physAcc += dt;
  let steps = 0, tSub = simTime - physAcc;
  while (physAcc >= PHYS_DT && steps < 12) {
    people.forEach((P, i) => stepPerson(P, i, PHYS_DT, tSub));
    physAcc -= PHYS_DT; tSub += PHYS_DT; steps++;
  }
  if (steps === 12) physAcc = 0;

  // people visuals: pose (sculling arms), waterline plane for the wet shader, foam injection, strobes
  const fp = foamUpdMat.uniforms.uPeople.value;
  people.forEach((P, i) => {
    P.root.position.copy(P.pos); P.root.quaternion.copy(P.q);
    setAims(P, simTime); solvePose(P.rig);
    const sc = probe(i * PROBES_PER + PROBES_PER - 1);
    _n.set(-sc.sx, 1, -sc.sz).normalize();
    P.wetU.plane.value.set(_n.x, _n.y, _n.z, -(_n.x * P.pos.x + _n.y * sc.h + _n.z * P.pos.z));
    P.headBones[0].getWorldPosition(P.wetU.head.value).add(P.headBones[1].getWorldPosition(_v2)).multiplyScalar(0.5);
    // the waterline crosses the inclined body around the collar and chest (probes 1 and 2)
    const rel = Math.hypot(P.vel.x - sea.current.x, P.vel.z - sea.current.z);
    const pa = probe(i * PROBES_PER + 1), pb = probe(i * PROBES_PER + 2);
    fp[i].set((pa.x0 + pb.x0) / 2, (pa.z0 + pb.z0) / 2, 0.24, clamp(0.15 + rel * 0.8, 0, 0.45) * smoothstep(0.3, 0.6, P.sub));
    const flash = ((simTime * 0.9 + P.strobePh) % 1) < 0.08 ? 1 : 0;
    P.bulbMat.emissiveIntensity = flash * (3 + 400 * nightF);
    P.glow.material.opacity = flash * (0.1 + 0.9 * nightF);
    if (params.showProbes) P.probeMats.forEach((m, k) => m.color.setRGB(1 - P.frac[k], 0.25 + 0.6 * P.frac[k], P.frac[k]));
  });
  foamUpdMat.uniforms.uPeopleCount.value = people.length;
  if (dt > 0) updateFoam(dt);
  oceanU.uFoamWorld.value = foamSrc.texture;
  for (const h of HOOKS.update) h(dt, dtReal);

  // camera: a feature may drive it (route planner map, mission flight); otherwise follow the group, stay above the water
  if (cameraDriver) cameraDriver(dt, dtReal);
  else {
    if (params.followGroup && people.length) {
      prevTarget.copy(controls.target);
      groupCentroid(centroid);
      controls.target.lerp(centroid, 1 - Math.exp(-dtReal * 0.8));
      camera.position.add(prevTarget.sub(controls.target).negate());
    }
    controls.update();
    const hc = probeBuf[CAM_PROBE * 4];
    if (!params.uwAllowUnder && camera.position.y < hc + 0.6) camera.position.y = hc + 0.6;
  }

  oceanU.uCenter.value.set(camera.position.x, camera.position.z);
  oceanU.uTime.value = simTime;
  oceanU.uWind.value.set(sea.toward[0], sea.toward[1], params.windSpeed);
  skyU.uCloud.value.w = simTime;
  skyU.uCamXZ.value.set(camera.position.x, camera.position.z);
  skyU.uPixAng.value = 2 * Math.tan(camera.fov * D2R / 2) / drawSize.y;
  skyDome.position.copy(camera.position);
  envTimer -= dtReal;
  if (envTimer <= 0) refreshEnv();

  updateThermalEnv();
  updateSensorPose(dt);
  lastDtReal = dtReal;
  for (const h of HOOKS.lateUpdate) h(dt, dtReal);

  hudT -= dtReal;
  if (hudT <= 0) { updateHud(); updateSensorOSD(); hudT = 0.25; }
}
let lastDtReal = 1 / 60;
function render() {
  for (const h of HOOKS.beforeRender) h();
  // submerged bodies -> refraction target (people layer only, transparent background)
  camera.layers.set(PEOPLE_LAYER);
  renderer.setRenderTarget(refrRT);
  renderer.setClearColor(0x000000, 0);
  renderer.clear();
  renderer.render(scene, camera);
  renderer.setClearColor(0x000000, 1);
  camera.layers.set(0);
  renderer.setRenderTarget(null);
  composer.render();
  renderSensors(lastDtReal);
  for (const h of HOOKS.afterRender) h();
}
let stepLimit = Infinity;        // the data recorder caps the step so that no capture is skipped (0 = hold)
function frame() {
  update(Math.min(clock.getDelta(), 0.05, stepLimit));
  render();
  requestAnimationFrame(frame);
}
function onResize() {
  camera.aspect = window.innerWidth / window.innerHeight; camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.getDrawingBufferSize(drawSize);
  composer.setPixelRatio(1); composer.setSize(drawSize.x, drawSize.y);
  refrRT.setSize(drawSize.x, drawSize.y);
  oceanU.uResolution.value.copy(drawSize);
  compositePass.uniforms.uAspect.value = drawSize.x / drawSize.y;
  for (const h of HOOKS.resize) h(drawSize.x, drawSize.y);
}
window.addEventListener('resize', onResize);
onResize();

updateSky();
placeCamera();
const loader = new GLTFLoader();
Promise.allSettled(MODEL_DEFS.map(d => loader.loadAsync(MODEL_BASE + d.file))).then(res => {
  res.forEach((r, i) => {
    if (r.status === 'fulfilled') templates.push(prepareTemplate(r.value, MODEL_DEFS[i]));
    else console.warn('model failed to load', MODEL_DEFS[i].file, r.reason);
  });
  if (!templates.length) throw new Error(i18n('Could not load the human models from the CDN (check the network connection).'));
  modelsReady = true;
  setPeopleCount(params.people);
  placeCamera();
});

// console hook for scripted testing: advance n frames of dt seconds, then draw once
const tick = (n = 1, dt = 1 / 60) => { for (let i = 0; i < n; i++) update(dt); render(); };
window.sim = { params, people, sea, oceanU, skyU, camera, controls, gui, tick, probe, setPeopleCount, templates, renderer, ocean, finalRT, h0RT, probeBuf, sensorCam, agc, THERM, irRT, HOOKS, get modelsReady() { return modelsReady; }, get stepLimit() { return stepLimit; } };
for (const h of HOOKS.ready) h();
requestAnimationFrame(frame);
