// ============================================================================
// GUI
// ============================================================================
const gui = new GUI({ title: 'Ocean · MOB controls' });
const waves = () => { needSpectrum = true; };
const sky = () => { skyDirty = true; };
const fSky = gui.addFolder('Sky & light');
fSky.add(params, 'timeOfDay', 0, 24, 0.05).name('Time of day (h)').onChange(sky);
fSky.add(params, 'haze', 0.5, 15, 0.1).name('Haze').onChange(() => { lutDirty = true; sky(); });
fSky.add(params, 'clouds', 0, 1, 0.01).name('Cloud cover').onChange(sky);
fSky.add(params, 'veil', 0, 1, 0.01).name('High cloud veil').onChange(sky);
fSky.add(params, 'exposure', 0.1, 4, 0.01).name('Exposure').onChange(sky);
fSky.add(params, 'bloom', 0, 0.3, 0.001).name('Bloom').onChange(v => bloomPass.strength = v * 2);
fSky.add(params, 'fog', 0, 5, 0.01).name('Aerial haze ×').onChange(sky);
const fSea = gui.addFolder('Wind & waves');
fSea.add(params, 'windSpeed', 0.5, 25, 0.1).name('Wind speed (m/s)').onChange(waves);
fSea.add(params, 'windFrom', 0, 360, 1).name('Wind from (°)').onChange(waves);
fSea.add(params, 'fetchKm', 5, 800, 1).name('Fetch (km)').onChange(waves);
fSea.add(params, 'waveHeight', 0, 2.5, 0.01).name('Wave height ×').onChange(waves);
fSea.add(params, 'steepness', 0, 1.6, 0.01).name('Steepness (choppy disp.)').onChange(waves);
fSea.add(params, 'choppiness', 0, 1, 0.01).name('Choppiness (spread)').onChange(waves);
fSea.add(params, 'swellHeight', 0, 4, 0.05).name('Swell height (m)').onChange(waves);
fSea.add(params, 'swellFrom', 0, 360, 1).name('Swell from (°)').onChange(waves);
fSea.add(params, 'swellLength', 30, 300, 1).name('Swell wavelength (m)').onChange(waves);
fSea.add(params, 'foam', 0, 3, 0.01).name('Whitecaps ×');
const fWater = gui.addFolder('Water');
const setWater = () => {
  const c = new THREE.Color(params.waterColor), s = new THREE.Color(params.scatterColor), t = params.turbidity;
  oceanU.uWaterCol.value.set(c.r * t, c.g * t, c.b * t, 0.0024);
  oceanU.uSss.value.set(s.r, s.g, s.b, 2.6);
  oceanU.uAbsorb.value.set(0.6, 0.3, 0.32).multiplyScalar(0.6 + 0.4 * t);   // absorption + scattering loss
  skyU.uWaterCol.value.set(c.r * t, c.g * t, c.b * t);
};
fWater.addColor(params, 'waterColor').name('Water colour').onChange(setWater);
fWater.addColor(params, 'scatterColor').name('Scatter colour').onChange(setWater);
fWater.add(params, 'turbidity', 0.2, 3, 0.01).name('Turbidity').onChange(setWater);
const fPeople = gui.addFolder('People overboard');
fPeople.add(params, 'people', 0, MAX_PEOPLE, 1).name('Number in water').onChange(setPeopleCount);
fPeople.add(params, 'followGroup').name('Camera follows group');
fPeople.add(params, 'showProbes').name('Show buoyancy probes').onChange(v => people.forEach(P => P.probeGroup.visible = v));
fPeople.add({ respawn }, 'respawn').name('Respawn / regroup');
const fSim = gui.addFolder('Simulation & camera');
fSim.add(params, 'timeScale', 0, 3, 0.05).name('Time scale');
fSim.add(params, 'paused').name('Pause');
fSim.add(params, 'autoRotate').name('Auto-orbit').onChange(v => controls.autoRotate = v);
fSim.add({ reset: () => placeCamera() }, 'reset').name('Reset camera');
setWater();

// camera on the far side of the group from the sun, so the sun glitter runs towards the viewer
function placeCamera() {
  const c = groupCentroid(new THREE.Vector3());
  controls.target.copy(c);
  const ang = Math.atan2(lightDir.z, lightDir.x) + Math.PI + 0.45;
  camera.position.set(c.x + Math.cos(ang) * 16, 3.2, c.z + Math.sin(ang) * 16);
}

