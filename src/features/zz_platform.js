// ============================================================================
// Platform: quality presets, automatic render resolution, FPS readout, loading overlay
// ============================================================================
// params.quality (0 low ... 3 ultra) is read by the feature modules to scale their cost.
const QUALITY = [
  { name: 'Low', maxPixelRatio: 0.75 },
  { name: 'Medium', maxPixelRatio: 1.0 },
  { name: 'High', maxPixelRatio: 1.5 },
  { name: 'Ultra', maxPixelRatio: 2.0 },
];
Object.assign(params, { quality: 2, autoResolution: true, renderScale: 1 });

function applyQuality() {
  const q = QUALITY[params.quality];
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, q.maxPixelRatio) * params.renderScale);
  window.dispatchEvent(new Event('resize'));          // onResize() re-sizes the composer and the screen-space targets
}

const fPlatform = gui.addFolder('Performance');
fPlatform.add(params, 'quality', { Low: 0, Medium: 1, High: 2, Ultra: 3 }).name('Quality').onChange(() => { params.renderScale = 1; applyQuality(); });
fPlatform.add(params, 'autoResolution').name('Auto resolution (hold 55+ fps)');
const renderScaleCtl = fPlatform.add(params, 'renderScale', 0.5, 1, 0.05).name('Render scale').onChange(applyQuality);
fPlatform.close();

// ---- FPS readout under the HUD
const fpsEl = document.createElement('div');
fpsEl.style.cssText = 'position:fixed;left:12px;top:0;margin-top:0;color:#93aebb;font:11px Consolas,monospace;pointer-events:none;text-shadow:0 0 3px #000';
document.body.appendChild(fpsEl);

// ---- dynamic resolution: frame time is smoothed; the render scale steps down quickly and recovers slowly
const perf = { ema: 1 / 60, cooldown: 2, shown: 0 };
HOOKS.update.push((dt, dtReal) => {
  perf.ema += (dtReal - perf.ema) * 0.05;
  perf.cooldown -= dtReal;
  perf.shown -= dtReal;
  const fps = 1 / Math.max(perf.ema, 1e-4);
  if (perf.shown <= 0) {
    const hud = document.getElementById('hud');
    fpsEl.style.top = (hud.offsetTop + hud.offsetHeight + 6) + 'px';
    fpsEl.textContent = `${fps.toFixed(0)} fps · ${i18n(QUALITY[params.quality].name)} · ${(renderer.getPixelRatio() * window.innerWidth).toFixed(0)}×${(renderer.getPixelRatio() * window.innerHeight).toFixed(0)}`;
    perf.shown = 0.5;
  }
  if (!params.autoResolution || perf.cooldown > 0 || document.hidden) return;
  if (fps < 50 && params.renderScale > 0.55) { params.renderScale = Math.max(0.5, params.renderScale - 0.1); perf.cooldown = 1.5; }
  else if (fps > 58 && params.renderScale < 1) { params.renderScale = Math.min(1, params.renderScale + 0.05); perf.cooldown = 4; }
  else return;
  renderScaleCtl.updateDisplay();
  applyQuality();
});

// ---- loading overlay until the human models have arrived from the CDN
const loadingEl = document.createElement('div');
loadingEl.style.cssText = 'position:fixed;inset:0;display:flex;align-items:center;justify-content:center;pointer-events:none;' +
  'color:#e6f0f4;font:14px "Segoe UI",system-ui,sans-serif;letter-spacing:.04em;text-shadow:0 0 8px #000;transition:opacity .8s';
loadingEl.textContent = i18n('Loading ocean and people…');
LANG_HOOKS.push(() => { loadingEl.textContent = i18n('Loading ocean and people…'); });
document.body.appendChild(loadingEl);
HOOKS.update.push(() => {
  if (modelsReady && loadingEl.isConnected) loadingEl.remove();
});

HOOKS.ready.push(() => {
  applyQuality();
  gui.folders.forEach((f, i) => { if (i > 0) f.close(); });   // many feature folders: start collapsed
  Object.assign(window.sim, { applyQuality, QUALITY });
});
