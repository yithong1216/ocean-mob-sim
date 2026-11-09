// ============================================================================
// Sea condition: the 0-9 sea-state scale (Douglas / WMO code 3700; 中国海况等级 无浪 ... 怒涛)
// ============================================================================
// The scale is defined by the significant height of the wind sea (swell is graded separately), so:
//   - picking a level sets a fully developed wind sea of the middle height of its band: the wind from the
//     Pierson-Moskowitz relation Hs = 0.0248 U10^2 (which also matches the Beaufort force of each level), and
//     the fetch at which this simulator's JONSWAP spectrum reaches that height (gF/U^2 ~ 1.3e4, near full
//     development), whitecap coverage that follows the wind, and a moderate swell that grows with the level;
//   - the level shown is always read back from the wind-sea height of the spectrum actually in use, so changing
//     the wind in the professional panel, or a weather preset, moves it.
// Once a level is picked, weather presets keep the sea (wind and wave height) and only change the sky;
// moving the wind, fetch or wave-height sliders hands control back to them.
const SEA_LEVELS = [
  { name: 'Calm (glassy)', range: '0 m', hs: 0, swell: [0.2, 180],
    desc: 'The sea is like a mirror, or only swell is present.' },
  { name: 'Calm (rippled)', range: '0–0.1 m', hs: 0.06, swell: [0.2, 160],
    desc: 'Ripples, or ripples together with swell.' },
  { name: 'Smooth (wavelets)', range: '0.1–0.5 m', hs: 0.3, swell: [0.3, 150],
    desc: 'Small wavelets; crests begin to break, but the foam looks glassy rather than white.' },
  { name: 'Slight', range: '0.5–1.25 m', hs: 0.88, swell: [0.5, 140],
    desc: 'Waves are small but conspicuous; crests break, with white foam (whitecaps) in places.' },
  { name: 'Moderate', range: '1.25–2.5 m', hs: 1.9, swell: [0.8, 130],
    desc: 'Waves take a pronounced form, with whitecaps everywhere.' },
  { name: 'Rough', range: '2.5–4 m', hs: 3.25, swell: [1.2, 150],
    desc: 'Large crests form; foam covers much of them and the wind starts to blow it off the crests.' },
  { name: 'Very rough', range: '4–6 m', hs: 5, swell: [1.6, 170],
    desc: 'Foam blown off the crests begins to stretch in streaks down the wave faces; crests sometimes take the long shape of storm waves.' },
  { name: 'High', range: '6–9 m', hs: 7.5, swell: [2.0, 190],
    desc: 'Streaks of blown foam cover the wave faces and reach into some troughs; the crests are covered in foam.' },
  { name: 'Very high', range: '9–14 m', hs: 11.5, swell: [2.5, 220],
    desc: 'Dense foam covers the wave faces and the sea turns white; only parts of the troughs are free of foam.' },
  { name: 'Phenomenal', range: '> 14 m', hs: 16, swell: [3.0, 250],
    desc: 'The whole sea is covered in dense foam, the air is full of spray and visibility is much reduced.' },
];
const SEA_EDGES = [0.03, 0.1, 0.5, 1.25, 2.5, 4, 6, 9, 14];        // upper wind-sea Hs of levels 0..8 (m)
Object.assign(params, { seaState: 4, seaLock: false });

const seaLevelOf = hs => { let l = 0; while (l < SEA_EDGES.length && hs >= SEA_EDGES[l]) l++; return l; };
const seaLabel = l => `${l} · ${SEA_LEVELS[l].name} (${SEA_LEVELS[l].range})`;
const SEA_OPTIONS = Object.fromEntries(SEA_LEVELS.map((_, l) => [seaLabel(l), l]));
// wind-sea Hs of this simulator's spectrum (same JONSWAP integral as initSpectrum)
function seaWindHs(U, Fkm) {
  const F = Fkm * 1000, alpha = 0.076 * Math.pow(U * U / (F * G), 0.22), wp = 22 * Math.pow(G * G / (U * F), 1 / 3);
  let m0 = 0;
  for (let i = 1; i <= 800; i++) m0 += jonswapCPU(i * 0.01, wp, alpha, 3.3) * 0.01;
  return 4 * Math.sqrt(m0);
}
function seaApply(level) {
  const S = SEA_LEVELS[level];
  let U = 0.5, F = 1;
  if (S.hs > 0) {
    U = Math.round(Math.sqrt(S.hs / 0.0248) * 10) / 10;
    let lo = 0.5, hi = 3000;
    for (let i = 0; i < 50; i++) { const m = Math.sqrt(lo * hi); if (seaWindHs(U, m) < S.hs) lo = m; else hi = m; }
    F = Math.max(1, Math.round(lo));
  }
  // whitecaps: coverage grows ~U^3.4 (Monahan & O'Muircheartaigh 1980); the slider's 1 is tuned for 7 m/s
  const W = u => 3.84e-6 * Math.pow(u, 3.41);
  const foam = clamp(1 + 1.1 * Math.log10(W(U) / W(7)), 0, 3);
  Object.assign(params, {
    windSpeed: U, fetchKm: F, waveHeight: 1, swellHeight: S.swell[0], swellLength: S.swell[1], foam: Math.round(foam * 100) / 100,
    seaState: level, seaLock: true,
  });
  needSpectrum = true;
  gui.controllersRecursive().forEach(c => c.updateDisplay());
  seaInfoUpdate();
}
function seaStateText() {
  const l = params.seaState;
  return LANG === 'zh' ? `${l} 级 · ${i18n(SEA_LEVELS[l].name, 'sea')}` : `${l} · ${SEA_LEVELS[l].name}`;
}

// ---- info card: level, height band, the textbook description and the height now in the water
const seaInfoEls = [];
function seaInfoCard() {
  const el = document.createElement('div');
  el.className = 'sea-info';
  seaInfoEls.push(el);
  return el;
}
let seaInfoKey = '';
function seaInfoUpdate() {
  const l = params.seaState, S = SEA_LEVELS[l], hs = (sea.HsW ?? 0).toFixed(2), zh = LANG === 'zh';
  const key = `${LANG}|${l}|${hs}`;
  if (key === seaInfoKey) return;
  seaInfoKey = key;
  const head = zh ? `${l} 级 · ${i18n(S.name, 'sea')}` : `Level ${l} · ${S.name}`;
  const band = zh ? `浪高 ${S.range} · 当前风浪有效波高 ${hs} m` : `Wave height ${S.range} · wind sea now Hs ${hs} m`;
  for (const el of seaInfoEls) el.innerHTML = `<b>${head}</b><span>${band}</span><p>${i18n(S.desc)}</p>`;
}
const seaCss = document.createElement('style');
seaCss.textContent = `
.sea-info { margin: 2px 6px 6px; padding: 7px 9px; border-radius: 4px; background: rgba(120, 190, 215, 0.1);
  border-left: 3px solid rgba(127, 212, 255, 0.7); color: #dbe9ee; font-size: 11px; line-height: 1.45; }
.sea-info b { display: block; color: #fff; font-size: 12px; }
.sea-info span { display: block; color: #93aebb; }
.sea-info p { margin: 4px 0 0; }`;
document.head.appendChild(seaCss);

// ---- professional panel: first item in "Wind & waves"
const seaCtl = fSea.add(params, 'seaState', SEA_OPTIONS).name('Sea condition').onChange(seaApply).listen();
const seaCard = seaInfoCard();
fSea.$children.prepend(seaCtl.domElement, seaCard);
const _ctlOf = p => fSea.controllers.find(c => c.property === p);
_ctlOf('windSpeed').max(40);
_ctlOf('fetchKm').min(1).max(2000);
for (const p of ['windSpeed', 'fetchKm', 'waveHeight']) {       // a manual change hands the sea back to the sliders
  const c = _ctlOf(p), prev = c._onChange;
  c.onChange(function (v) { params.seaLock = false; if (prev) prev.call(this, v); });
}

// the level follows the wind sea actually simulated (after weather blends, slider moves ...)
let seaT = 0;
HOOKS.update.push((dt, dtReal) => {
  seaT -= dtReal;
  if (seaT > 0) return;
  seaT = 0.25;
  if (!params.seaLock && sea.HsW !== undefined) params.seaState = seaLevelOf(sea.HsW);
  seaInfoUpdate();
});
LANG_HOOKS.push(() => { seaInfoKey = ''; seaInfoUpdate(); });
HOOKS.ready.push(() => { window.sim.seaState = { levels: SEA_LEVELS, apply: seaApply, levelOf: seaLevelOf, windHs: seaWindHs }; });
