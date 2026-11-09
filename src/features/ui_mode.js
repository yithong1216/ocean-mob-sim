// ============================================================================
// Simple / professional interface
// ============================================================================
// Simple: one small panel with the four things a visitor changes: sea condition, weather, time of day and the
// number of people in the water. Professional: the full control panel. Both panels edit the same params, and a
// change in the simple panel runs the professional control's own handlers, so the two always agree.
// The choice is remembered per browser; a first visit opens the simple panel.
const UI_KEY = 'oceanMob.ui';
let UI_MODE = (() => { try { const s = localStorage.getItem(UI_KEY); if (s === 'simple' || s === 'pro') return s; } catch (e) { /* blocked */ } return 'simple'; })();

const simpleGui = new GUI({ title: 'Quick controls', width: 285 });
I18N_GUIS.push(simpleGui);
const _proCtl = (prop, folder) => gui.controllersRecursive().find(c => c.property === prop && (!folder || c.parent === folder));
// mirror one professional control: same object and property, its handlers run on every change
function simpleMirror(pro, add) {
  const c = add(simpleGui).name(pro._name);
  c.onChange(() => { pro._callOnChange(); pro.updateDisplay(); });
  c.onFinishChange(() => { pro._callOnFinishChange(); });
  return c.listen();
}
simpleMirror(seaCtl, g => g.add(params, 'seaState', SEA_OPTIONS));
simpleGui.$children.appendChild(seaInfoCard());
const proWeather = _proCtl('weather');
simpleMirror(proWeather, g => g.add(params, 'weather', Object.keys(SKY_PRESETS))).name('Weather');
const proTime = _proCtl('timeOfDay');
simpleMirror(proTime, g => g.add(params, 'timeOfDay', 0, 24, 0.05));
const proPeople = _proCtl('people');
simpleMirror(proPeople, g => g.add(params, 'people', 0, MAX_PEOPLE, 1));

// ---- the switches at the top right: [Simple | Pro]  [EN | 中文]
const uiSwitches = document.createElement('div');
uiSwitches.id = 'uiSwitches';
const modeEl = document.createElement('div');
modeEl.className = 'seg';
modeEl.setAttribute('role', 'group');
modeEl.innerHTML = '<button type="button" data-m="simple"></button><button type="button" data-m="pro"></button>';
uiSwitches.appendChild(modeEl);
document.body.appendChild(uiSwitches);
const uiCss = document.createElement('style');
uiCss.textContent = `
#uiSwitches { position: fixed; top: 6px; z-index: 1002; display: flex; gap: 6px; }
#uiSwitches .seg { display: flex; overflow: hidden; border-radius: 6px; border: 1px solid rgba(190, 215, 225, 0.45);
  background: rgba(5, 14, 22, 0.6); backdrop-filter: blur(6px); box-shadow: 0 2px 10px rgba(0, 0, 0, 0.35);
  font: 12px/1 "Segoe UI", system-ui, "Microsoft YaHei", "PingFang SC", sans-serif; }
#uiSwitches button { all: unset; cursor: pointer; padding: 6px 11px; color: #93aebb; letter-spacing: 0.02em; }
#uiSwitches button + button { border-left: 1px solid rgba(190, 215, 225, 0.25); }
#uiSwitches button:hover { color: #cfe3ea; background: rgba(120, 190, 215, 0.12); }
#uiSwitches button.on { background: rgba(64, 160, 200, 0.6); color: #fff; font-weight: 600; }
#uiSwitches button:focus-visible { outline: 2px solid #7fd4ff; outline-offset: -2px; }`;
document.head.appendChild(uiCss);

function placeUiSwitches() {
  const panel = UI_MODE === 'simple' ? simpleGui : gui;
  const r = panel.domElement.getBoundingClientRect(), w = uiSwitches.offsetWidth || 180;
  uiSwitches.style.right = Math.max(6, Math.min(window.innerWidth - r.left + 8, window.innerWidth - w - 6)) + 'px';
}
function uiModeLabels() {
  const [s, p] = modeEl.querySelectorAll('button');
  s.textContent = i18n('Simple'); p.textContent = i18n('Professional');
  modeEl.setAttribute('aria-label', i18n('Interface'));
}
function setUiMode(m) {
  UI_MODE = m === 'pro' ? 'pro' : 'simple';
  try { localStorage.setItem(UI_KEY, UI_MODE); } catch (e) { /* the choice lasts for this visit */ }
  if (UI_MODE === 'simple') { gui.hide(); simpleGui.show(); } else {
    simpleGui.hide(); gui.show();
    gui.controllersRecursive().forEach(c => c.updateDisplay());   // values changed while it was hidden
  }
  modeEl.querySelectorAll('button').forEach(b => {
    const on = b.dataset.m === UI_MODE;
    b.classList.toggle('on', on);
    b.setAttribute('aria-pressed', on ? 'true' : 'false');
  });
  placeUiSwitches();
}
modeEl.addEventListener('click', e => {
  const b = e.target.closest('button');
  if (b && b.dataset.m !== UI_MODE) setUiMode(b.dataset.m);
});
window.addEventListener('resize', placeUiSwitches);
LANG_HOOKS.push(() => { uiModeLabels(); placeUiSwitches(); });
uiModeLabels();
setUiMode(UI_MODE);
HOOKS.ready.push(() => {
  setUiMode(UI_MODE);
  window.sim.setUiMode = setUiMode;
  window.sim.simpleGui = simpleGui;
  Object.defineProperty(window.sim, 'uiMode', { get: () => UI_MODE, enumerable: true });
});
