// ============================================================================
// Language switch: EN / 中文 at the top right, next to the Simple / Professional switch (ui_mode.js)
// ============================================================================
// The control panels are translated in place: folder titles, control labels and dropdown option names (the option
// values stay the same, so nothing that reads params changes). The English originals are kept on each item.
const langEl = document.createElement('div');
langEl.id = 'langSwitch';
langEl.className = 'seg';
langEl.setAttribute('role', 'group');
langEl.setAttribute('aria-label', 'Language / 语言');
langEl.innerHTML = '<button type="button" data-l="en" lang="en">EN</button><button type="button" data-l="zh" lang="zh-CN">中文</button>';
uiSwitches.appendChild(langEl);

function translateGUI() {
  for (const root of [gui, ...I18N_GUIS]) {
    for (const f of [root, ...root.foldersRecursive()]) {
      if (f._enTitle === undefined) f._enTitle = f._title;
      f.title(i18n(f._enTitle));
    }
    for (const c of root.controllersRecursive()) {
      if (c._enName === undefined) c._enName = c._name;
      c.name(i18n(c._enName, c.parent._enTitle));
      if (c._names) {                                 // dropdown: new display names, same values
        if (c._enNames === undefined) { c._enNames = [...c._names]; c._enValues = [...c._values]; }
        const o = {};
        c._enNames.forEach((n, i) => { o[i18n(n, 'opt:' + c._enName)] = c._enValues[i]; });
        if (Object.keys(o).length === c._enNames.length) c.options(o);
      }
    }
  }
}
function setLanguage(l) {
  LANG = l === 'zh' ? 'zh' : 'en';
  try { localStorage.setItem(LANG_KEY, LANG); } catch (e) { /* storage blocked: the choice lasts for this visit */ }
  document.documentElement.lang = LANG === 'zh' ? 'zh-CN' : 'en';
  document.title = i18n('Ocean MOB Simulator');
  document.querySelectorAll('[data-i18n]').forEach(el => { el.textContent = i18n(el.dataset.i18n); });
  translateGUI();
  langEl.querySelectorAll('button').forEach(b => {
    const on = b.dataset.l === LANG;
    b.classList.toggle('on', on);
    b.setAttribute('aria-pressed', on ? 'true' : 'false');
  });
  for (const h of LANG_HOOKS) h(LANG);
  updateHud();
  updateSensorOSD();
}
langEl.addEventListener('click', e => {
  const b = e.target.closest('button');
  if (b && b.dataset.l !== LANG) setLanguage(b.dataset.l);
});
HOOKS.ready.push(() => {
  setLanguage(LANG);
  window.sim.setLanguage = setLanguage;
  Object.defineProperty(window.sim, 'language', { get: () => LANG, enumerable: true });
});
