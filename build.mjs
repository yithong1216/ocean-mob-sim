// Builds the single shareable file:  node build.mjs   ->  ocean_mob_sim.html
// Concatenates src/core/*.js (00..09), then src/features/*.js, then src/core/99_main.js into one module
// script inside src/page.html. `import` lines from any chunk are hoisted to the top and de-duplicated.
// No dependencies: plain Node 18+.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.dirname(fileURLToPath(import.meta.url));
const list = dir => fs.readdirSync(path.join(root, dir)).filter(f => f.endsWith('.js')).sort().map(f => `${dir}/${f}`);
const core = list('src/core');
const main = core.filter(f => /\/99_/.test(f));
const order = [...core.filter(f => !/\/99_/.test(f)), ...list('src/features'), ...main];

// imports are merged per module: { named } bindings are unioned, default / namespace imports kept once
const modules = new Map();   // specifier -> { def, ns, named: Set }
const bodies = [];
for (const file of order) {
  let src = fs.readFileSync(path.join(root, file), 'utf8').replace(/\r\n/g, '\n');
  src = src.replace(/^import\s([^;]*?)\sfrom\s+['"]([^'"]+)['"];[ \t]*\n/gm, (m, clause, spec) => {
    const mod = modules.get(spec) || { def: null, ns: null, named: new Set() };
    modules.set(spec, mod);
    const named = clause.match(/\{([^}]*)\}/);
    if (named) named[1].split(',').map(s => s.trim()).filter(Boolean).forEach(n => mod.named.add(n));
    const rest = clause.replace(/\{[^}]*\}/, '').split(',').map(s => s.trim()).filter(Boolean);
    for (const r of rest) {
      if (r.startsWith('*')) mod.ns = r;
      else mod.def = r;
    }
    return '';
  });
  bodies.push(`// ===== ${file} =====\n${src}`);
}
const imports = [...modules].map(([spec, m]) => {
  const parts = [m.def, m.ns, m.named.size ? `{ ${[...m.named].join(', ')} }` : null].filter(Boolean);
  return `import ${parts.join(', ')} from '${spec}';`;
});
// the licence/credits header (first comment of 00_header) stays above the imports
const header = bodies.shift();
const page = fs.readFileSync(path.join(root, 'src/page.html'), 'utf8');
const bundle = header.replace(/\n+$/, '\n') + imports.join('\n') + '\n\n' + bodies.join('');
const out = page.replace('/*@BUNDLE@*/\n', () => bundle);
fs.writeFileSync(path.join(root, 'ocean_mob_sim.html'), out);
console.log(`built ocean_mob_sim.html from ${order.length} files (${(out.length / 1024).toFixed(0)} KB)`);
