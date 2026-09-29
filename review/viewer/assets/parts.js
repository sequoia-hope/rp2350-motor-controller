/* parts.js — the part pane, shared by the PCB and 3D tabs.
 *
 * What the board file says about one part, laid out the way A365's
 * properties panel is: <board>/3d/parts.json, written by pcbview's 3D stage
 * from the board alone. Either tab can choose a part; the choice is
 * broadcast as a `pv-part` event, so the other tab has it selected too when
 * it is opened.
 */
(() => {
  const PV = window.PV = window.PV || {};
  const cache = {};

  // parts.json, fetched once per URL
  PV.parts = url => cache[url] || (cache[url] = fetch(url)
    .then(r => r.ok ? r.json() : Promise.reject(r.status)).catch(() => null));

  PV.select = (ref, from) =>
    dispatchEvent(new CustomEvent('pv-part', { detail: { ref, from } }));

  const esc = s => String(s ?? '').replace(/[&<>"]/g,
    c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);
  PV.esc = esc;
  const ATTRS = { smd: 'SMD', through_hole: 'through-hole', dnp: 'not fitted (DNP)',
    exclude_from_bom: 'not in the BOM', exclude_from_pos_files: 'not in the placement file',
    board_only: 'board only', allow_missing_courtyard: 'no courtyard' };
  const mm = v => (Math.round(v * 100) / 100).toFixed(2).replace('-', '−');
  const dl = (rows, cls) => '<dl' + (cls ? ' class="' + cls + '"' : '') + '>' +
    rows.filter(Boolean).map(([k, v]) => '<dt>' + k + '</dt><dd>' + v + '</dd>').join('') + '</dl>';
  const small = t => t ? '<small>' + esc(t) + '</small>' : '';
  PV.mm = mm;

  // The number the footprint (or the project's tables) gives it, to its LCSC
  // page -- and to JLCPCB's, the same C-number, which lists a few parts LCSC's
  // own shop does not. With none, a search for its value and package.
  function lcsc(d) {
    const ext = (href, text, cls) => '<a' + (cls ? ' class="' + cls + '"' : '') + ' href="' + esc(href) +
      '" target="_blank" rel="noopener">' + esc(text) + '</a>';
    if (/^C\d+$/.test(d.lcsc)) {
      return ext('https://www.lcsc.com/product-detail/' + d.lcsc + '.html', d.lcsc) + ' ' +
        ext('https://jlcpcb.com/partdetail/' + d.lcsc, 'JLCPCB', 'b3-alt') + small(d.mpn);
    }
    if (d.lcsc) return esc(d.lcsc) + small(d.mpn);
    const pkg = (d.footprint.match(/_(\d{4})_\d{4}Metric/) || [])[1];
    const q = [d.value.replace(/\//g, ' '), pkg].filter(Boolean).join(' ');
    return '<span class="b3-nc">none given</span> ' +
      ext('https://www.lcsc.com/search?q=' + encodeURIComponent(q), 'search LCSC', 'b3-alt');
  }

  // aside: the pane (.pv-iref, .pv-ib inside it); data: parts.json; ref: the
  // part, or null to empty it; extra: what only the caller knows
  // ({height} from the 3D model, {placeholder} when it has none)
  PV.pane = (aside, data, ref, extra = {}) => {
    const iref = aside.querySelector('.pv-iref'), ib = aside.querySelector('.pv-ib');
    if (!ref) {
      iref.textContent = '';
      ib.innerHTML = '<p class="b3-empty">Click a part to see it here.</p>';
      return;
    }
    const d = data && data.parts && data.parts[ref];
    iref.textContent = ref;
    if (!d) {
      ib.innerHTML = '<p class="b3-empty">' + esc(ref) + ' is not in parts.json.</p>';
      return;
    }
    const faces = (data.faces || {});
    const [fname, fnote] = faces[d.side] || [d.side === 'back' ? 'Back' : 'Front', ''];
    let h = dl([
      ['Designator', esc(ref) + (d.attrs.includes('dnp') ? '<span class="b3-dnp">DNP</span>' : '')],
      ['Component', esc(d.value) + small(d.role)],
      !d.attrs.some(a => a === 'exclude_from_bom' || a === 'exclude_from_pos_files') &&
        ['LCSC', lcsc(d)],
      ['Footprint', '<span class="mono">' + esc(d.footprint) + '</span>' +
        small([d.lib, d.descr].filter(Boolean).join(' · '))],
    ]);
    let where = 'x ' + mm(d.x) + ' &middot; y ' + mm(d.y) + ' mm';
    if (data.polar) {
      const dx = d.x - data.polar[0], dy = data.polar[1] - d.y;
      where += small('r ' + mm(Math.hypot(dx, dy)) + ' mm · θ ' +
        ((Math.atan2(dy, dx) * 180 / Math.PI + 360) % 360).toFixed(1) + '° from the centre');
    } else {
      where += small('as pcbnew reads it, y down');
    }
    h += dl([
      ['Location', where],
      ['Rotation', String(d.rot).replace('-', '−') + '&deg;'],
      ['Side', esc(fname) + small(fnote)],
      extra.height != null && ['Height', mm(extra.height) + ' mm' + small('its model, above the laminate')],
      extra.placeholder && ['3D model', '<span class="b3-nc">none that meshes</span>' +
        small('drawn as a box the size of its courtyard')],
    ]);
    const par = [];
    if (d.attrs.length) par.push(['Mounting', esc(d.attrs.map(a => ATTRS[a] || a).join(', '))]);
    if (/^https?:\/\//.test(d.datasheet))
      par.push(['Datasheet', '<a href="' + esc(d.datasheet) + '" target="_blank" rel="noopener">open</a>']);
    if (d.description) par.push(['Description', esc(d.description)]);
    for (const [k, v] of Object.entries(d.fields)) par.push([esc(k), esc(v)]);
    if (par.length) h += '<h4>Parameters</h4>' + dl(par, 'par');
    if (d.pins.length) {
      h += '<h4>Pins <span>' + d.pins.length + '</span></h4><table class="b3-pins"><tbody>' +
        d.pins.map(([n, net]) => '<tr><td>' + esc(n) + '</td><td>' +
          (!net || net.startsWith('unconnected-') ? '<span class="b3-nc">not connected</span>' : esc(net)) +
          '</td></tr>').join('') + '</tbody></table>';
    }
    ib.innerHTML = h;
    ib.scrollTop = 0;
  };

  // "H1 H2 H3 H4 H10" -> "H1–H4, H10"
  PV.span = refs => {
    const out = [];
    for (let i = 0; i < refs.length;) {
      const m = refs[i].match(/^([A-Z_]*)(\d+)$/i);
      let j = i;
      if (m) {
        while (j + 1 < refs.length) {
          const n = refs[j + 1].match(/^([A-Z_]*)(\d+)$/i);
          if (!n || n[1] !== m[1] || +n[2] !== +refs[j].match(/\d+$/)[0] + 1) break;
          j++;
        }
      }
      out.push(j > i + 1 ? refs[i] + '&ndash;' + refs[j] : refs.slice(i, j + 1).join(', '));
      i = j + 1;
    }
    return out.join(', ');
  };

  PV.natural = (a, b) => {
    const [, pa, na] = a.match(/^(\D*)(\d*)/), [, pb, nb] = b.match(/^(\D*)(\d*)/);
    return pa < pb ? -1 : pa > pb ? 1 : (+na - +nb);
  };
})();
