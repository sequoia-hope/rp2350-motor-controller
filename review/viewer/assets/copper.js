/* copper.js — the PCB tab: every layer, stacked in register.
 *
 * Everything it draws comes from <board>/layers/layers.json, written by
 * pcbview's layer stage: one SVG per layer, all cropped to the same box in
 * KiCad's own millimetres (view_mm), plus body.svg, the board filled from its
 * outline. The board is fitted into the canvas and panned and zoomed as one;
 * the cursor reads out the x and y pcbnew would show. A click (not a drag)
 * picks the part under the cursor from <board>/3d/parts.json and opens the
 * part pane, the same one the 3D tab has.
 */
(() => {
  const root = document.getElementById('copper');
  if (!root) return;
  const DIR = root.dataset.dir;
  const $ = (sel, el = root) => el.querySelector(sel);
  const el = (tag, cls, html) => {
    const n = document.createElement(tag);
    if (cls) n.className = cls;
    if (html != null) n.innerHTML = html;
    return n;
  };
  const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

  fetch(DIR + 'layers.json')
    .then(r => r.ok ? r.json() : Promise.reject(r.status))
    .then(build)
    .catch(e => {
      const f = $('#cu-fallback');
      f.hidden = false;
      f.innerHTML = 'The layer plots are not built yet &mdash; run <code>python3 -m pcbview build</code>. (' + e + ')';
    });

  function build(meta) {
    const [vx, vy, vw, vh] = meta.view_mm;          // the crop, KiCad mm
    const copper = meta.layers.filter(l => l.kind === 'copper');
    const over = meta.layers.filter(l => l.kind !== 'copper');
    const rows = copper.concat(over);               // panel order, and key order
    const state = { sel: copper[0].slug, vis: {}, dim: 0.16, mirror: false, s: 1, tx: 0, ty: 0, grid: false };
    rows.forEach(l => state.vis[l.slug] = l.kind === 'copper' || l.kind === 'outline');

    const plate = $('#cu-plate'), pan = $('#cu-pan'), flip = $('#cu-flip');
    $('.cu').style.setProperty('--plate', meta.plate);
    const body = el('img');
    body.src = DIR + meta.body;
    body.alt = 'the board';
    body.style.zIndex = 0;
    flip.appendChild(body);
    const imgs = {};
    for (const l of rows) {
      const img = el('img');
      img.alt = l.name + ' of ' + meta.board;
      img.dataset.src = DIR + l.file;            // loaded when first shown
      flip.appendChild(img);
      imgs[l.slug] = img;
    }
    const mark = el('div', 'cu-mark');
    mark.hidden = true;
    flip.appendChild(mark);

    // ---- the panel ---------------------------------------------------------
    const box = $('#cu-layers');
    const HEAD = { copper: 'Copper, front to back', outline: 'Overlay' };
    let lastKind = null;
    rows.forEach((l, i) => {
      const k = l.kind === 'copper' ? 'copper' : 'outline';
      if (k !== lastKind) box.appendChild(el('div', 'cu-head', HEAD[k]));
      lastKind = k;
      const row = el('div', 'cu-lay');
      row.dataset.slug = l.slug;
      row.innerHTML =
        `<span class="k">${i < 9 ? i + 1 : ''}</span>` +
        `<input type="radio" name="cu-sel" id="cu-r-${l.slug}" title="draw this layer on top, opaque">` +
        `<input type="checkbox" id="cu-v-${l.slug}" title="show this layer at all">` +
        `<span class="sw" style="background:${l.color}"></span>` +
        `<label for="cu-r-${l.slug}"><span class="nm">${l.name}</span>` +
        `<span class="fx">${facts(l)}</span></label>`;
      box.appendChild(row);
      row.querySelector('input[type=radio]').onchange = () => { state.sel = l.slug; state.vis[l.slug] = true; draw(); };
      row.querySelector('input[type=checkbox]').onchange = e => { state.vis[l.slug] = e.target.checked; draw(); };
    });
    const [bw0, bh0] = meta.size_mm;
    $('#cu-stats').textContent =
      (meta.round ? `Ø${meta.round.dia} mm` : `${bw0} × ${bh0} mm`) + ` · ${copper.length} copper layers · ` +
      `${meta.tracks.toLocaleString('en')} track segments, ${Math.round(meta.length_mm).toLocaleString('en')} mm · ` +
      `${meta.vias.toLocaleString('en')} vias · plotted ${meta.generated}`;

    // ---- controls ------------------------------------------------------------
    const dim = $('#cu-dim'), mir = $('#cu-mirror');
    dim.value = state.dim * 100;
    dim.oninput = () => { state.dim = dim.value / 100; draw(); };
    mir.onchange = () => { state.mirror = mir.checked; draw(); };
    root.addEventListener('click', e => {
      const act = e.target.closest('button')?.dataset.act;
      if (!act) return;
      e.preventDefault();
      if (act === 'all') rows.forEach(l => state.vis[l.slug] = true);
      else if (act === 'none') rows.forEach(l => state.vis[l.slug] = l.slug === state.sel);
      else if (act === 'solo') rows.forEach(l => state.vis[l.slug] = l.slug === state.sel || l.kind === 'outline');
      else if (act === 'in') zoom(state.s * 1.5, plate.clientWidth / 2, plate.clientHeight / 2);
      else if (act === 'out') zoom(state.s / 1.5, plate.clientWidth / 2, plate.clientHeight / 2);
      else if (act === 'fit') { state.s = 1; state.tx = state.ty = 0; }
      else if (act === 'grid') { state.grid = !state.grid; grid(); }
      draw();
    });

    // ---- the board in the canvas: fitted, then panned and zoomed ----------------
    let fit = 1, bl = 0, bt = 0, bw = 0, bh = 0;
    function layout() {
      const W = plate.clientWidth, H = plate.clientHeight;
      if (!W || !H) return;
      fit = Math.min(W / vw, H / vh);             // px per mm at 1x
      bw = vw * fit; bh = vh * fit; bl = (W - bw) / 2; bt = (H - bh) / 2;
      Object.assign(flip.style, { left: bl + 'px', top: bt + 'px', width: bw + 'px', height: bh + 'px' });
      place();
    }
    const maxZoom = () => Math.max(16, 400 / fit);   // at most 400 px to the millimetre

    function zoom(next, ax, ay) {
      const s2 = clamp(next, 1, maxZoom());
      state.tx = ax - (ax - state.tx) * s2 / state.s;
      state.ty = ay - (ay - state.ty) * s2 / state.s;
      state.s = s2;
      place();
    }

    // the board may move until its edge reaches the middle of the canvas
    function place() {
      const W = plate.clientWidth, H = plate.clientHeight, s = state.s;
      state.tx = clamp(state.tx, Math.min(0, W / 2 - s * (bl + bw)), Math.max(0, W / 2 - s * bl));
      state.ty = clamp(state.ty, Math.min(0, H / 2 - s * (bt + bh)), Math.max(0, H / 2 - s * bt));
      if (s === 1 && !drag) { state.tx = state.ty = 0; }
      pan.style.transform = `translate(${state.tx}px, ${state.ty}px) scale(${s})`;
      flip.style.transform = state.mirror ? 'scaleX(-1)' : '';
      $('#cu-zoom').textContent = s.toFixed(1) + '×';
      const mmpx = 1 / (fit * s);                   // the scale bar
      const nice = [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100].reduce((a, b) =>
        Math.abs(b - 90 * mmpx) < Math.abs(a - 90 * mmpx) ? b : a);
      $('#cu-bar').style.width = (nice / mmpx) + 'px';
      $('#cu-barlab').textContent = (nice < 1 ? nice : nice) + ' mm';
      $('#cu-back').textContent = state.mirror ? 'from the back' : '';
    }

    // a point on the plate, in KiCad mm
    function at(px, py) {
      const u = ((px - state.tx) / state.s - bl) / bw;
      const v = ((py - state.ty) / state.s - bt) / bh;
      return [vx + (state.mirror ? 1 - u : u) * vw, vy + v * vh];
    }

    function readout(px, py) {
      const [x, y] = at(px, py);
      let t = `x ${x.toFixed(2)}  y ${y.toFixed(2)} mm`;
      if (meta.polar) {
        const dx = x - meta.polar[0], dy = meta.polar[1] - y;
        t += `  ·  r ${Math.hypot(dx, dy).toFixed(2)} θ ${((Math.atan2(dy, dx) * 180 / Math.PI + 360) % 360).toFixed(1)}°`;
      }
      $('#cu-read').textContent = t;
    }

    let drag = null;
    plate.addEventListener('pointerdown', e => {
      drag = { x: e.clientX, y: e.clientY, tx: state.tx, ty: state.ty, moved: false };
      plate.setPointerCapture(e.pointerId);
    });
    plate.addEventListener('pointerup', e => {
      const was = drag;
      drag = null;
      plate.classList.remove('drag');
      plate.releasePointerCapture(e.pointerId);
      if (was && !was.moved) {
        const r = plate.getBoundingClientRect();
        pick(e.clientX - r.left, e.clientY - r.top);
      }
      place();
    });
    plate.addEventListener('pointermove', e => {
      const r = plate.getBoundingClientRect();
      if (drag) {
        if (!drag.moved && Math.hypot(e.clientX - drag.x, e.clientY - drag.y) > 4) {
          drag.moved = true;
          plate.classList.add('drag');
        }
        if (drag.moved) {
          state.tx = drag.tx + (e.clientX - drag.x);
          state.ty = drag.ty + (e.clientY - drag.y);
          place();
        }
      }
      readout(e.clientX - r.left, e.clientY - r.top);
    });
    plate.addEventListener('pointerleave', () => { $('#cu-read').textContent = ''; });
    plate.addEventListener('wheel', e => {
      e.preventDefault();
      const r = plate.getBoundingClientRect();
      zoom(state.s * Math.exp(-e.deltaY * 0.0018), e.clientX - r.left, e.clientY - r.top);
    }, { passive: false });
    plate.addEventListener('dblclick', e => {
      const r = plate.getBoundingClientRect();
      if (state.s > 1.01) { state.s = 1; state.tx = state.ty = 0; place(); }
      else zoom(4, e.clientX - r.left, e.clientY - r.top);
    });

    const shown = () => !root.hidden && root.offsetParent !== null;
    document.addEventListener('keydown', e => {
      if (!shown() || /^(INPUT|TEXTAREA|SELECT)$/.test(e.target.tagName) || e.metaKey || e.ctrlKey) return;
      const n = parseInt(e.key, 10);
      if (n >= 1 && n <= Math.min(9, rows.length)) {
        state.sel = rows[n - 1].slug; state.vis[state.sel] = true;
      } else if (e.key === 'm') { state.mirror = mir.checked = !state.mirror; }
      else if (e.key === 's') { rows.forEach(l => state.vis[l.slug] = l.slug === state.sel || l.kind === 'outline'); }
      else if (e.key === 'a') { rows.forEach(l => state.vis[l.slug] = true); }
      else if (e.key === '0') { state.s = 1; state.tx = state.ty = 0; }
      else if (e.key === 'Escape') { choose(null); return; }
      else if (e.key === '+' || e.key === '=') zoom(state.s * 1.5, plate.clientWidth / 2, plate.clientHeight / 2);
      else if (e.key === '-') zoom(state.s / 1.5, plate.clientWidth / 2, plate.clientHeight / 2);
      else return;
      e.preventDefault();
      draw();
    });

    function draw() {
      const back = state.mirror;
      rows.forEach((l, i) => {
        const img = imgs[l.slug], on = state.vis[l.slug], sel = l.slug === state.sel;
        if (on && !img.src) img.src = img.dataset.src;          // load on demand
        img.style.display = on ? '' : 'none';
        img.style.opacity = sel || l.kind !== 'copper' ? 1 : state.dim;
        // physical order, front on top -- reversed when looking from the
        // back; the outline over everything so a hole reads as a hole, and
        // the selected layer over the rest of the copper
        img.style.zIndex = l.kind === 'outline' ? 300 : l.kind !== 'copper' ? 250 + i
          : sel ? 200 : back ? rows.length - i : i + 1;
        const row = box.querySelector(`.cu-lay[data-slug="${l.slug}"]`);
        row.classList.toggle('on', sel);
        row.querySelector('input[type=radio]').checked = sel;
        row.querySelector('input[type=checkbox]').checked = on;
      });
      const l = rows.find(r => r.slug === state.sel);
      $('#cu-note').innerHTML = `<b>${l.name}</b> — ${PV.esc(l.note)}` +
        (l.zones && l.zones.length ? ` Pours: <b>${l.zones.map(PV.esc).join(', ')}</b>.` : '');
      place();
      if (state.grid) grid(true);
    }

    // ---- grid mode: the copper layers again, as tiles --------------------------
    function grid(refresh) {
      const g = $('#cu-gridbox');
      g.hidden = !state.grid;
      plate.hidden = state.grid;
      $('[data-act=grid]').classList.toggle('on', state.grid);
      if (!state.grid || (g.childElementCount && !refresh)) return;
      g.innerHTML = '';
      for (const l of copper) {
        const cell = el('div', 'cu-cell'), t = el('div', 'cu-tile');
        t.style.aspectRatio = vw + ' / ' + vh;
        for (const src of [meta.body, l.file, 'edge.svg']) {
          const i = el('img'); i.src = DIR + src; i.alt = l.name; t.appendChild(i);
        }
        t.appendChild(el('b', null, l.name));
        cell.appendChild(t);
        cell.appendChild(el('div', 'cu-cap', `${facts(l)}<span>pour ${l.zones.map(PV.esc).join(', ') || 'none'}</span>`));
        g.appendChild(cell);
      }
    }

    // ---- parts: click to pick, the pane on the right -------------------------
    const pane = $('#cu-info');
    let parts = null, chosen = null;
    const partsReady = root.dataset.parts ? PV.parts(root.dataset.parts).then(d => { parts = d; return d; })
      : Promise.resolve(null);

    function pick(px, py) {
      if (!parts) return;
      const [x, y] = at(px, py);
      const face = state.mirror ? 'back' : 'front';
      let best = null;
      for (const [ref, d] of Object.entries(parts.parts)) {
        const b = d.box;
        if (!b || x < b[0] || x > b[2] || y < b[1] || y > b[3]) continue;
        const area = (b[2] - b[0]) * (b[3] - b[1]);
        const rank = (d.side === face ? 0 : 1e6) + area;       // the face you look at, then the smallest
        if (!best || rank < best[0]) best = [rank, ref];
      }
      choose(best ? best[1] : null, true);
    }

    function choose(ref, mine) {
      chosen = ref;
      const d = ref && parts && parts.parts[ref];
      if (d && d.box) {
        const b = d.box;
        Object.assign(mark.style, {
          left: (100 * (b[0] - vx) / vw) + '%', top: (100 * (b[1] - vy) / vh) + '%',
          width: (100 * (b[2] - b[0]) / vw) + '%', height: (100 * (b[3] - b[1]) / vh) + '%',
        });
        mark.hidden = false;
      } else mark.hidden = true;
      if (ref) pane.hidden = false;
      PV.pane(pane, parts, ref);
      if (mine) PV.select(ref, 'copper');
      requestAnimationFrame(layout);
    }

    pane.querySelector('.b3-x').addEventListener('click', () => { pane.hidden = true; choose(null, true); });
    addEventListener('pv-part', e => {
      if (e.detail.from === 'copper') return;
      partsReady.then(() => { if (e.detail.ref !== chosen) choose(e.detail.ref, false); });
    });

    draw();
    layout();
    addEventListener('resize', layout);
    new ResizeObserver(layout).observe(plate);
  }

  const facts = l => l.kind !== 'copper' ? '' :
    (l.tracks ? `${l.tracks} tracks · ${Math.round(l.length_mm)} mm · ` : 'plane · ') + `${l.pads} pads`;
})();
