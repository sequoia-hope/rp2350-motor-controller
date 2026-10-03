/* sch.js — the SCH tab: each sheet as KiCad plots it, one at a time.
 *
 * <board>/sch/sheets.json (pcbview's schematic stage) lists the sheets in
 * page order; pick one on the left, drag to pan, scroll to zoom. The sheet is
 * fitted to the canvas, and may be zoomed until its own millimetre is about
 * 40 pixels wide, however big the paper. #sch-<sheet path> opens a sheet.
 */
(() => {
  function start() {
    const panel = document.getElementById('schematic');
    if (!panel) return;
    const dir = panel.dataset.dir;
    const $ = s => panel.querySelector(s);
    const list = $('#sch-sheets'), plate = $('#sch-plate'), pan = $('#sch-pan'), img = $('#sch-img');
    const zoomLab = $('#sch-zoom'), cap = $('#sch-cap'), pdf = $('#sch-pdf'), msg = $('#sch-msg');
    let sheets = [], cur = -1, z = 1, x = 0, y = 0, fetched = false;
    let fit = 1, iw = 0, ih = 0;                 // px per mm at 1x, the sheet's size in mm

    const shown = () => !panel.hidden && panel.offsetParent !== null;
    const maxZoom = () => Math.max(8, 40 / fit);

    function layout() {
      const W = plate.clientWidth, H = plate.clientHeight;
      if (!W || !H || !iw) return;
      fit = Math.min((W - 24) / iw, (H - 24) / ih);
      const w = iw * fit, h = ih * fit;
      Object.assign(img.style, { width: w + 'px', height: h + 'px',
        left: (W - w) / 2 + 'px', top: (H - h) / 2 + 'px' });
      place();
    }
    function place() {
      if (z <= 1) { z = 1; x = 0; y = 0; }
      pan.style.transform = `translate(${x}px, ${y}px) scale(${z})`;
      zoomLab.textContent = z.toFixed(1) + '×';
    }
    const fitAll = () => { z = 1; x = 0; y = 0; place(); };
    function zoomAt(f, cx, cy) {
      const nz = Math.min(maxZoom(), Math.max(1, z * f));
      x = cx - (cx - x) * nz / z; y = cy - (cy - y) * nz / z; z = nz;
      place();
    }

    function show(i) {
      if (i === cur || !sheets[i]) return;
      cur = i;
      const s = sheets[i];
      list.querySelectorAll('button').forEach((b, k) => b.classList.toggle('on', k === i));
      document.querySelectorAll('.a3-row[data-sheet]').forEach(r =>
        r.classList.toggle('here', r.dataset.sheet === s.sheet));
      [iw, ih] = s.size_mm || [420, 297];
      img.alt = `${s.title}: ${s.desc}`;
      img.src = dir + s.file;
      cap.textContent = `${s.title} — ${s.source || s.file}` + (s.paper ? `, ${s.paper}` : '') +
        (s.page ? `, page ${s.page}` : '') + (s.desc ? `. ${s.desc}.` : '.');
      fitAll();
      layout();
    }

    function load() {
      if (fetched || !shown()) return;
      fetched = true;
      fetch(dir + 'sheets.json').then(r => r.ok ? r.json() : Promise.reject(r.status)).then(j => {
        sheets = j.sheets;
        if (j.pdf) { pdf.href = dir + j.pdf; pdf.hidden = false; }
        list.replaceChildren(...sheets.map((s, i) => {
          const b = document.createElement('button');
          b.type = 'button';
          b.className = 'sch-sheet';
          b.innerHTML = `<span class="k">${i + 1}</span><span class="nm"></span><span class="fx"></span>`;
          b.querySelector('.nm').textContent = s.title;
          b.querySelector('.fx').textContent = s.desc;
          b.addEventListener('click', () => {
            history.replaceState(null, '', '#sch-' + s.sheet);
            show(i);
          });
          return b;
        }));
        // a hash's sheet, or the first that has parts on it: a top level
        // that only holds sheet boxes is not much to open on
        const first = Math.max(0, sheets.findIndex(s => s.symbols !== 0));
        show(wanted() >= 0 ? wanted() : first);
      }).catch(e => {
        msg.hidden = false;
        msg.innerHTML = 'The schematic plots are missing: run <code>python3 -m pcbview build</code>. (' + e + ')';
      });
    }
    const wanted = () => sheets.findIndex(s => '#sch-' + s.sheet === decodeURIComponent(location.hash));

    // drag / pinch / double tap: gesture.js; the mouse wheel is the plate's own
    PV.gestures(plate, {
      start: () => plate.classList.add('drag'),
      end: () => plate.classList.remove('drag'),
      pan: (dx, dy) => { if (z > 1) { x += dx; y += dy; place(); } },
      zoom: (f, cx, cy) => zoomAt(f, cx, cy),
      dblclick: (cx, cy) => zoomAt(2, cx, cy),
    });
    plate.addEventListener('wheel', e => {
      e.preventDefault();
      const r = plate.getBoundingClientRect();
      zoomAt(Math.exp(-e.deltaY * 0.0015), e.clientX - r.left, e.clientY - r.top);
    }, { passive: false });
    panel.querySelectorAll('[data-sch]').forEach(b => b.addEventListener('click', () => {
      if (b.dataset.sch === 'fit') fitAll();
      else zoomAt(b.dataset.sch === 'in' ? 1.5 : 1 / 1.5, plate.clientWidth / 2, plate.clientHeight / 2);
    }));
    addEventListener('keydown', e => {
      if (!shown() || e.target.closest('input, textarea, select') || e.metaKey || e.ctrlKey) return;
      const r = panel.getBoundingClientRect();                 // and on screen
      if (r.bottom < 0 || r.top > innerHeight) return;
      if (e.key === '0') fitAll();
      else if (e.key === '+' || e.key === '=') zoomAt(1.5, plate.clientWidth / 2, plate.clientHeight / 2);
      else if (e.key === '-') zoomAt(1 / 1.5, plate.clientWidth / 2, plate.clientHeight / 2);
      else if (/^[1-9]$/.test(e.key) && sheets[+e.key - 1]) show(+e.key - 1);
    });

    // viewer.js fires resize when a tab is chosen: the first time this one is,
    // fetch the list and the first sheet
    addEventListener('resize', () => { load(); layout(); });
    new ResizeObserver(layout).observe(plate);
    addEventListener('hashchange', () => { const i = wanted(); if (i >= 0) show(i); });
    load();
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start);
  else start();
})();
