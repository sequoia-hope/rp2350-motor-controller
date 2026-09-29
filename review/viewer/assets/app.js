/* app.js — what makes the page move: the tabs, the project tree, the hash.
 *
 * The page itself is written whole by pcbview's build: the tree, the tab
 * strip, the panels and the board's facts are all in the HTML. This only
 * switches between them. A #hash names a view (#schematic, #copper,
 * #board3d, #overview -- or #sch, #pcb, #3d, #info) or a sheet (#sch-<path>),
 * so any view can be linked to. The viewers find out that their tab has been
 * opened from the resize event it fires, as they need to measure themselves.
 */
(() => {
  const ALIAS = { sch: 'schematic', pcb: 'copper', '3d': 'board3d', info: 'overview' };
  const LABEL = { schematic: 'Schematic', copper: 'PCB', board3d: '3D', overview: 'Overview' };
  const vw = document.querySelector('section.vw');
  const closeNav = () => document.documentElement.classList.remove('nav-open');

  function show(id, push) {
    if (!vw) return;
    const panel = vw.querySelector('#' + id + '.vw-panel');
    if (!panel) return;
    vw.querySelectorAll('.vw-panel').forEach(p => { p.hidden = p !== panel; });
    vw.querySelectorAll('.vw-tabs [data-tab]').forEach(b =>
      b.setAttribute('aria-selected', b.dataset.tab === id));
    const info = vw.querySelector('[data-vw=info]');
    if (info) info.classList.toggle('on', id === 'overview');
    const lay = vw.querySelector('[data-vw=layers]');
    if (lay) lay.hidden = !panel.querySelector('.cu-panel');
    document.querySelectorAll('.a3-row[data-view]').forEach(r =>
      r.classList.toggle('here', r.dataset.view === id));
    const crumb = document.getElementById('pv-view');
    if (crumb) crumb.textContent = LABEL[id] || id;
    if (push && location.hash.slice(1) !== id) history.pushState(null, '', '#' + id);
    dispatchEvent(new Event('resize'));
  }

  function route() {
    const h = decodeURIComponent(location.hash.slice(1));
    if (h.startsWith('sch-')) return show('schematic');
    show(ALIAS[h] || h || 'copper');
    if (!vw.querySelector('.vw-panel:not([hidden])')) show('copper');
  }

  function start() {
    // the tree: twisties open and shut, folders toggle on their row
    document.querySelectorAll('.a3-tree li').forEach(li => {
      const row = li.querySelector(':scope > .a3-row'), tw = row && row.querySelector('.a3-tw');
      const ul = li.querySelector(':scope > ul');
      if (!tw || !ul) return;
      const toggle = () => {
        ul.hidden = !ul.hidden;
        tw.setAttribute('aria-expanded', !ul.hidden);
      };
      tw.addEventListener('click', e => { e.preventDefault(); e.stopPropagation(); toggle(); });
      if (!row.querySelector('a')) row.addEventListener('click', toggle);
    });
    // a row that is a link: the whole row follows it
    document.querySelectorAll('.a3-row').forEach(row => {
      const a = row.querySelector('a.t');
      if (a) row.addEventListener('click', e => { if (e.target === row || e.target.closest('.ic')) a.click(); });
    });
    // this board's views and sheets: stay on the page
    document.querySelectorAll('.a3-row[data-view] a, .a3-row[data-sheet] a').forEach(a =>
      a.addEventListener('click', e => {
        e.preventDefault();
        const row = a.closest('.a3-row');
        if (row.dataset.sheet) {
          history.pushState(null, '', '#sch-' + row.dataset.sheet);
          show('schematic');
          dispatchEvent(new HashChangeEvent('hashchange'));
        } else show(row.dataset.view, true);
        closeNav();
      }));

    const menu = document.querySelector('.a3-menu');
    if (menu) menu.addEventListener('click', e => {
      e.stopPropagation();
      document.documentElement.classList.toggle('nav-open');
    });
    document.addEventListener('click', e => {
      if (!e.target.closest('.a3-side')) closeNav();
    });
    if (!vw) return;

    vw.querySelectorAll('.vw-tabs [data-tab]').forEach(b =>
      b.addEventListener('click', () => show(b.dataset.tab, true)));
    const lay = vw.querySelector('[data-vw=layers]');
    lay.addEventListener('click', () => {
      const off = vw.classList.toggle('nolayers');
      lay.setAttribute('aria-pressed', !off);
      dispatchEvent(new Event('resize'));
    });
    let before = 'copper';
    vw.querySelector('[data-vw=info]').addEventListener('click', () => {
      const open = !vw.querySelector('#overview').hidden;
      if (!open) before = vw.querySelector('.vw-panel:not([hidden])').id;
      show(open ? before : 'overview', true);
    });
    const full = vw.querySelector('[data-vw=full]');
    if (!vw.requestFullscreen) full.hidden = true;
    full.addEventListener('click', () => document.fullscreenElement
      ? document.exitFullscreen() : vw.requestFullscreen());
    document.addEventListener('fullscreenchange', () => dispatchEvent(new Event('resize')));
    addEventListener('hashchange', route);
    addEventListener('popstate', route);
    route();
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start);
  else start();
})();
