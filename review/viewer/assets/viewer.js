/* viewer.js — the viewer's own controls: the tab strip, the side-panel
 * toggle, the about-this-board button, full screen, and the #hash that names
 * a view.
 *
 * A #hash names a view (#schematic, #copper, #board3d, #overview -- or #sch,
 * #pcb, #3d, #info) or a sheet (#sch-<path>), so any view can be linked to.
 * The viewers find out that their tab has been opened from the resize event
 * this fires, as they need to measure themselves; anything else can listen
 * for `pv-view` (detail: {id}), as app.js does to move its breadcrumb.
 *
 * On pcbview's own page a tab is a history entry. Embedded in a longer page
 * (<section class="vw" data-pv-embed>, written by `embed` in pcbview.toml), a
 * tab changes nothing but the viewer, and a #hash that is not a view is the
 * page's business. There, elements anywhere on the page marked
 * data-pv-note="<view>" are shown only while that view is open: the page's
 * own words about each tab.
 */
(() => {
  const ALIAS = { sch: 'schematic', pcb: 'copper', '3d': 'board3d', info: 'overview' };
  const PV = window.PV = window.PV || {};
  const vw = document.querySelector('section.vw');
  if (!vw) return;
  const embedded = vw.hasAttribute('data-pv-embed');
  const panelOf = id => vw.querySelector('#' + CSS.escape(id) + '.vw-panel');

  function show(id, push) {
    const panel = panelOf(id);
    if (!panel) return false;
    vw.querySelectorAll('.vw-panel').forEach(p => { p.hidden = p !== panel; });
    vw.querySelectorAll('.vw-tabs [data-tab]').forEach(b =>
      b.setAttribute('aria-selected', b.dataset.tab === id));
    const info = vw.querySelector('[data-vw=info]');
    if (info) info.classList.toggle('on', id === 'overview');
    const lay = vw.querySelector('[data-vw=layers]');
    if (lay) lay.hidden = !panel.querySelector('.cu-panel');
    document.querySelectorAll('[data-pv-note]').forEach(n => { n.hidden = n.dataset.pvNote !== id; });
    if (push && !embedded && location.hash.slice(1) !== id) history.pushState(null, '', '#' + id);
    dispatchEvent(new CustomEvent('pv-view', { detail: { id } }));
    dispatchEvent(new Event('resize'));
    return true;
  }
  PV.view = id => show(ALIAS[id] || id, true);
  PV.current = () => (vw.querySelector('.vw-panel:not([hidden])') || {}).id;

  // the view a #hash names, or null
  function named() {
    const h = decodeURIComponent(location.hash.slice(1));
    if (h.startsWith('sch-') && panelOf('schematic')) return 'schematic';
    const id = ALIAS[h] || h;
    return id && panelOf(id) ? id : null;
  }

  // a hidden panel is no place to scroll to, so the browser did not: now it is
  function route(first) {
    const id = named();
    if (id) {
      show(id);
      if (embedded) requestAnimationFrame(() => panelOf(id).scrollIntoView());
    } else if (!embedded && !location.hash.slice(1)) show('copper');
    else if (first) show(PV.current() || 'copper');
  }

  function start() {
    vw.querySelectorAll('.vw-tabs [data-tab]').forEach(b =>
      b.addEventListener('click', () => show(b.dataset.tab, true)));
    const lay = vw.querySelector('[data-vw=layers]');
    if (lay) lay.addEventListener('click', () => {
      const off = vw.classList.toggle('nolayers');
      lay.setAttribute('aria-pressed', !off);
      dispatchEvent(new Event('resize'));
    });
    let before = 'copper';
    const info = vw.querySelector('[data-vw=info]');
    if (info) info.addEventListener('click', () => {
      const open = PV.current() === 'overview';
      if (!open) before = PV.current();
      show(open ? before : 'overview', true);
    });
    const full = vw.querySelector('[data-vw=full]');
    if (full) {
      if (!vw.requestFullscreen) full.hidden = true;
      full.addEventListener('click', () => document.fullscreenElement
        ? document.exitFullscreen() : vw.requestFullscreen());
    }
    document.addEventListener('fullscreenchange', () => dispatchEvent(new Event('resize')));
    addEventListener('hashchange', () => route(false));
    addEventListener('popstate', () => route(false));
    route(true);
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start);
  else start();
})();
