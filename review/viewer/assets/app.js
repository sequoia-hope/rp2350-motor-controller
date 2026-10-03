/* app.js — pcbview's own page round the viewer: the project tree and the
 * breadcrumb.
 *
 * The page itself is written whole by pcbview's build: the tree, the tab
 * strip, the panels and the board's facts are all in the HTML. viewer.js
 * runs the viewer (its tabs and the #hash); this follows it, marking the
 * view on screen in the tree and the breadcrumb, and makes the tree open,
 * shut and navigate. Loaded before viewer.js, so it hears the first view.
 */
(() => {
  const LABEL = { schematic: 'Schematic', copper: 'PCB', board3d: '3D', overview: 'Overview' };
  const closeNav = () => document.documentElement.classList.remove('nav-open');

  addEventListener('pv-view', e => {
    const id = e.detail.id;
    document.querySelectorAll('.a3-row[data-view]').forEach(r =>
      r.classList.toggle('here', r.dataset.view === id));
    const crumb = document.getElementById('pv-view');
    if (crumb) crumb.textContent = LABEL[id] || id;
  });

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
          // viewer.js opens the tab for a sheet's hash, sch.js the sheet
          history.pushState(null, '', '#sch-' + row.dataset.sheet);
          dispatchEvent(new HashChangeEvent('hashchange'));
        } else window.PV.view(row.dataset.view);
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
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start);
  else start();
})();
