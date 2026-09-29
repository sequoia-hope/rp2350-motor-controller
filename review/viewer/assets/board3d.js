/* board3d.js — the 3D tab.
 *
 * Draws <board>/3d/board.glb, written by pcbview's 3D stage: the board as
 * kicad-cli exports it, merged and quantized, each part's node carrying its
 * reference, value, footprint and side (and `placeholder` for a part drawn as
 * a box because it has no model that meshes). board.json says what the
 * caption says; parts.json what the part pane on the right says about a
 * clicked part (parts.js draws the pane, shared with the PCB tab). The panel
 * names the directory: <div id="board3d" data-dir="rev-a/3d/">. Nothing
 * loads until the tab is opened, and nothing draws while it is shut.
 *
 * glTF is y-up: the board lies in x-z, the outward face (F.Cu) towards +y,
 * and +z is KiCad's +y, so looking down -y with -z up is pcbnew's own view.
 */
import * as THREE from 'three';
import { TrackballControls } from './vendor/TrackballControls.js';
import { GLTFLoader } from './vendor/GLTFLoader.js';
import { RoomEnvironment } from './vendor/RoomEnvironment.js';

const root = document.getElementById('board3d');
if (root) init(root);

// camera direction (from the board towards the eye) and up, per view
const VIEWS = {
  front: [[0, 1, 0], [0, 0, -1]],
  back: [[0, -1, 0], [0, 0, -1]],       // turned over left to right, like pcbnew's flip
  tilt: [[0, 0.62, 0.78], [0, 1, 0]],
  edge: [[0, 0.06, 1], [0, 1, 0]],
};
// the board's own meshes, by the layer export_3d.py names them, into toggles
const GROUPS = [
  ['parts', 'Parts', null],
  ['copper', 'Copper', ['copper', 'pad', 'via']],
  ['soldermask', 'Solder mask', ['soldermask']],
  ['silkscreen', 'Silkscreen', ['silkscreen']],
  ['PCB', 'Laminate', ['PCB']],
];
const HILITE = new THREE.Color(0x2f6feb);

function init(root) {
  const DIR = root.dataset.dir;
  const $ = s => root.querySelector(s);
  const stage = $('#b3-stage'), msg = $('#b3-msg'), tag = $('#b3-tag');
  const info = $('#b3-info');
  let details = null;                          // parts.json, when it has come
  const say = html => { msg.innerHTML = html; msg.hidden = !html; };

  let started = false, visible = false, running = false;
  new IntersectionObserver(es => {
    visible = es.some(e => e.isIntersecting);
    if (visible && !started) { started = true; start(); }
    if (visible) kick();
  }, { rootMargin: '300px' }).observe(stage);

  let renderer, scene, camera, controls, board, tween = null, dirty = true;
  const parts = new Map(), layers = {}, occluders = [];
  let hover = null, chosen = null, pendingRef = null;

  function start() {
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true });
    } catch (e) {
      say('This browser would not start WebGL, so there is no 3D view here.');
      return;
    }
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    renderer.toneMapping = THREE.ACESFilmicToneMapping;    // bare copper glares without
    stage.prepend(renderer.domElement);
    scene = new THREE.Scene();
    scene.background = new THREE.Color(getComputedStyle(stage).backgroundColor);   // style.css's canvas
    const pmrem = new THREE.PMREMGenerator(renderer);
    scene.environment = pmrem.fromScene(new RoomEnvironment(renderer), 0.04).texture;
    camera = new THREE.PerspectiveCamera(30, 1, 1e-4, 10);
    const head = new THREE.DirectionalLight(0xffffff, 0.7);    // rides with the eye
    head.position.set(0.3, 0.6, 1);
    camera.add(head);
    scene.add(camera);
    controls = new TrackballControls(camera, renderer.domElement);
    Object.assign(controls, { rotateSpeed: 3.5, zoomSpeed: 1.3, panSpeed: 0.8,
                              dynamicDampingFactor: 0.14 });
    controls.addEventListener('start', () => { tween = null; });
    controls.addEventListener('change', () => { dirty = true; });
    new ResizeObserver(resize).observe(stage);
    resize();
    wire();
    say('Loading the model&hellip;');

    fetch(DIR + 'board.json').then(r => r.ok ? r.json() : null).then(facts).catch(() => {});
    PV.parts(DIR + 'parts.json').then(d => { details = d; if (chosen) pane(chosen); });
    new GLTFLoader().load(DIR + 'board.glb', loaded,
      e => { if (e.total) say('Loading the model&hellip; ' + Math.round(100 * e.loaded / e.total) + '%'); },
      e => say('The model is not built yet &mdash; run <code>python3 -m pcbview build</code>. (' +
               (e.message || e) + ')'));
  }

  function loaded(gltf) {
    board = gltf.scene;
    board.traverse(o => {
      const x = o.userData;
      if (x.ref) parts.set(x.ref, o);
      if (x.layer) {
        (layers[x.layer] = layers[x.layer] || []).push(o);
        if (x.layer === 'PCB' || x.layer === 'soldermask') occluders.push(o);
      }
    });
    // centre on the laminate, not on everything: a connector may hang over the edge
    const pcb = layers.PCB ? new THREE.Box3().setFromObject(layers.PCB[0])
                           : new THREE.Box3().setFromObject(board);
    board.position.sub(pcb.getCenter(new THREE.Vector3()));
    scene.add(board);
    const size = pcb.getSize(new THREE.Vector3());
    board.userData.radius = Math.max(size.x, size.z) / 2;
    view('tilt', true);
    toggles();
    finder();
    say('');
    redraw();
    if (pendingRef) { choose(parts.get(pendingRef) || null, false); pendingRef = null; }
  }

  // ---- views ---------------------------------------------------------------
  function fitDistance(r) {
    const fov = THREE.MathUtils.degToRad(camera.fov) / 2;
    return 1.08 * r / Math.sin(Math.min(fov, Math.atan(Math.tan(fov) * camera.aspect)));
  }

  function view(name, now) {
    const [d, u] = VIEWS[name];
    const r = board.userData.radius;
    // tilted, the disc foreshortens; edge on, it is a strip: fit what shows
    const dist = fitDistance(r * ({ tilt: 0.85, edge: 0.75 }[name] || 1));
    fly(new THREE.Vector3(...d).normalize(), new THREE.Vector3(...u), new THREE.Vector3(), dist, now);
    root.querySelectorAll('[data-view]').forEach(b => b.classList.toggle('on', b.dataset.view === name));
    return name;
  }

  function fly(dir, up, target, dist, now) {
    const eye = new THREE.PerspectiveCamera();   // a camera: lookAt aims its -z, not +z
    eye.position.copy(target).addScaledVector(dir, dist);
    eye.up.copy(up);
    eye.lookAt(target);
    const to = { q: eye.quaternion.clone(), t: target.clone(), d: dist };
    still();
    if (now) { place(to.q, to.t, to.d); redraw(); return; }
    tween = { from: { q: camera.quaternion.clone(), t: controls.target.clone(),
                      d: camera.position.distanceTo(controls.target) },
              to, t0: performance.now() };
    kick();
  }

  function place(q, t, d) {
    camera.quaternion.copy(q);
    camera.up.set(0, 1, 0).applyQuaternion(q);
    camera.position.set(0, 0, d).applyQuaternion(q).add(t);
    controls.target.copy(t);
    camera.lookAt(t);
  }

  // TrackballControls keeps turning after a flick, and has no way to stop it
  // but a damping of 1 for one update
  function still() {
    const d = controls.dynamicDampingFactor;
    controls.dynamicDampingFactor = 1;
    controls.update();
    controls.dynamicDampingFactor = d;
  }

  function step(now) {
    if (!tween) return false;
    const k = Math.min(1, Math.max(0, (now - tween.t0) / 450)), e = k * k * (3 - 2 * k);
    const { from, to } = tween;
    place(from.q.clone().slerp(to.q, e), from.t.clone().lerp(to.t, e),
          from.d + (to.d - from.d) * e);
    if (k === 1) tween = null;
    return true;
  }

  // ---- drawing -------------------------------------------------------------
  function kick() {
    if (running || !renderer) return;
    running = true;
    requestAnimationFrame(loop);
  }

  function redraw() { dirty = true; kick(); }

  // runs while the section is on screen; draws only when something moved
  function loop(now) {
    if (!visible) { running = false; return; }
    if (step(now)) dirty = true;
    else controls.update();
    if (dirty) {
      dirty = false;
      clip();
      renderer.render(scene, camera);
      placeTag();
    }
    requestAnimationFrame(loop);
  }

  // copper sits 35 um over the laminate and the mask over that: the depth
  // range follows the eye, or they fight at any distance worth looking from
  function clip() {
    const d = camera.position.distanceTo(controls.target);
    const r = board ? board.userData.radius : 0.05;
    camera.near = Math.max(d / 100, 1e-5);
    camera.far = d + 4 * r;
    camera.updateProjectionMatrix();
  }

  function resize() {
    const w = stage.clientWidth, h = stage.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    controls.handleResize();
    redraw();
  }

  // ---- parts: hover, click, find ------------------------------------------
  const ray = new THREE.Raycaster();
  const shown = o => { for (; o; o = o.parent) if (!o.visible) return false; return true; };

  function partAt(ev) {
    if (!board) return null;
    const r = renderer.domElement.getBoundingClientRect();
    ray.setFromCamera(new THREE.Vector2(((ev.clientX - r.left) / r.width) * 2 - 1,
                                        -((ev.clientY - r.top) / r.height) * 2 + 1), camera);
    const targets = [...parts.values(), ...occluders].filter(shown);
    for (const h of ray.intersectObjects(targets, true)) {
      if (!shown(h.object)) continue;
      for (let o = h.object; o; o = o.parent) if (o.userData.ref) return o;
      return null;                           // the board is in the way
    }
    return null;
  }

  function tint(part, on) {
    part.traverse(o => {
      if (!o.isMesh) return;
      if (on && !o.userData.own) {
        o.userData.own = o.material;
        o.material = o.material.clone();
        o.material.emissive = HILITE;
        o.material.emissiveIntensity = 0.7;
      } else if (!on && o.userData.own) {
        o.material.dispose();
        o.material = o.userData.own;
        delete o.userData.own;
      }
    });
    redraw();
  }

  function setHover(p) {
    if (p === hover) return;
    if (hover && hover !== chosen) tint(hover, false);
    hover = p;
    if (hover) tint(hover, true);
    renderer.domElement.style.cursor = hover ? 'pointer' : '';
    label();
  }

  function choose(p, go, mine) {
    if (chosen && chosen !== p && chosen !== hover) tint(chosen, false);
    chosen = p;
    if (chosen) tint(chosen, true);
    $('#b3-find').value = chosen ? chosen.userData.ref : '';
    pane(chosen);
    if (mine) PV.select(chosen ? chosen.userData.ref : null, 'board3d');
    if (chosen) info.hidden = false;           // the stage narrows; its observer re-fits
    label();
    if (chosen && go) focus(chosen);
  }

  function focus(p) {
    const box = new THREE.Box3().setFromObject(p);
    if (box.isEmpty()) return;                   // a node with no geometry: nowhere to fly
    const c = box.getCenter(new THREE.Vector3());
    let dir = camera.position.clone().sub(controls.target).normalize();
    let up = camera.up.clone();
    const back = p.userData.side === 'back';
    if (back ? dir.y > 0 : dir.y < 0) {          // it is on the face turned away
      [dir, up] = VIEWS[back ? 'back' : 'front'].map(v => new THREE.Vector3(...v));
      dir.lerp(new THREE.Vector3(0, 0, 1), 0.35).normalize();
      up.set(0, 1, 0);
    }
    const size = box.getSize(new THREE.Vector3()).length();
    fly(dir, up, c, Math.max(fitDistance(size * 1.6), 0.012));
    root.querySelectorAll('[data-view]').forEach(b => b.classList.remove('on'));
  }

  // the tag names what is under the cursor, and the chosen part only while
  // the pane is shut: with it open, the pane says it
  const tagged = () => hover || (info.hidden ? chosen : null);

  function label() {
    const p = tagged();
    if (!p) { tag.hidden = true; return; }
    const x = p.userData;
    tag.innerHTML = '<b>' + x.ref + '</b> ' + esc(x.value) +
      '<span>' + esc(x.footprint) + ' &middot; ' + x.side + (x.placeholder ? ' &middot; no model' : '') + '</span>';
    tag.hidden = false;
    placeTag();
  }

  function placeTag() {
    const p = tagged();
    if (!p || tag.hidden) return;
    const box = new THREE.Box3().setFromObject(p);
    const at = box.getCenter(new THREE.Vector3()).project(camera);
    const w = stage.clientWidth, h = stage.clientHeight;
    tag.style.left = Math.round((at.x + 1) / 2 * w) + 'px';
    tag.style.top = Math.round((1 - at.y) / 2 * h) + 'px';
  }

  function finder() {
    const refs = [...parts.keys()].sort(natural);
    $('#b3-refs').innerHTML = refs.map(r => {
      const x = parts.get(r).userData;
      return '<option value="' + r + '">' + esc(x.value) + ' &middot; ' + x.side + '</option>';
    }).join('');
    const box = $('#b3-find');
    box.disabled = false;
    const go = () => {
      const v = box.value.trim().toUpperCase();
      const p = parts.get(v);
      box.classList.toggle('bad', !!v && !p);
      if (p) {
        if (!shown(p)) { layerOn('parts'); }
        choose(p, true, true);
      }
    };
    box.addEventListener('change', go);
    box.addEventListener('keydown', e => { if (e.key === 'Enter') go(); });
  }

  // ---- toggles ---------------------------------------------------------------
  function toggles() {
    const box = $('#b3-layers');
    for (const [key, name, list] of GROUPS) {
      const objs = list ? list.flatMap(l => layers[l] || []) : [...parts.values()];
      if (!objs.length) continue;
      const lab = document.createElement('label');
      lab.innerHTML = '<input type="checkbox" checked data-layer="' + key + '"> ' + name;
      lab.firstChild.addEventListener('change', e => {
        objs.forEach(o => { o.visible = e.target.checked; });
        if (!e.target.checked && key === 'parts') { setHover(null); choose(null); }
        redraw();
      });
      box.appendChild(lab);
    }
  }

  function layerOn(key) {
    const c = root.querySelector('[data-layer="' + key + '"]');
    if (c && !c.checked) { c.checked = true; c.dispatchEvent(new Event('change')); }
  }

  function wire() {
    root.querySelectorAll('[data-view]').forEach(b =>
      b.addEventListener('click', () => { if (board) view(b.dataset.view); }));
    const cv = renderer.domElement;
    let down = null, pending = null;
    cv.addEventListener('pointerdown', e => { down = [e.clientX, e.clientY]; });
    cv.addEventListener('pointermove', e => {
      if (e.buttons) return;
      if (!pending) pending = requestAnimationFrame(() => { pending = null; setHover(partAt(e)); });
    });
    cv.addEventListener('pointerleave', () => setHover(null));
    cv.addEventListener('pointerup', e => {
      if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4) return;
      const p = partAt(e);
      choose(p === chosen ? null : p, false, true);
    });
    cv.addEventListener('dblclick', e => { const p = partAt(e); if (p) choose(p, true, true); });
    root.addEventListener('keydown', e => {
      if (e.key === 'Escape') { choose(null, false, true); $('#b3-find').blur(); }
    });
    info.querySelector('.b3-x').addEventListener('click', () => { info.hidden = true; choose(null, false, true); });
  }

  // a part chosen on the PCB tab is chosen here too, without flying to it
  addEventListener('pv-part', e => {
    if (e.detail.from === 'board3d') return;
    if (!board) { pendingRef = e.detail.ref; return; }
    const p = e.detail.ref ? parts.get(e.detail.ref) : null;
    if (p !== chosen) choose(p || null, false);
  });

  // ---- the part pane (parts.js) ------------------------------------------------
  function pane(p) {
    if (!p) { PV.pane(info, details, null); return; }
    const x = p.userData;
    PV.pane(info, details, x.ref, x.placeholder ? { placeholder: true }
      : { height: height(p, x.side === 'back') });
  }

  // how far the part's model stands off its own face of the laminate, in mm
  function height(p, back) {
    if (!layers.PCB) return null;
    const pcb = new THREE.Box3().setFromObject(layers.PCB[0]);
    const box = new THREE.Box3().setFromObject(p);
    if (box.isEmpty()) return null;
    return 1000 * (back ? pcb.min.y - box.min.y : box.max.y - pcb.max.y);
  }

  function facts(f) {
    if (!f) return;
    const span = PV.span;
    let html = f.modelled + ' of ' + f.parts + ' footprints drawn from their models';
    if (f.no_model.length) html += '; ' + f.no_model.length + ' name no model';
    html += '. ';
    if (f.boxed && f.boxed.length)
      html += '<b class="warn">' + span(f.boxed) + '</b> ' + (f.boxed.length > 1 ? 'have' : 'has') +
        ' no model that meshes and ' + (f.boxed.length > 1 ? 'are' : 'is') +
        ' drawn as a box (the overview says why). ';
    if (f.substituted.length)
      html += span(f.substituted) + (f.substituted.length > 1 ? ' are' : ' is') + ' drawn with a stand-in model. ';
    if (f.flattened && f.flattened.length)
      html += span(f.flattened) + '&rsquo;s model was flattened to mesh. ';
    html += (f.bytes / 1e6).toFixed(1) + '&nbsp;MB, ' + f.triangles.toLocaleString('en') +
      ' triangles, exported ' + f.generated + '. Drag to turn, right-drag to pan, scroll to zoom; ' +
      'click a part for its pane, double-click to fly to it.';
    $('#b3-facts').innerHTML = html;
  }
}

const natural = (a, b) => PV.natural(a, b);
const esc = s => PV.esc(s);
