/* gesture.js — pointer gestures for the pan-and-zoom plates (the PCB and SCH
 * tabs; the 3D tab's TrackballControls does its own).
 *
 * One pointer drags to pan; a tap (a press that did not move) is a click, and
 * two taps in quick succession a double. Two pointers pinch: the view scales by
 * the change in the distance between them, about their midpoint, and pans with
 * the midpoint — so a two-finger drag still pans. The plates set
 * `touch-action: none`, which tells the browser not to pinch the page itself;
 * without this it also meant a second finger was ignored, or yanked the pan.
 *
 *   PV.gestures(plate, {
 *     pan(dx, dy),        // move by this much, plate px, incremental
 *     zoom(f, cx, cy),    // scale by f about (cx, cy), plate px
 *     click(x, y), dblclick(x, y),
 *     move(x, y),         // a hover: the pointer is over (x, y) with nothing pressed
 *     start(), end(),     // a drag or pinch begins / the last pointer lifts
 *   })
 */
(() => {
  const PV = window.PV = window.PV || {};
  PV.gestures = (plate, h) => {
    const pts = new Map();          // pointerId -> {x, y} in client px
    let one = null;                 // the single-pointer drag: where it started, whether it moved
    let two = null;                 // the pinch: {d, cx, cy} at the last event
    let lastTap = null;             // {t, x, y}: for the double tap
    const local = (x, y) => { const r = plate.getBoundingClientRect(); return [x - r.left, y - r.top]; };
    const pair = () => {
      const [a, b] = [...pts.values()];
      return { d: Math.max(1, Math.hypot(b.x - a.x, b.y - a.y)), cx: (a.x + b.x) / 2, cy: (a.y + b.y) / 2 };
    };

    plate.addEventListener('pointerdown', e => {
      if (e.button > 0) return;
      pts.set(e.pointerId, { x: e.clientX, y: e.clientY });
      plate.setPointerCapture(e.pointerId);
      if (pts.size === 1) {
        one = { x: e.clientX, y: e.clientY, moved: false };
      } else if (pts.size === 2) {
        // the second finger ends the drag (no tap can come of it) and starts the pinch
        one = null; two = pair();
        h.start?.();
      }
    });

    plate.addEventListener('pointermove', e => {
      if (pts.has(e.pointerId)) pts.set(e.pointerId, { x: e.clientX, y: e.clientY });
      if (two && pts.size >= 2) {
        const p = pair();
        const [cx, cy] = local(two.cx, two.cy);
        h.zoom?.(p.d / two.d, cx, cy);
        h.pan?.(p.cx - two.cx, p.cy - two.cy);
        two = p;
        return;
      }
      if (one) {
        if (!one.moved && Math.hypot(e.clientX - one.x, e.clientY - one.y) > 4) {
          one.moved = true;
          h.start?.();
        }
        if (one.moved) {
          h.pan?.(e.clientX - one.x, e.clientY - one.y);
          one.x = e.clientX; one.y = e.clientY;
        }
        return;
      }
      if (!pts.size) h.move?.(...local(e.clientX, e.clientY));
    });

    const lift = e => {
      if (!pts.has(e.pointerId)) return;
      pts.delete(e.pointerId);
      if (plate.hasPointerCapture(e.pointerId)) plate.releasePointerCapture(e.pointerId);
      if (pts.size >= 2) { two = pair(); return; }
      if (pts.size === 1) {
        // one finger of the pinch left: the other goes on as a plain drag, from where it is
        const [p] = pts.values();
        two = null; one = { x: p.x, y: p.y, moved: true };
        return;
      }
      const was = one; one = two = null;
      if (was && !was.moved && e.type === 'pointerup') {
        const [x, y] = local(e.clientX, e.clientY);
        const now = performance.now();
        if (lastTap && now - lastTap.t < 350 && Math.hypot(x - lastTap.x, y - lastTap.y) < 30) {
          lastTap = null;
          h.dblclick?.(x, y);
        } else {
          lastTap = { t: now, x, y };
          h.click?.(x, y);
        }
      }
      h.end?.();
    };
    plate.addEventListener('pointerup', lift);
    plate.addEventListener('pointercancel', lift);
  };
})();
