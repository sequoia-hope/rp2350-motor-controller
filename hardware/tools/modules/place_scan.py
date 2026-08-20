#!/usr/bin/env python3
"""Generic legal-pose scanner/placer on real KiCad geometry (sequential).
For each job: probe footprint (existing ref or new part) is tried at every
STEP/rotation pose inside a window around its anchor. Hard: same-side
courtyard overlap with any other part (DRC-excluded tuck-unders aside), pad
vs other-net pad clearance on a shared copper layer (incl. local clearance),
vias of other nets under a pad, board edge inset. Soft (cost): other-net
track segments crossing a pad (each is copper to rip and reroute). Score =
(rip, distance). The best pose is applied before the next job runs, so jobs
see each other. --apply writes the board and rips the crossed copper."""
import os, sys, math, json, argparse, collections
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
sys.path.insert(0, '/home/sequoia/pcb/rp2350-motor-controller/hardware/tools/jiggle2')
from common import load_board, NM, quiet_stderr
import pcbnew
from geom import poly_dist, bbox, bbox_dist, rect_poly
KLIB = '/usr/share/kicad/footprints'; PARTS = '/home/sequoia/pcb/rp2350-motor-controller/hardware/parts'
CLR = 0.18; HOLECLR = 0.254; EDGE = 0.3; STEP = 0.25; CYMARGIN = 0.06   # DRC inflates courtyard outlines by ~45 um
def mm(v): return int(round(v / NM))
def vec(x, y): return pcbnew.VECTOR2I(mm(x), mm(y))
def pad_box(p, extra=0.0):
    bb = p.GetBoundingBox(); return (bb.GetLeft()*NM-extra, bb.GetTop()*NM-extra, bb.GetRight()*NM+extra, bb.GetBottom()*NM+extra)
def overlap(a, b): return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]
def cy_polys(fp, lay):
    try: fp.BuildCourtyardCaches()
    except AttributeError: pass
    ps = fp.GetCourtyard(lay); out = []
    for i in range(ps.OutlineCount()):
        o = ps.Outline(i); pts = [(o.CPoint(j).x*NM, o.CPoint(j).y*NM) for j in range(o.PointCount())]
        if len(pts) >= 3: out.append((pts, bbox(pts)))
    if not out:
        # courtyard-less footprint (test points, some jumpers): pads' envelope + 0.25 mm on the side it lives on
        side_is_b = (lay == pcbnew.B_CrtYd)
        if fp.IsFlipped() == side_is_b:
            bbs = [pad_box(p) for p in fp.Pads()]
            if bbs:
                bb = (min(b[0] for b in bbs)-0.25, min(b[1] for b in bbs)-0.25, max(b[2] for b in bbs)+0.25, max(b[3] for b in bbs)+0.25)
                out.append((rect_poly(bb), bb))
    return out
def seg_pt_dist(p, a, b):
    ax, ay = a; bx, by = b; px, py = p; dx, dy = bx-ax, by-ay; L2 = dx*dx+dy*dy
    t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((px-ax)*dx+(py-ay)*dy)/L2)); return math.hypot(px-ax-t*dx, py-ay-t*dy)
def seg_box_hit(a, b, hw, box):
    """segment (with halfwidth) intersects axis box?"""
    x0, y0, x1, y1 = box[0]-hw, box[1]-hw, box[2]+hw, box[3]+hw
    if max(a[0], b[0]) < x0 or min(a[0], b[0]) > x1 or max(a[1], b[1]) < y0 or min(a[1], b[1]) > y1: return False
    # clip test: closest point of segment to box centre within half-diagonal is too loose; do proper: any box corner/edge vs segment
    cx, cy = (x0+x1)/2, (y0+y1)/2
    if x0 <= a[0] <= x1 and y0 <= a[1] <= y1: return True
    if x0 <= b[0] <= x1 and y0 <= b[1] <= y1: return True
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    for i in range(4):
        c1, c2 = corners[i], corners[(i+1) % 4]
        # segment-segment intersection
        def orient(p, q, r): return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
        d1, d2 = orient(c1, c2, a), orient(c1, c2, b); d3, d4 = orient(a, b, c1), orient(a, b, c2)
        if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)): return True
    return False
class Scanner:
    def __init__(self, board, excluded_pairs=None):
        self.bd = board
        self.outline = pcbnew.SHAPE_POLY_SET(); board.GetBoardPolygonOutlines(self.outline)
        self.inner = pcbnew.SHAPE_POLY_SET(self.outline); self.inner.Inflate(int(-EDGE/NM), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, int(0.01/NM))
    def collect(self, ax, ay, win, skip_ref):
        near = lambda x, y: abs(x-ax) <= win+6 and abs(y-ay) <= win+6
        crts, fpads, segs, vias = [], [], [], []
        for fp in self.bd.GetFootprints():
            ref = fp.GetReference()
            if ref == skip_ref: continue
            for lay in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
                for pts, bb in cy_polys(fp, lay):
                    if near((bb[0]+bb[2])/2, (bb[1]+bb[3])/2): crts.append((lay, pts, bb, ref))
            try: flc = fp.GetLocalClearance() or 0
            except TypeError: flc = 0
            for p in fp.Pads():
                pb = pad_box(p)
                if not near((pb[0]+pb[2])/2, (pb[1]+pb[3])/2): continue
                try: lc = p.GetLocalClearance() or 0
                except TypeError: lc = 0
                lc = max(lc, flc)*NM
                fpads.append((pb, p.GetNetname(), p.IsOnLayer(pcbnew.F_Cu), p.IsOnLayer(pcbnew.B_Cu), ref, max(lc, CLR), p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH))
        for t in self.bd.GetTracks():
            if t.Type() == pcbnew.PCB_VIA_T:
                p = t.GetPosition(); x, y = p.x*NM, p.y*NM
                if near(x, y): vias.append(((x, y), max(t.GetWidth(pcbnew.PADSTACK.ALL_LAYERS)*NM/2, t.GetDrill()*NM/2 + 0.1), t.GetNetname()))
            else:
                s, e = t.GetStart(), t.GetEnd(); a, b = (s.x*NM, s.y*NM), (e.x*NM, e.y*NM)
                if near(*a) or near(*b): segs.append((t.GetLayer(), a, b, t.GetWidth()*NM/2, t.GetNetname()))
        return crts, fpads, segs, vias
    def scan(self, probe, ax, ay, win, rots, sides, skip_ref, free_nets=(), protect=(), top=6, step=STEP):
        crts, fpads, segs, vias = self.collect(ax, ay, win, skip_ref)
        results = []
        n = int(win/step)
        for i in range(-n, n+1):
            for j in range(-n, n+1):
                x, y = ax+i*step, ay+j*step; d = math.hypot(x-ax, y-ay)
                if d > win: continue
                for back in sides:
                    for deg in rots:
                        probe.SetPosition(vec(x, y))
                        if probe.IsFlipped() != back: probe.Flip(probe.GetPosition(), False)
                        probe.SetOrientationDegrees(deg)
                        want = pcbnew.B_CrtYd if back else pcbnew.F_CrtYd
                        mycy = cy_polys(probe, want)
                        if not mycy: continue
                        bad = False
                        for pts, bb in mycy:
                            for px, py in ((bb[0], bb[1]), (bb[2], bb[1]), (bb[0], bb[3]), (bb[2], bb[3])):
                                if not self.inner.Contains(vec(px, py)): bad = True; break
                            if bad: break
                            for lay, opts, obb, oref in crts:
                                if lay != want or bbox_dist(bb, obb) >= CYMARGIN: continue
                                if poly_dist(pts, opts, bb, obb) < CYMARGIN: bad = True; break
                            if bad: break
                        if bad: continue
                        rip = collections.Counter(); ripset = []
                        for p in probe.Pads():
                            mn = p.GetNetname(); mf = p.IsOnLayer(pcbnew.F_Cu); mbk = p.IsOnLayer(pcbnew.B_Cu)
                            pb0 = pad_box(p)
                            for pb, pn, pf, pbk, oref, oclr, npth in fpads:
                                if (pn == mn and pn) or not ((mf and pf) or (mbk and pbk)): continue
                                if overlap(pad_box(p, oclr), pb): bad = True; break
                            if bad: break
                            for (vx, vy), vr, vn in vias:
                                if vn == mn: continue
                                if pb0[0]-vr-CLR < vx < pb0[2]+vr+CLR and pb0[1]-vr-CLR < vy < pb0[3]+vr+CLR: bad = True; break
                            if bad: break
                            for lay, a, b, hw, sn in segs:
                                if sn == mn or sn in free_nets: continue
                                if (lay == pcbnew.F_Cu and not mf) or (lay == pcbnew.B_Cu and not mbk): continue
                                if lay not in (pcbnew.F_Cu, pcbnew.B_Cu): continue
                                if seg_box_hit(a, b, hw+CLR, pb0): rip[sn] += 1; ripset.append((lay, a, b))
                        if bad or any(k in protect for k in rip): continue
                        results.append((sum(rip.values()), round(d, 3), x, y, deg, back, dict(rip)))
        results.sort(key=lambda r: (r[0], r[1]))
        return results[:top], len(results)
def load_probe(bd, job):
    if job.get('swap'):
        old = next(f for f in bd.GetFootprints() if f.GetReference() == job['ref'])
        job = dict(job, swap=False, new=True, value=old.GetValue(), nets={p.GetNumber(): p.GetNetname() for p in old.Pads()},
                   path=old.GetPath().AsString() if hasattr(old.GetPath(), 'AsString') else None, back=old.IsFlipped())
        extra = [(f.GetName(), f.GetText()) for f in old.GetFields() if f.GetName() not in ('Reference', 'Value', 'Footprint', 'Datasheet', 'Description')]
        bd.Delete(old)
        fp = load_probe(bd, job)
        for k, v in extra:
            fp.SetField(k, v)
            for fld in fp.GetFields():
                if fld.GetName() == k: fld.SetLayer(pcbnew.B_Fab if job['back'] else pcbnew.F_Fab); fld.SetVisible(False); fld.SetMirrored(bool(job['back']))
        return fp
    if job.get('new'):
        fp = pcbnew.FootprintLoad(job['lib'], job['fp'])
        if fp is None: raise SystemExit(f"cannot load {job['lib']}/{job['fp']}")
        bd.Add(fp); fp.SetReference(job['ref']); fp.SetValue(job.get('value', ''))
        try: fp.SetFPIDAsString(job['fpid'])
        except Exception: pass
        for p in fp.Pads():
            nn = job['nets'].get(p.GetNumber())
            if nn:
                n = bd.FindNet(nn)
                if n is None: n = pcbnew.NETINFO_ITEM(bd, nn); bd.Add(n)
                p.SetNet(n)
        if job.get('lcsc'):
            fp.SetField('LCSC', job['lcsc'])
            for fld in fp.GetFields():
                if fld.GetName() == 'LCSC': fld.SetLayer(pcbnew.B_Fab if job['back'] else pcbnew.F_Fab); fld.SetVisible(False); fld.SetMirrored(bool(job['back']))
        if job.get('path'):
            try: fp.SetPath(pcbnew.KIID_PATH(job['path']))
            except Exception as e: print('path set failed', e)
        return fp
    for f in bd.GetFootprints():
        if f.GetReference() == job['ref']: return f
    raise SystemExit(f"{job['ref']} not on board")
def run(board_path, jobs, apply=False, out=None, log=print):
    bd = load_board(board_path); sc = Scanner(bd)
    chosen = {}
    for job in jobs:
        fp = load_probe(bd, job)
        p0 = fp.GetPosition(); x0, y0 = p0.x*NM, p0.y*NM; r0 = fp.GetOrientationDegrees(); b0 = fp.IsFlipped()
        ax, ay = job.get('anchor', (x0, y0))
        sides = (True, False) if job.get('sides_both') else ((job['back'],) if 'back' in job else (b0,))
        best, nposes = sc.scan(fp, ax, ay, job.get('win', 4.0), job.get('rots', (0, 90, 180, 270)), sides, job['ref'],
                               job.get('free', ()), job.get('protect', ()), step=job.get('step', STEP))
        if not best:
            log(f"{job['ref']}: NO legal pose in window {job.get('win',4.0)} around ({ax:.2f},{ay:.2f}) -- leaving at ({x0:.2f},{y0:.2f})")
            if job.get('new') or job.get('swap'): raise SystemExit(f"refusing to continue: new part {job['ref']} has no legal pose")
            fp.SetPosition(vec(x0, y0)); 
            if fp.IsFlipped() != b0: fp.Flip(fp.GetPosition(), False)
            fp.SetOrientationDegrees(r0); chosen[job['ref']] = None; continue
        rip, d, x, y, deg, back, ripd = best[0]
        log(f"{job['ref']:5s}: {nposes:4d} legal poses; best rip={rip} d={d:.2f} -> ({x:.3f},{y:.3f}) rot{deg} {'B' if back else 'F'} {ripd}   alts: " +
            '; '.join(f"rip{b[0]} d{b[1]:.2f} ({b[2]:.2f},{b[3]:.2f}) r{b[4]}" for b in best[1:4]))
        fp.SetPosition(vec(x, y))
        if fp.IsFlipped() != back: fp.Flip(fp.GetPosition(), False)
        fp.SetOrientationDegrees(deg)
        chosen[job['ref']] = dict(x=x, y=y, rot=deg, back=back, rip=rip, d=d, ripnets=ripd, from_=(round(x0, 3), round(y0, 3), r0))
    if apply:
        # rip other-net copper under the placed pads (F/B tracks only; vias were hard constraints)
        placed = [job['ref'] for job in jobs if chosen.get(job['ref'])]
        boxes = []
        for f in bd.GetFootprints():
            if f.GetReference() not in placed: continue
            for p in f.Pads():
                boxes.append((pad_box(p, CLR+0.02), p.GetNetname(), p.IsOnLayer(pcbnew.F_Cu), p.IsOnLayer(pcbnew.B_Cu)))
        n = 0; ripped_nets = collections.Counter()
        for t in list(bd.GetTracks()):
            if t.Type() == pcbnew.PCB_VIA_T: continue
            lay = t.GetLayer()
            if lay not in (pcbnew.F_Cu, pcbnew.B_Cu): continue
            s, e = t.GetStart(), t.GetEnd(); a, b = (s.x*NM, s.y*NM), (e.x*NM, e.y*NM); hw = t.GetWidth()*NM/2
            for box, nn, onf, onb in boxes:
                if t.GetNetname() == nn: continue
                if (lay == pcbnew.F_Cu and not onf) or (lay == pcbnew.B_Cu and not onb): continue
                if seg_box_hit(a, b, hw, box): ripped_nets[t.GetNetname()] += 1; bd.Delete(t); n += 1; break
        log(f'ripped {n} segments under the new pads: {dict(ripped_nets)}')
        with quiet_stderr():
            pcbnew.ZONE_FILLER(bd).Fill(bd.Zones()); pcbnew.SaveBoard(out or board_path, bd)
        log(f'saved {out or board_path}')
    return chosen
if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('board'); ap.add_argument('jobs'); ap.add_argument('--apply', action='store_true'); ap.add_argument('--out')
    a = ap.parse_args()
    jobs = json.load(open(a.jobs))
    for j in jobs:
        if 'anchor' in j: j['anchor'] = tuple(j['anchor'])
        if 'rots' in j: j['rots'] = tuple(j['rots'])
    ch = run(a.board, jobs, a.apply, a.out)
    json.dump(ch, open(os.path.splitext(a.jobs)[0] + '_result.json', 'w'), indent=1)
