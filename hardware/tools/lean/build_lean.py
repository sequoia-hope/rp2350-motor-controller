#!/usr/bin/env python3
"""Build the rev-b-lean board: the shipped rev-A copper + the rev-B schematic
+ Tier A of review/shrink.html, without the rev-b-fixes re-route.

Runs on the board AFTER hardware/tools/revb/sync_pcb.py --apply (F-27/F-39:
Q5, U29, D31, C79, U11 pad 5, TP15/TP17), which is stage 1 of the build.
Everything here is driven by the schematic netlist (kicad-cli export) so the
acceptance test, hardware/tools/revb/check_sync.py, is the spec.

Stages (each logged to build_log.txt):

  A  net renames        Net-(U18-A) -> /ENC_A_TRX_A ... Net-(U8-GPIO38) -> /TERM_SW
                        copper keeps its identity, only the label changes
  B  pad re-nets        R82-R85 pad 2 -> /TERM_x_R, D1 pad 3 -> /LED_VDD: the
                        tracks touching those pads on the old net are cut (all
                        four A-lines pass THROUGH the resistor pad, so the cut
                        leaves one airwire per channel for the routing pass)
  C  values             the 15 value-only changes (FETs, TVS ratings)
  D  footprint swaps    every footprint whose schematic footprint is one of the
                        rev-b-lean library parts (_JLC tight courtyards, F-43
                        0402 shrink, snubbers, courtyard-free test points,
                        WS2812B-2020).  Position, rotation, side, path, fields
                        and pad nets carry over; copper that no longer reaches
                        a smaller pad is re-bridged with a stub (shrink_0402.py
                        method).  D1 is rotated -90 because the 2020's pin
                        order runs one position around from the 3528's.
  E  new parts          the 33 rev-B parts. Each is tried at its rev-b-fixes
                        pose, then in a 3 mm / 0.25 mm / 4-rotation window
                        around it; a pose is accepted only with zero conflicts
                        (same-side courtyard, other-net pad, other-net track or
                        via under a pad, board edge). Anything else is PARKED
                        below the board edge, grouped, on its intended side.
                        Rev-A copper is never ripped for a new part.
  F  paths              schematic links (KIID paths) set from the netlist for
                        every footprint, so KiCad's own netlist update agrees
  G  refill + save

  usage: build_lean.py [--pcb BOARD] [--dry-run]
"""
import os, sys, re, json, math, argparse, subprocess, tempfile, collections
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(HW, 'tools', 'jiggle2'))
from common import load_board, NM, quiet_stderr, seg_seg_dist   # noqa: E402
import pcbnew                                                    # noqa: E402

BOARD = os.path.join(HW, 'rp2350_driver.kicad_pcb')
SCH = os.path.join(HW, 'rp2350_driver.kicad_sch')
PARTS = os.path.join(HW, 'parts')
KLIB = '/usr/share/kicad/footprints'
POSES = os.path.join(HERE, 'revb_fixes_poses.json')
CLR = 0.18
EDGE = 0.3
STEP = 0.25
WINDOW = 3.0

NET_RENAMES = {
    'Net-(U18-A)': '/ENC_A_TRX_A', 'Net-(U18-B)': '/ENC_A_TRX_B',
    'Net-(U19-A)': '/ENC_B_TRX_A', 'Net-(U19-B)': '/ENC_B_TRX_B',
    'Net-(U20-A)': '/ENC_C_TRX_A', 'Net-(U20-B)': '/ENC_C_TRX_B',
    'Net-(U21-A)': '/ENC_D_TRX_A', 'Net-(U21-B)': '/ENC_D_TRX_B',
    'Net-(U8-GPIO38)': '/TERM_SW',
}
PAD_CUTS = [  # (ref, pad, new net) -- copper on the old net touching the pad is cut
    ('R82', '2', '/TERM_A_R'), ('R83', '2', '/TERM_B_R'),
    ('R84', '2', '/TERM_C_R'), ('R85', '2', '/TERM_D_R'),
    ('D1', '3', '/LED_VDD'),
]
VALUE_REFS = 'AH1 AL1 BH1 BL1 CH1 CH2 CL1 CL2 D5 D9 D13 D17 D21'.split()
ROT_DELTA = {'D1': -90}          # see stage D in the docstring
OUR_PARTS = re.compile(r'^parts:(.*_JLC|TestPoint_Pad_D1\.[05]mm_no_courtyard|'
                       r'LED_WS2812B-2020_PLCC4_2\.0x2\.0mm_MotorBuck)$')
STD_FIELDS = {'Reference', 'Value', 'Footprint', 'Datasheet', 'Description'}
PARK_GROUPS = [
    ('encoder-termination', 'U30 U31 U32 U33 R97 R98 R99 R100 C110 C111 C112 C113 C114 '
                            'R101 R102 R103 R104 R105 R106 R107 R108 R109 R110 R111 R112'),
    ('e-stop', 'Q6 R113'), ('led', 'D30'), ('can-termination', 'JP2 R114 R115 C115'),
]


def nm(v):
    return int(round(v / NM))


def vec(x, y):
    return pcbnew.VECTOR2I(nm(x), nm(y))


# --- netlist -----------------------------------------------------------
def read_netlist():
    with tempfile.TemporaryDirectory() as td:
        xf = os.path.join(td, 'n.xml')
        subprocess.run(['kicad-cli', 'sch', 'export', 'netlist', '--format', 'kicadxml',
                        '-o', xf, SCH], check=True, capture_output=True)
        root = ET.parse(xf).getroot()
    comps, pins = {}, collections.defaultdict(dict)
    for c in root.find('components'):
        props = {p.get('name'): p.get('value') for p in c.findall('property')}
        comps[c.get('ref')] = dict(value=c.findtext('value'), fpid=c.findtext('footprint'),
                                   tstamp=c.findtext('tstamps'), dnp='dnp' in props,
                                   lcsc=props.get('LCSC'))
    for n in root.find('nets'):
        for nd in n.findall('node'):
            pins[nd.get('ref')][nd.get('pin')] = n.get('name')
    return comps, pins


def lib_dir(fpid):
    lib, name = fpid.split(':', 1)
    return (PARTS if lib == 'parts' else f'{KLIB}/{lib}.pretty'), name


# --- board helpers -----------------------------------------------------
def fp_by_ref(bd, ref):
    for f in bd.GetFootprints():
        if f.GetReference() == ref:
            return f
    return None


def net(bd, name):
    n = bd.FindNet(name)
    if n is None:
        n = pcbnew.NETINFO_ITEM(bd, name)
        bd.Add(n)
    return n


def track_pts(t):
    if t.Type() == pcbnew.PCB_VIA_T:
        p = t.GetPosition()
        return (p.x * NM, p.y * NM), (p.x * NM, p.y * NM)
    s, e = t.GetStart(), t.GetEnd()
    return (s.x * NM, s.y * NM), (e.x * NM, e.y * NM)


def pad_box(pad, extra=0.0):
    bb = pad.GetBoundingBox()
    return (bb.GetLeft() * NM - extra, bb.GetTop() * NM - extra,
            bb.GetRight() * NM + extra, bb.GetBottom() * NM + extra)


def overlap(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def seg_rect_dist(a, b, r):
    """Distance between segment a-b and axis-aligned rect r (0 if it enters)."""
    x0, y0, x1, y1 = r
    for p in (a, b):
        if x0 <= p[0] <= x1 and y0 <= p[1] <= y1:
            return 0.0
    edges = (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
             ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0)))
    return min(seg_seg_dist(a, b, e0, e1) for e0, e1 in edges)


def copy_field_placement(old, new, name):
    src = dst = None
    for f in old.GetFields():
        if f.GetName() == name:
            src = f
    for f in new.GetFields():
        if f.GetName() == name:
            dst = f
    if src is None or dst is None:
        return
    dst.SetPosition(src.GetPosition()); dst.SetLayer(src.GetLayer())
    dst.SetVisible(src.IsVisible()); dst.SetMirrored(src.IsMirrored())
    dst.SetTextAngle(src.GetTextAngle()); dst.SetTextSize(src.GetTextSize())
    dst.SetTextThickness(src.GetTextThickness())


def park_field(fp, name, back):
    for f in fp.GetFields():
        if f.GetName() == name:
            f.SetLayer(pcbnew.B_Fab if back else pcbnew.F_Fab)
            f.SetVisible(False); f.SetMirrored(back)


def swap(bd, ref, fpid, log, rot_delta=0):
    """Replace footprint `ref` by library part `fpid` in place (shrink_0402 method)."""
    old = fp_by_ref(bd, ref)
    lib, name = lib_dir(fpid)
    with quiet_stderr():
        new = pcbnew.FootprintLoad(lib, name)
    if new is None:
        raise SystemExit(f'cannot load {fpid}')
    pos = old.GetPosition(); rot = old.GetOrientationDegrees(); back = old.IsFlipped()
    nets = {p.GetNumber(): p.GetNetname() for p in old.Pads()}
    extra = [(f.GetName(), f.GetText()) for f in old.GetFields() if f.GetName() not in STD_FIELDS]
    old_fpid = old.GetFPIDAsString()
    bd.Add(new)
    new.SetPosition(pos)
    if back:
        new.Flip(pos, False)
    new.SetOrientationDegrees(rot + rot_delta)
    new.SetReference(ref); new.SetValue(old.GetValue())
    new.SetFPIDAsString(fpid); new.SetPath(old.GetPath())
    for fn in ('Reference', 'Value'):
        copy_field_placement(old, new, fn)
    for k, v in extra:
        new.SetField(k, v); park_field(new, k, back)
    missing = []
    for pad in new.Pads():
        nn = nets.get(pad.GetNumber())
        if nn is None:
            missing.append(pad.GetNumber()); continue
        pad.SetNet(net(bd, nn))
    if missing:
        raise SystemExit(f'{ref}: no net for pad(s) {missing} in {fpid}')
    bd.Delete(old)
    log(f'   {ref:5s} {old_fpid:40s} -> {fpid} rot {rot + rot_delta:g}')
    return new


def landed_on(bd, refs):
    pads = [(fp.GetReference(), p) for fp in bd.GetFootprints()
            if fp.GetReference() in refs for p in fp.Pads()]
    on = {}
    for t in bd.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T:
            continue
        for ref, pad in pads:
            if t.GetNetCode() != pad.GetNetCode():
                continue
            for idx, end in enumerate((t.GetStart(), t.GetEnd())):
                if pad.HitTest(end):
                    on[(t.m_Uuid.AsString(), idx, ref, pad.GetNumber())] = (
                        pad.GetNetname(), t.GetLayer(), end.x, end.y, t.GetWidth())
    return on


def reconnect(bd, before, after, log):
    pads = {(fp.GetReference(), p.GetNumber()): p
            for fp in bd.GetFootprints() for p in fp.Pads()}
    want = {}
    for key in set(before) - set(after):
        _u, _i, ref, num = key
        _n, layer, x, y, width = before[key]
        k = (ref, num, layer, x, y)
        want[k] = max(width, want.get(k, 0))
    added = 0
    for (ref, num, layer, x, y), width in sorted(want.items()):
        pad = pads.get((ref, num))
        if pad is None:
            continue
        tgt = pad.GetPosition()
        if tgt.x == x and tgt.y == y:
            continue
        # never wider than the pad it lands on: an 0805 snubber fed by 0.8 mm
        # phase copper would otherwise grow two stubs 0.16 mm apart at 0402
        width = min(width, pad.GetSize().x, pad.GetSize().y)
        t = pcbnew.PCB_TRACK(bd)
        t.SetStart(pcbnew.VECTOR2I(x, y)); t.SetEnd(tgt)
        t.SetLayer(layer); t.SetWidth(width); t.SetNet(pad.GetNet())
        bd.Add(t); added += 1
        log(f'   stub {ref}.{num} [{pad.GetNetname()}] {bd.GetLayerName(layer)} '
            f'({x*NM:.3f},{y*NM:.3f}) -> pad, w={width*NM:.2f}')
    return added


def rip_near_pads(bd, pads, log, why=''):
    """Delete every track/via of ANOTHER net that KiCad's own shape test says
    collides with one of `pads` at the board clearance.  Used after a pad
    changes net (stage B) or size (stage D): copper that was legal against
    the old pad and is not against the new one becomes an airwire for the
    routing pass, never a silent short."""
    n = 0
    clr = nm(CLR)
    for t in list(bd.GetTracks()):
        via = t.Type() == pcbnew.PCB_VIA_T
        lay = None if via else t.GetLayer()
        if lay is not None and lay not in (pcbnew.F_Cu, pcbnew.B_Cu):
            continue
        # cheap reject before the exact test
        a, b = track_pts(t)
        hw = (t.GetWidth(pcbnew.F_Cu) if via else t.GetWidth()) / 2 * NM
        for pad in pads:
            if t.GetNetCode() == pad.GetNetCode():
                continue
            layers = [pcbnew.F_Cu, pcbnew.B_Cu] if via else [lay]
            hit = False
            for L in layers:
                if not pad.IsOnLayer(L):
                    continue
                if seg_rect_dist(a, b, pad_box(pad)) >= hw + CLR + 0.05:
                    continue
                if pad.GetEffectiveShape(L).Collide(t.GetEffectiveShape(L), clr):
                    hit = True; break
            if hit:
                log(f'   rip {"via" if via else bd.GetLayerName(lay)} [{t.GetNetname()}] '
                    f'{"" if via else f"{math.hypot(b[0]-a[0], b[1]-a[1]):.2f} mm "}'
                    f'at ({a[0]:.3f},{a[1]:.3f}): {why}')
                bd.Delete(t); n += 1
                break
    return n


# --- placement feasibility ----------------------------------------------
class Scene:
    """Static geometry of the board for conflict checks (built once)."""

    def __init__(self, bd):
        outline = pcbnew.SHAPE_POLY_SET()
        bd.GetBoardPolygonOutlines(outline)
        self.inner = pcbnew.SHAPE_POLY_SET(outline)
        self.inner.Inflate(nm(-EDGE), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, nm(0.01))
        self.crts, self.pads, self.segs, self.vias = [], [], [], []
        self.refresh(bd)

    def refresh(self, bd):
        self.crts, self.pads = [], []
        for fp in bd.GetFootprints():
            ref = fp.GetReference()
            for lay in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
                poly = fp.GetCourtyard(lay)
                if poly and poly.OutlineCount():
                    b = poly.BBox()
                    self.crts.append((ref, lay, (b.GetLeft() * NM, b.GetTop() * NM,
                                                 b.GetRight() * NM, b.GetBottom() * NM)))
            for p in fp.Pads():
                self.pads.append((ref, p.GetNumber(), pad_box(p), p.GetNetname(),
                                  p.IsOnLayer(pcbnew.F_Cu), p.IsOnLayer(pcbnew.B_Cu)))
        self.segs, self.vias = [], []
        for t in bd.GetTracks():
            a, b = track_pts(t)
            if t.Type() == pcbnew.PCB_VIA_T:
                self.vias.append((a, t.GetWidth(pcbnew.F_Cu) / 2 * NM, t.GetNetname()))
            else:
                self.segs.append((t.GetLayer(), a, b, t.GetWidth() / 2 * NM, t.GetNetname()))

    def conflicts(self, fp, limit=6):
        ref = fp.GetReference(); back = fp.IsFlipped()
        want = pcbnew.B_CrtYd if back else pcbnew.F_CrtYd
        out = []
        poly = fp.GetCourtyard(want)
        if not (poly and poly.OutlineCount()):
            return ['no courtyard']
        b = poly.BBox()
        c = (b.GetLeft() * NM, b.GetTop() * NM, b.GetRight() * NM, b.GetBottom() * NM)
        for px, py in ((c[0], c[1]), (c[2], c[1]), (c[0], c[3]), (c[2], c[3])):
            if not self.inner.Contains(vec(px, py)):
                return ['edge']
        for oref, lay, box in self.crts:
            if oref != ref and lay == want and overlap(c, box):
                out.append(f'courtyard:{oref}')
                if len(out) >= limit:
                    return out
        for p in fp.Pads():
            mb = pad_box(p); mbc = pad_box(p, CLR); mn = p.GetNetname()
            mf, mbk = p.IsOnLayer(pcbnew.F_Cu), p.IsOnLayer(pcbnew.B_Cu)
            for oref, num, pb, pn, pf, pbk in self.pads:
                if oref == ref or (pn == mn and pn) or not ((mf and pf) or (mbk and pbk)):
                    continue
                if overlap(mbc, pb):
                    out.append(f'pad:{oref}.{num}')
                    if len(out) >= limit:
                        return out
            for lay, a, bb, hw, sn in self.segs:
                if sn == mn:
                    continue
                if (lay == pcbnew.F_Cu and not mf) or (lay == pcbnew.B_Cu and not mbk):
                    continue
                if lay not in (pcbnew.F_Cu, pcbnew.B_Cu):
                    continue
                if seg_rect_dist(a, bb, mb) < hw + CLR:
                    out.append(f'track:{sn}')
                    if len(out) >= limit:
                        return out
            for a, r, sn in self.vias:
                if sn == mn:
                    continue
                if seg_rect_dist(a, a, mb) < r + CLR:
                    out.append(f'via:{sn}')
                    if len(out) >= limit:
                        return out
        return out


def place_new(bd, ref, fpid, value, nets, pose, lcsc, log):
    lib, name = lib_dir(fpid)
    with quiet_stderr():
        fp = pcbnew.FootprintLoad(lib, name)
    if fp is None:
        raise SystemExit(f'cannot load {fpid}')
    bd.Add(fp)
    fp.SetReference(ref); fp.SetValue(value); fp.SetFPIDAsString(fpid)
    x, y, rot, back = pose
    fp.SetPosition(vec(x, y))
    if back:
        fp.Flip(fp.GetPosition(), False)
    fp.SetOrientationDegrees(rot)
    if lcsc:
        fp.SetField('LCSC', lcsc); park_field(fp, 'LCSC', back)
    for pad in fp.Pads():
        nn = nets.get(pad.GetNumber())
        if nn:
            pad.SetNet(net(bd, nn))
    return fp


def set_pose(fp, x, y, rot, back):
    fp.SetPosition(vec(x, y))
    if fp.IsFlipped() != back:
        fp.Flip(fp.GetPosition(), False)
    fp.SetOrientationDegrees(rot)


# --- main ----------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcb', default=BOARD)
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()
    lines = []

    def log(s):
        print(s); lines.append(s)

    comps, pins = read_netlist()
    poses = {p['ref']: p for p in json.load(open(POSES))}
    bd = load_board(a.pcb)
    log(f'loaded {a.pcb}: {len(list(bd.GetFootprints()))} footprints, '
        f'{len(list(bd.GetTracks()))} tracks/vias; netlist {len(comps)} components')

    # A. net renames -------------------------------------------------------
    log('A. net renames')
    for old, new in NET_RENAMES.items():
        on = bd.FindNet(old)
        if on is None:
            log(f'   {old}: not on board (skip)'); continue
        nn = net(bd, new); k = 0
        for t in bd.GetTracks():
            if t.GetNetCode() == on.GetNetCode():
                t.SetNet(nn); k += 1
        for fp in bd.GetFootprints():
            for p in fp.Pads():
                if p.GetNetCode() == on.GetNetCode():
                    p.SetNet(nn); k += 1
        for z in bd.Zones():
            if z.GetNetCode() == on.GetNetCode():
                z.SetNet(nn); k += 1
        log(f'   {old} -> {new}: {k} items')

    # B. pad re-nets with cuts --------------------------------------------
    log('B. pad re-nets (copper on the old net touching the pad is cut)')
    for ref, num, new in PAD_CUTS:
        fp = fp_by_ref(bd, ref)
        pad = [p for p in fp.Pads() if p.GetNumber() == num][0]
        old = pad.GetNetname()
        pad.SetNet(net(bd, new))
        cut = rip_near_pads(bd, [pad], log, why=f'{ref}.{num} {old} -> {new}')
        log(f'   {ref}.{num}: {old} -> {new}, {cut} segment(s) cut')

    # C. values ------------------------------------------------------------
    log('C. values')
    for ref in VALUE_REFS:
        fp = fp_by_ref(bd, ref)
        if fp.GetValue() != comps[ref]['value']:
            log(f'   {ref}: {fp.GetValue()} -> {comps[ref]["value"]}')
            fp.SetValue(comps[ref]['value'])

    # D. footprint swaps ---------------------------------------------------
    log('D. footprint swaps to the rev-b-lean library parts')
    todo = []
    for fp in bd.GetFootprints():
        ref = fp.GetReference()
        c = comps.get(ref)
        if c and OUR_PARTS.match(c['fpid'] or '') and fp.GetFPIDAsString() != c['fpid']:
            todo.append((ref, c['fpid']))
    before = landed_on(bd, {r for r, _ in todo})
    geom0 = {fp.GetReference(): sorted(pad_box(p) for p in fp.Pads())
             for fp in bd.GetFootprints()}
    for ref, fpid in sorted(todo):
        swap(bd, ref, fpid, log, ROT_DELTA.get(ref, 0))
    after = landed_on(bd, {r for r, _ in todo})
    lost = sorted(set(before) - set(after))
    stubs = reconnect(bd, before, after, log)
    log(f'   {len(todo)} swapped; {len(before)} endpoints were on-pad, '
        f'{len(lost)} stranded, {stubs} stub(s) added')
    # pads that changed shape may now sit on copper that cleared the old pad
    # (a trace between an 0603's pads no longer fits between an 0402's)
    changed = [fp for fp in bd.GetFootprints()
               if fp.GetReference() in dict(todo)
               and sorted(pad_box(p) for p in fp.Pads()) != geom0.get(fp.GetReference())]
    ripped = 0
    for fp in changed:
        ripped += rip_near_pads(bd, list(fp.Pads()), log, why=f'{fp.GetReference()} pads changed')
    log(f'   {len(changed)} footprint(s) changed pad geometry; {ripped} foreign track(s)/via(s) ripped')

    # E. new parts ---------------------------------------------------------
    log('E. new parts')
    have = {fp.GetReference() for fp in bd.GetFootprints()}
    new_refs = [r for r in comps if r not in have]
    scene = Scene(bd)
    placed, parked = [], []
    for ref in sorted(new_refs, key=lambda r: (re.sub(r'\d', '', r), int(re.sub(r'\D', '', r) or 0))):
        c = comps[ref]; p = poses.get(ref)
        if p is None:
            log(f'   {ref}: no rev-b-fixes pose; parking'); parked.append(ref); continue
        fp = place_new(bd, ref, c['fpid'], c['value'], pins[ref],
                       (p['x'], p['y'], p['rot'], p['back']), c['lcsc'], log)
        conf = scene.conflicts(fp)
        if not conf:
            log(f'   {ref:5s} at rev-b-fixes pose ({p["x"]:.3f},{p["y"]:.3f}) rot {p["rot"]:g} '
                f'{"B" if p["back"] else "F"}: clean')
            placed.append(ref); scene.refresh(bd); continue
        first = conf
        best = None
        n = int(WINDOW / STEP)
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                x, y = p['x'] + i * STEP, p['y'] + j * STEP
                d = math.hypot(x - p['x'], y - p['y'])
                if d > WINDOW or (best and d >= best[0]):
                    continue
                for rot in (p['rot'], p['rot'] + 90, p['rot'] + 180, p['rot'] + 270):
                    set_pose(fp, x, y, rot, p['back'])
                    if not scene.conflicts(fp, limit=1):
                        best = (d, x, y, rot); break
        if best:
            d, x, y, rot = best
            set_pose(fp, x, y, rot, p['back'])
            log(f'   {ref:5s} moved {d:.2f} mm to ({x:.3f},{y:.3f}) rot {rot:g}: clean '
                f'(rev-b-fixes pose had {first})')
            placed.append(ref); scene.refresh(bd)
        else:
            log(f'   {ref:5s} no clean pose within {WINDOW} mm ({first}); parking')
            parked.append(ref)

    # park below the board, grouped, on the intended side
    bb = bd.GetBoardEdgesBoundingBox()
    x_left = bb.GetLeft() * NM + 1.5; y_row = bb.GetBottom() * NM + 3.0
    groups = collections.OrderedDict((g, r.split()) for g, r in PARK_GROUPS)
    for g, refs in groups.items():
        row = [r for r in refs if r in parked]
        if not row:
            continue
        x = x_left; ymax = 0
        for ref in row:
            fp = fp_by_ref(bd, ref); p = poses[ref]
            set_pose(fp, x, y_row, 0, p['back'])
            cb = fp.GetCourtyard(pcbnew.B_CrtYd if p['back'] else pcbnew.F_CrtYd).BBox()
            w = cb.GetWidth() * NM; h = cb.GetHeight() * NM
            fp.SetPosition(vec(x + w / 2, y_row + h / 2))
            x += w + 1.0; ymax = max(ymax, h)
        log(f'   parked {g}: {" ".join(row)} at y={y_row:.1f}')
        y_row += ymax + 2.0
    stray = [r for r in parked if r not in sum(groups.values(), [])]
    if stray:
        log(f'   parked ungrouped: {stray}')
    log(f'   placed {len(placed)}, parked {len(parked)}')

    # F. paths -------------------------------------------------------------
    log('F. schematic paths')
    k = 0
    for fp in bd.GetFootprints():
        c = comps.get(fp.GetReference())
        if not c or not c['tstamp']:
            continue
        want = '/' + c['tstamp']
        if fp.GetPath().AsString() != want:
            fp.SetPath(pcbnew.KIID_PATH(want)); k += 1
    log(f'   {k} path(s) set')

    # G. refill + save -----------------------------------------------------
    with quiet_stderr():
        pcbnew.ZONE_FILLER(bd).Fill(bd.Zones())
    log(f'G. refilled {len(list(bd.Zones()))} zones')
    if a.dry_run:
        log('dry run - not saved')
    else:
        with quiet_stderr():
            pcbnew.SaveBoard(a.pcb, bd)
        log(f'saved {a.pcb}')
    with open(os.path.join(HERE, 'build_log.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
