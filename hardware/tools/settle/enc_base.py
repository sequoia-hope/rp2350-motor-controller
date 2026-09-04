#!/usr/bin/env python3
"""settle: base boards for the encoder-corner squeeze (F-48 knobs applied).

Starts from data/boards/power_up.kicad_pcb (the squeezed power stage, no
encoder ghosts yet) and writes

    data/boards/enc_v035.kicad_pcb   every 0.5/0.25 via shrunk to 0.35/0.20
                                     (the rules' floor; JLC builds 0.2 holes)
    data/boards/enc_pofv.kicad_pcb   the same, plus via-in-pad inside REGION:
                                     a via within 1.2 mm of a same-net SMD pad
                                     is moved to the pad centre; a stub track
                                     between them is deleted, any other track
                                     ending at the via follows it.

kicad-cli DRC (all severities) is run on each and compared with power_up.
    python3 enc_base.py
"""
import json, os, shutil, subprocess, sys, math
from common import HERE, load_board, quiet_stderr
import pcbnew

BOARDS = os.path.join(HERE, 'configs', 'data', 'boards')
SRC = os.path.join(BOARDS, 'power_up.kicad_pcb')
REGION = (112.4, 96.0, 130.5, 128.5)
NEAR = 1.2      # mm, via centre to pad centre
NM = 1e6


def in_region(p):
    return REGION[0] <= p.x / NM <= REGION[2] and REGION[1] <= p.y / NM <= REGION[3]


def shrink_vias(board):
    """0.5/0.25 -> 0.35/0.20, except a via that is connected only by its
    annulus overlapping something (a same-net track end or pad edge lying
    in the ring between r=0.175 and r=0.25): shrinking it would open the net."""
    tracks = [t for t in board.GetTracks() if t.GetClass() == 'PCB_TRACK']
    pads = [p for f in board.GetFootprints() for p in f.Pads()]
    n, kept = 0, []
    for t in board.GetTracks():
        if t.GetClass() != 'PCB_VIA':
            continue
        if not (round(t.GetWidth() / NM, 3) == 0.5 and round(t.GetDrillValue() / NM, 3) == 0.25):
            continue
        vp = t.GetPosition(); net = t.GetNetname()
        ring = False
        for s in tracks:
            if s.GetNetname() != net:
                continue
            for e in (s.GetStart(), s.GetEnd()):
                d = math.hypot(e.x - vp.x, e.y - vp.y) / NM
                if 0.175 + 0.5 * s.GetWidth() / NM <= d < 0.25 + 0.5 * s.GetWidth() / NM:
                    ring = True
        for p in pads:
            if p.GetNetname() != net or abs(p.GetPosition().x - vp.x) > 3 * NM or abs(p.GetPosition().y - vp.y) > 3 * NM:
                continue
            sh = p.GetEffectiveShape(p.GetLayer())
            big = sh.Collide(pcbnew.SHAPE_CIRCLE(vp, int(0.25 * NM)), 0)
            small = sh.Collide(pcbnew.SHAPE_CIRCLE(vp, int(0.175 * NM)), 0)
            if big and not small:
                ring = True
        if ring:
            kept.append((net, round(vp.x / NM, 2), round(vp.y / NM, 2)))
            continue
        t.SetWidth(int(0.35 * NM)); t.SetDrill(int(0.20 * NM)); n += 1
    if kept:
        print(f'   kept at 0.5/0.25 (annulus-connected): {kept}', flush=True)
    return n


def pofv(board):
    """Move a via into the same-net SMD pad it feeds, when the via's only
    attachments are stub tracks ending in that pad (or nothing at all: a
    plane-stitch via beside the pad), and the new position clears every
    other-net pad/track by rule + guard (copper 0.35 dia, hole 0.20)."""
    fps = list(board.GetFootprints())
    pads = [p for f in fps for p in f.Pads()]
    smd = [p for p in pads if p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD]
    tracks = [t for t in board.GetTracks() if t.GetClass() == 'PCB_TRACK']
    vias = [t for t in board.GetTracks() if t.GetClass() == 'PCB_VIA']
    holes = [(p.GetPosition(), p.GetDrillSize().x) for p in pads if p.GetDrillSize().x > 0] + \
            [(v.GetPosition(), v.GetDrillValue()) for v in vias]
    CLR, HCLR, H2H, GUARD = 0.10, 0.254, 0.254, 0.02
    moved, deleted, skipped = [], 0, {'attached': 0, 'clearance': 0, 'hole': 0}
    used = set()

    def near(a, b, mm):
        return abs(a.x - b.x) < mm * NM and abs(a.y - b.y) < mm * NM

    for v in vias:
        vp = v.GetPosition()
        if not in_region(vp):
            continue
        best = None
        for p in smd:
            if p.GetNetname() != v.GetNetname():
                continue
            if p.HitTest(vp):
                best = None; break
            d = math.hypot(vp.x - p.GetPosition().x, vp.y - p.GetPosition().y) / NM
            if d < NEAR and (best is None or d < best[0]):
                best = (d, p)
        if not best or id(best[1]) in used:
            continue
        d, pad = best
        new = pad.GetPosition()
        touching = [t for t in tracks if t.GetNetname() == v.GetNetname() and
                    (near(t.GetStart(), vp, 0.001) or near(t.GetEnd(), vp, 0.001))]
        stubs = []
        ok = True
        for t in touching:
            other = t.GetEnd() if near(t.GetStart(), vp, 0.001) else t.GetStart()
            if pad.HitTest(other) or near(other, new, 0.05):
                stubs.append(t)
            else:
                ok = False
        if not ok:
            skipped['attached'] += 1; continue
        # geometric pre-check at the new position
        cu = pcbnew.SHAPE_CIRCLE(pcbnew.VECTOR2I(new.x, new.y), int(0.175 * NM))
        hole = pcbnew.SHAPE_CIRCLE(pcbnew.VECTOR2I(new.x, new.y), int(0.10 * NM))
        clash = False
        for p in pads:
            if p.GetNetname() == v.GetNetname():
                continue
            if not near(p.GetPosition(), new, 3.0):
                continue
            sh = p.GetEffectiveShape(p.GetLayer())
            if sh.Collide(cu, int((CLR + GUARD) * NM)) or sh.Collide(hole, int((HCLR + GUARD) * NM)):
                clash = True; break
        if not clash:
            for t in tracks:
                if t.GetNetname() == v.GetNetname():
                    continue
                if not (near(t.GetStart(), new, 3.0) or near(t.GetEnd(), new, 3.0)):
                    continue
                sh = t.GetEffectiveShape()
                if sh.Collide(cu, int((CLR + GUARD) * NM)) or sh.Collide(hole, int((HCLR + GUARD) * NM)):
                    clash = True; break
        if clash:
            skipped['clearance'] += 1; continue
        if any(not near(hp, vp, 0.001) and
               math.hypot(hp.x - new.x, hp.y - new.y) - (hd + 0.20 * NM) / 2 < (H2H + GUARD) * NM
               for hp, hd in holes if near(hp, new, 3.0)):
            skipped['hole'] += 1; continue
        used.add(id(pad))
        for t in stubs:
            board.Delete(t); tracks.remove(t); deleted += 1
        v.SetPosition(pcbnew.VECTOR2I(new.x, new.y))
        moved.append((v.GetNetname(), pad.GetParentFootprint().GetReference(), pad.GetNumber(), round(d, 2)))
    print(f'   POFV skipped: {skipped}', flush=True)
    return moved, deleted, 0


def drc(path):
    out = path.replace('.kicad_pcb', '_drc.json')
    subprocess.run(['kicad-cli', 'pcb', 'drc', '--format', 'json', '--units', 'mm', '--severity-all',
                    '-o', out, path], check=True, capture_output=True)
    d = json.load(open(out))
    from collections import Counter
    c = Counter((x['type'], x['severity']) for x in d['violations'] if not x.get('excluded'))
    return len(d['unconnected_items']), c


def emit(name, do_pofv):
    with quiet_stderr():
        board = pcbnew.LoadBoard(SRC)
    n = shrink_vias(board)
    msg = f'{name}: {n} vias 0.5/0.25 -> 0.35/0.20'
    if do_pofv:
        moved, deleted, followed = pofv(board)
        msg += f'; POFV {len(moved)} vias into pads, {deleted} stubs deleted'
        json.dump(moved, open(os.path.join(BOARDS, name + '_pofv.json'), 'w'), indent=0)
    dst = os.path.join(BOARDS, name + '.kicad_pcb')
    with quiet_stderr():
        pcbnew.SaveBoard(dst, board)
    for ext in ('.kicad_pro', '.kicad_prl'):
        if os.path.exists(SRC.replace('.kicad_pcb', ext)):
            shutil.copy(SRC.replace('.kicad_pcb', ext), dst.replace('.kicad_pcb', ext))
    print(msg, flush=True)
    return dst


if __name__ == '__main__':
    base_u, base_c = drc(SRC)
    print(f'power_up: {base_u} unconnected; errors {sum(n for (t, s), n in base_c.items() if s == "error")}, '
          f'warnings {sum(n for (t, s), n in base_c.items() if s == "warning")}')
    for name, p in (('enc_v035', False),) + ((('enc_pofv', True),) if '--pofv' in sys.argv else ()):
        dst = emit(name, p)
        u, c = drc(dst)
        new = {k: v - base_c.get(k, 0) for k, v in c.items() if v - base_c.get(k, 0) > 0}
        gone = {k: base_c[k] - c.get(k, 0) for k in base_c if base_c[k] - c.get(k, 0) > 0}
        print(f'   {name}: {u} unconnected (base {base_u}); new {dict(new) or "none"}; gone {dict(gone) or "none"}', flush=True)
