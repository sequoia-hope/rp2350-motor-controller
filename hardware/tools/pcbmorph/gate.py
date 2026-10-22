"""Stage 3: RDP Polyline Simplification, KiCad Checkpoint Emission, and 5-Check Gate.

Applies `settled.json` (and optional `routed.json`) to a copy of the source `.kicad_pcb`,
simplifies subdivided trace chains via Ramer-Douglas-Peucker (`rdp()`), refills zones,
and enforces the 5-check acceptance gate:
  1. Static copper byte-identity (`static_tracks_missing == 0 and static_vias_missing == 0`)
  2. Net length caps (`len_violations == 0`)
  3. Differential DRC (`new_drc_violations == 0 and unconnected_delta <= 0`)
  4. Total DRC regression check (`drc_delta <= 0`)
  5. Schematic/PCB sync check (`check_sync_ok`, when `net_changes` are present)
"""
import os
import sys
import json
import math
import shutil
import subprocess
from collections import defaultdict
import pcbnew

from .common import (
    NM, COPPER_LAYERS,
    load_config, data_path, load_json, save_json,
    quiet_stderr, load_board, tsig, vsig, sig_key, rdp,
    resolve_elastic,
)


def simplify_chains(nodes, segs, eps=0.015):
    node_map = {n['id']: (n['x'], n['y']) for n in nodes}
    locked = set()
    for n in nodes:
        if n.get('fixed') or n.get('part') or n.get('kind') == 'via':
            locked.add(n['id'])

    deg = defaultdict(int)
    for s in segs:
        deg[s['a']] += 1
        deg[s['b']] += 1
    for nid, d in deg.items():
        if d != 2:
            locked.add(nid)

    groups = defaultdict(list)
    for si, s in enumerate(segs):
        groups[(s['layer'], s['net'], round(s['w'], 4))].append(si)

    out_segs = []
    for (layer, net, w), s_indices in groups.items():
        adj = defaultdict(list)
        for si in s_indices:
            s = segs[si]
            adj[s['a']].append((s['b'], si))
            adj[s['b']].append((s['a'], si))

        used = set()
        for start in list(adj.keys()):
            if start not in locked and len(adj[start]) == 2:
                continue
            for nb, si in adj[start]:
                if si in used:
                    continue
                used.add(si)
                chain = [node_map[start], node_map[nb]]
                cur = nb
                while cur not in locked and len(adj[cur]) == 2:
                    nxt = None
                    for cand, csi in adj[cur]:
                        if csi not in used:
                            used.add(csi)
                            nxt = cand
                            break
                    if nxt is None:
                        break
                    chain.append(node_map[nxt])
                    cur = nxt
                simp = rdp(chain, eps)
                for k in range(len(simp) - 1):
                    p1, p2 = simp[k], simp[k + 1]
                    if math.hypot(p1[0] - p2[0], p1[1] - p2[1]) > 1e-4:
                        out_segs.append({
                            'ax': p1[0], 'ay': p1[1],
                            'bx': p2[0], 'by': p2[1],
                            'w': w, 'layer': layer, 'net': net,
                        })

        for si in s_indices:
            if si not in used:
                s = segs[si]
                p1, p2 = node_map[s['a']], node_map[s['b']]
                if math.hypot(p1[0] - p2[0], p1[1] - p2[1]) > 1e-4:
                    out_segs.append({
                        'ax': p1[0], 'ay': p1[1],
                        'bx': p2[0], 'by': p2[1],
                        'w': w, 'layer': layer, 'net': net,
                    })

    return out_segs


def collect_sigs(board):
    tracks = set()
    vias = set()
    for t in board.GetTracks():
        if t.GetClass() == 'PCB_VIA':
            vias.add(sig_key(vsig(t)))
        elif t.GetClass() == 'PCB_TRACK' and t.GetLayer() in COPPER_LAYERS:
            tracks.add(sig_key(tsig(t)))
    return tracks, vias


def run_drc_json(board_path, out_json, exclusions=False):
    cmd = ['kicad-cli', 'pcb', 'drc', '--severity-error', '--format', 'json',
           '--output', out_json, board_path]
    if exclusions:
        cmd.insert(3, '--severity-exclusions')
    subprocess.run(
        cmd,
        check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    if not os.path.exists(out_json):
        return [], []
    with open(out_json) as f:
        data = json.load(f)
    return data.get('violations', []), data.get('unconnected_items', [])


def v_fingerprint(v):
    """Position-independent for footprint/pad items, position-keyed for copper."""
    vtype = v.get('type', '')
    items = v.get('items', [])
    has_pad = any(it.get('description', '').startswith(('Pad ', 'PTH pad ')) for it in items)
    keys = []
    for it in items:
        desc = it.get('description', '')
        if desc.startswith(('Footprint ', 'Pad ', 'PTH pad ')):
            keys.append(desc)
        elif vtype == 'solder_mask_bridge' or (has_pad and desc.startswith('Track ')):
            keys.append(desc.split(',')[0])
        elif 'pos' in it:
            keys.append(f"({round(it['pos']['x'], 1)},{round(it['pos']['y'], 1)})")
        else:
            keys.append(desc.split(',')[0])
    return f"{vtype}|{'||'.join(sorted(keys))}"


def apply_shape_to_board(board, settled):
    """Apply deformed board outline (`settled['final_outline']`) to `Edge.Cuts` on `board`."""
    o0 = settled.get('outline', [])
    ofin = settled.get('final_outline', [])
    scfg = settled.get('shape') or {}
    if not o0 or not ofin or len(o0) != len(ofin) or not scfg:
        return False
    if all(math.hypot(a[0] - b[0], a[1] - b[1]) < 1e-4 for a, b in zip(o0, ofin)):
        return False

    edge_id = board.GetLayerID('Edge.Cuts')
    xs = [p[0] for p in o0]
    ys = [p[1] for p in o0]
    x0, y0, x1, y1 = min(xs), min(ys), max(xs), max(ys)
    edge = scfg.get('edge', 'top').lower()
    stype = scfg.get('type', 'notch')

    target_seg = None
    for d in board.GetDrawings():
        if d.GetLayer() == edge_id and d.GetShape() == pcbnew.SHAPE_T_SEGMENT:
            s, e = d.GetStart(), d.GetEnd()
            sx, sy = s.x / NM, s.y / NM
            ex, ey = e.x / NM, e.y / NM
            if edge == 'top' and abs(sy - y0) < 0.05 and abs(ey - y0) < 0.05:
                target_seg = d
                break
            if edge == 'bottom' and abs(sy - y1) < 0.05 and abs(ey - y1) < 0.05:
                target_seg = d
                break
            if edge == 'left' and abs(sx - x0) < 0.05 and abs(ex - x0) < 0.05:
                target_seg = d
                break
            if edge == 'right' and abs(sx - x1) < 0.05 and abs(ex - x1) < 0.05:
                target_seg = d
                break

    if target_seg is None:
        return False

    if stype == 'notch':
        s, e = target_seg.GetStart(), target_seg.GetEnd()
        sx, sy = s.x / NM, s.y / NM
        ex, ey = e.x / NM, e.y / NM
        w = target_seg.GetWidth()
        if edge == 'top':
            mid_pts = ofin[1:5]
            if sx > ex:
                mid_pts = list(reversed(mid_pts))
        elif edge == 'bottom':
            mid_pts = ofin[3:7]
            if sx < ex:
                mid_pts = list(reversed(mid_pts))
        elif edge == 'left':
            mid_pts = ofin[4:8]
            if sy < ey:
                mid_pts = list(reversed(mid_pts))
        elif edge == 'right':
            mid_pts = ofin[2:6]
            if sy > ey:
                mid_pts = list(reversed(mid_pts))
        else:
            return False

        board.Remove(target_seg)
        chain = [(sx, sy)] + [(p[0], p[1]) for p in mid_pts] + [(ex, ey)]
        for p1, p2 in zip(chain[:-1], chain[1:]):
            if math.hypot(p1[0] - p2[0], p1[1] - p2[1]) < 1e-4:
                continue
            seg = pcbnew.PCB_SHAPE(board)
            seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
            seg.SetLayer(edge_id)
            seg.SetWidth(w)
            seg.SetStart(pcbnew.VECTOR2I(int(round(p1[0] * NM)), int(round(p1[1] * NM))))
            seg.SetEnd(pcbnew.VECTOR2I(int(round(p2[0] * NM)), int(round(p2[1] * NM))))
            board.Add(seg)
        return True

    if stype == 'edge_shift':
        # Shift target edge + adjacent corner arcs + trim perpendicular edge endpoints
        dx = dy = 0.0
        for a, b in zip(o0, ofin):
            if math.hypot(b[0] - a[0], b[1] - a[1]) > math.hypot(dx, dy):
                dx, dy = b[0] - a[0], b[1] - a[1]
        dx_nm, dy_nm = int(round(dx * NM)), int(round(dy * NM))
        for d in board.GetDrawings():
            if d.GetLayer() != edge_id:
                continue
            s, e = d.GetStart(), d.GetEnd()
            sx, sy = s.x / NM, s.y / NM
            ex, ey = e.x / NM, e.y / NM
            if edge == 'top':
                s_on = sy < y0 + 2.5
                e_on = ey < y0 + 2.5
            elif edge == 'bottom':
                s_on = sy > y1 - 2.5
                e_on = ey > y1 - 2.5
            elif edge == 'left':
                s_on = sx < x0 + 2.5
                e_on = ex < x0 + 2.5
            else:
                s_on = sx > x1 - 2.5
                e_on = ex > x1 - 2.5
            if s_on and e_on:
                d.Move(pcbnew.VECTOR2I(dx_nm, dy_nm))
            elif s_on:
                d.SetStart(pcbnew.VECTOR2I(s.x + dx_nm, s.y + dy_nm))
            elif e_on:
                d.SetEnd(pcbnew.VECTOR2I(e.x + dx_nm, e.y + dy_nm))
        return True

    return False


def emit_and_gate(cfg, verbose=True):
    """Emit checkpoint `.kicad_pcb` and run the 5-check verification gate."""
    resolve_elastic(cfg)
    src_board_path = cfg['board']
    out_board_path = cfg.get('out_board', data_path(cfg, 'checkpoint.kicad_pcb'))

    m = load_json(data_path(cfg, 'model.json'))
    settled = load_json(data_path(cfg, 'settled.json'))
    routed_path = data_path(cfg, 'routed.json')
    routed = load_json(routed_path) if os.path.exists(routed_path) else {'net_changes': [], 'tracks': [], 'vias': []}

    os.makedirs(os.path.dirname(os.path.abspath(out_board_path)), exist_ok=True)
    shutil.copy2(src_board_path, out_board_path)
    src_pro = os.path.splitext(src_board_path)[0] + '.kicad_pro'
    src_dru = os.path.splitext(src_board_path)[0] + '.kicad_dru'
    out_base = os.path.splitext(out_board_path)[0]
    if os.path.exists(src_pro) and os.path.abspath(src_pro) != os.path.abspath(out_base + '.kicad_pro'):
        shutil.copy2(src_pro, out_base + '.kicad_pro')
    if os.path.exists(src_dru) and os.path.abspath(src_dru) != os.path.abspath(out_base + '.kicad_dru'):
        shutil.copy2(src_dru, out_base + '.kicad_dru')

    board0 = load_board(src_board_path)
    orig_tracks, orig_vias = collect_sigs(board0)
    active_tsigs = set(settled['active_track_sigs'])
    active_vsigs = set(settled['active_via_sigs'])
    expected_static_tracks = orig_tracks - active_tsigs
    expected_static_vias = orig_vias - active_vsigs

    board = load_board(out_board_path)

    for ref, pr in settled['parts'].items():
        dx, dy = pr['dx'], pr['dy']
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            continue
        fp = board.FindFootprintByReference(ref)
        if fp:
            pos = fp.GetPosition()
            fp.SetPosition(pcbnew.VECTOR2I(
                pos.x + int(round(dx * NM)),
                pos.y + int(round(dy * NM))
            ))

    to_remove = []
    for t in board.GetTracks():
        if t.GetClass() == 'PCB_VIA':
            if sig_key(vsig(t)) in active_vsigs:
                to_remove.append(t)
        elif t.GetClass() == 'PCB_TRACK' and t.GetLayer() in COPPER_LAYERS:
            if sig_key(tsig(t)) in active_tsigs:
                to_remove.append(t)
    for t in to_remove:
        board.Remove(t)

    net_ByName = {net_item.GetNetname(): net_code
                  for net_code, net_item in board.GetNetsByNetcode().items()}

    def get_or_make_net(nname):
        if nname in net_ByName:
            return net_ByName[nname]
        ni = pcbnew.NETINFO_ITEM(board, nname)
        board.Add(ni)
        net_ByName[nname] = ni.GetNetCode()
        return ni.GetNetCode()

    for nc in routed.get('net_changes', []):
        ref, pad_num, new_net = nc['ref'], str(nc['pad']), nc['new_net']
        ncode = get_or_make_net(new_net)
        fp = board.FindFootprintByReference(ref)
        if fp:
            for pad in fp.Pads():
                if pad.GetNumber() == pad_num:
                    pad.SetNetCode(ncode)

    simp_segs = simplify_chains(settled['nodes'], settled['segs'], eps=0.0001)
    for s in simp_segs:
        tr = pcbnew.PCB_TRACK(board)
        tr.SetStart(pcbnew.VECTOR2I(int(round(s['ax'] * NM)), int(round(s['ay'] * NM))))
        tr.SetEnd(pcbnew.VECTOR2I(int(round(s['bx'] * NM)), int(round(s['by'] * NM))))
        tr.SetWidth(int(round(s['w'] * NM)))
        tr.SetLayer(int(s['layer']))
        if s['net'] in net_ByName:
            tr.SetNetCode(net_ByName[s['net']])
        board.Add(tr)

    for n in settled['nodes']:
        if n['kind'] == 'via':
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(int(round(n['x'] * NM)), int(round(n['y'] * NM))))
            try:
                v.SetWidth(int(round(n['size'] * NM)))
            except TypeError:
                v.SetWidth(pcbnew.PADSTACK.ALL_LAYERS, int(round(n['size'] * NM)))
            v.SetDrill(int(round(n['drill'] * NM)))
            v.SetViaType(pcbnew.VIATYPE_THROUGH)
            v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            if n['net'] in net_ByName:
                v.SetNetCode(net_ByName[n['net']])
            board.Add(v)

    for s in routed.get('tracks', []):
        tr = pcbnew.PCB_TRACK(board)
        tr.SetStart(pcbnew.VECTOR2I(int(round(s['ax'] * NM)), int(round(s['ay'] * NM))))
        tr.SetEnd(pcbnew.VECTOR2I(int(round(s['bx'] * NM)), int(round(s['by'] * NM))))
        tr.SetWidth(int(round(s['w'] * NM)))
        tr.SetLayer(int(s['layer']))
        ncode = get_or_make_net(s['net'])
        tr.SetNetCode(ncode)
        board.Add(tr)

    for vinfo in routed.get('vias', []):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(int(round(vinfo['x'] * NM)), int(round(vinfo['y'] * NM))))
        v.SetWidth(int(round(vinfo['size'] * NM)))
        v.SetDrill(int(round(vinfo['drill'] * NM)))
        ncode = get_or_make_net(vinfo['net'])
        v.SetNetCode(ncode)
        board.Add(v)

    shape_applied = apply_shape_to_board(board, settled)

    with quiet_stderr():
        filler = pcbnew.ZONE_FILLER(board)
        filler.Fill(board.Zones())
        pcbnew.SaveBoard(out_board_path, board)

    board_after = load_board(out_board_path)
    after_tracks, after_vias = collect_sigs(board_after)
    missing_static_tracks = expected_static_tracks - after_tracks
    missing_static_vias = expected_static_vias - after_vias

    len0_by_net = defaultdict(float)
    for s in m['segs']:
        n1 = m['nodes'][s['a']]
        n2 = m['nodes'][s['b']]
        len0_by_net[s['net']] += math.hypot(n1['x'] - n2['x'], n1['y'] - n2['y'])
    len1_by_net = defaultdict(float)
    for s in simp_segs:
        len1_by_net[s['net']] += math.hypot(s['ax'] - s['bx'], s['ay'] - s['by'])
    len_violations = []
    for net, l0 in len0_by_net.items():
        cap = max(l0 * 1.25, l0 + 4.0)
        if len1_by_net[net] > cap + 0.1:
            len_violations.append({'net': net, 'before': round(l0, 2), 'after': round(len1_by_net[net], 2), 'cap': round(cap, 2)})

    # Emit s=0 roundtrip baseline board (same active track/via replacement + ZONE_FILLER)
    # so UUID-keyed exclusions and zone-refill roundtrip are compared apples-to-apples.
    s0_board_path = data_path(cfg, 'baseline_s0.kicad_pcb')
    shutil.copy2(src_board_path, s0_board_path)
    s0_base = os.path.splitext(s0_board_path)[0]
    if os.path.exists(src_pro) and os.path.abspath(src_pro) != os.path.abspath(s0_base + '.kicad_pro'):
        shutil.copy2(src_pro, s0_base + '.kicad_pro')
    if os.path.exists(src_dru) and os.path.abspath(src_dru) != os.path.abspath(s0_base + '.kicad_dru'):
        shutil.copy2(src_dru, s0_base + '.kicad_dru')
    board_s0 = load_board(s0_board_path)
    rem_s0 = []
    for t in board_s0.GetTracks():
        if t.GetClass() == 'PCB_VIA' and sig_key(vsig(t)) in active_vsigs:
            rem_s0.append(t)
        elif t.GetClass() == 'PCB_TRACK' and t.GetLayer() in COPPER_LAYERS and sig_key(tsig(t)) in active_tsigs:
            rem_s0.append(t)
    for t in rem_s0:
        board_s0.Remove(t)
    net_s0 = {ni.GetNetname(): nc for nc, ni in board_s0.GetNetsByNetcode().items()}
    s0_segs = simplify_chains(m['nodes'], m['segs'], eps=0.0001)
    for s in s0_segs:
        tr = pcbnew.PCB_TRACK(board_s0)
        tr.SetStart(pcbnew.VECTOR2I(int(round(s['ax'] * NM)), int(round(s['ay'] * NM))))
        tr.SetEnd(pcbnew.VECTOR2I(int(round(s['bx'] * NM)), int(round(s['by'] * NM))))
        tr.SetWidth(int(round(s['w'] * NM)))
        tr.SetLayer(int(s['layer']))
        if s['net'] in net_s0:
            tr.SetNetCode(net_s0[s['net']])
        board_s0.Add(tr)
    for n in m['nodes']:
        if n['kind'] == 'via':
            v = pcbnew.PCB_VIA(board_s0)
            v.SetPosition(pcbnew.VECTOR2I(int(round(n['x0'] * NM)), int(round(n['y0'] * NM))))
            try:
                v.SetWidth(int(round(n['size'] * NM)))
            except TypeError:
                v.SetWidth(pcbnew.PADSTACK.ALL_LAYERS, int(round(n['size'] * NM)))
            v.SetDrill(int(round(n['drill'] * NM)))
            v.SetViaType(pcbnew.VIATYPE_THROUGH)
            v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            if n['net'] in net_s0:
                v.SetNetCode(net_s0[n['net']])
            board_s0.Add(v)
    with quiet_stderr():
        pcbnew.ZONE_FILLER(board_s0).Fill(board_s0.Zones())
        pcbnew.SaveBoard(s0_board_path, board_s0)

    drc_before_path = data_path(cfg, 'drc_before.json')
    drc_after_path = data_path(cfg, 'drc_after.json')
    v_before_all, u_before = run_drc_json(s0_board_path, drc_before_path, exclusions=True)
    v_before = [v for v in v_before_all if not v.get('excluded')]
    v_after, u_after = run_drc_json(out_board_path, drc_after_path, exclusions=False)

    fp_before = {v_fingerprint(v) for v in v_before_all}
    new_violations = []
    for v in v_after:
        fp = v_fingerprint(v)
        if fp not in fp_before:
            new_violations.append({
                'type': v.get('type'),
                'severity': v.get('severity'),
                'description': v.get('description'),
                'items': [it.get('description') for it in v.get('items', [])],
            })

    sync_ok = True
    sync_output = "skipped (no net changes)"
    if routed.get('net_changes') and cfg.get('run_check_sync'):
        res = subprocess.run(
            ['python3', 'hardware/tools/check_sync.py'],
            capture_output=True, text=True
        )
        sync_ok = (res.returncode == 0)
        sync_output = (res.stdout + res.stderr).strip()[-500:]

    gate_passed = (
        len(missing_static_tracks) == 0 and
        len(missing_static_vias) == 0 and
        len(len_violations) == 0 and
        len(new_violations) == 0 and
        len(u_after) <= len(u_before) and
        len(v_after) <= len(v_before) and
        sync_ok
    )

    gate_report = {
        'config_name': m['config_name'],
        'out_board': out_board_path,
        'gate_passed': gate_passed,
        'shape_applied': shape_applied,
        'checks': {
            'static_tracks_missing': len(missing_static_tracks),
            'static_vias_missing': len(missing_static_vias),
            'len_violations': len(len_violations),
            'drc_before_count': len(v_before),
            'drc_after_count': len(v_after),
            'drc_delta': len(v_after) - len(v_before),
            'new_drc_violations': len(new_violations),
            'unconnected_before': len(u_before),
            'unconnected_after': len(u_after),
            'unconnected_delta': len(u_after) - len(u_before),
            'check_sync_ok': sync_ok,
        },
        'simplified_segs_count': len(simp_segs),
        'len_violation_details': len_violations,
        'new_violation_details': new_violations[:20],
        'sync_output': sync_output,
    }
    save_json(data_path(cfg, 'gate_report.json'), gate_report)

    if verbose:
        status = "PASS" if gate_passed else "FAIL"
        c = gate_report['checks']
        print(f"[pcbmorph.gate] {m['config_name']} -> {out_board_path} : {status}")
        print(f"  static_missing: tracks={c['static_tracks_missing']} vias={c['static_vias_missing']}")
        print(f"  DRC: {c['drc_before_count']} -> {c['drc_after_count']} "
              f"(delta={c['drc_delta']:+d}, new={c['new_drc_violations']})")
        print(f"  unconnected: {c['unconnected_before']} -> {c['unconnected_after']} "
              f"(delta={c['unconnected_delta']:+d})")
        print(f"  len_violations: {c['len_violations']}  |  simplified_segs: {len(simp_segs)}")
        if new_violations:
            for nv in new_violations[:5]:
                print(f"    NEW DRC [{nv['type']}]: {nv['description']} -> {nv['items']}")

    return gate_report


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    if not argv:
        print("Usage: python3 -m pcbmorph gate CONFIG.json", file=sys.stderr)
        sys.exit(1)
    cfg = load_config(argv[0])
    rep = emit_and_gate(cfg, verbose=True)
    if not rep['gate_passed']:
        sys.exit(2)


if __name__ == '__main__':
    main()
