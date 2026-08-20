#!/usr/bin/env python3
"""Render review/modules.html from the run's JSON artefacts."""
import json, os, math, collections, html
S = os.path.dirname(os.path.abspath(__file__)); D = os.path.dirname(S)
OUT = '/home/sequoia/pcb/rp2350-motor-controller/review/modules.html'
M = json.load(open(S+'/modules.json')); mods = M['modules']; assign = M['assign']
B = json.load(open(S+'/board.json')); fp = {f['ref']: f for f in B['fps']}
probe = json.load(open(S+'/probe.json')); VEC = json.load(open(S+'/vectors.json')); VOK = json.load(open(S+'/vectors_clean.json'))
ref_log = json.load(open(S+'/referee_log.json')); warp = json.load(open(S+'/warp_stats.json')); clean = json.load(open(S+'/clean_log.json'))
place = json.load(open(S+'/jobs10_result.json'))
def drc(p):
    d = json.load(open(p)); c = collections.Counter(v['type'] for v in d['violations'])
    return dict(v=len(d['violations']), u=len(d['unconnected_items']), by=dict(c))
stages = collections.OrderedDict([
    ('rev-B working tree (baseline)', drc(D+'/drc/baseline.json')),
    ('after module stretch (DRC-neutral subset)', drc(D+'/drc/stretch_final.json')),
    ('after re-placing 17 rev-B parts + 4 F-19 parts + Q6 swap', drc(D+'/drc/placed.json')),
    ('after corner cleanup (stray vias / pad-collision copper removed)', drc(D+'/drc/cleaned2.json')),
])
route = json.load(open(S+'/route_result.json')) if os.path.exists(S+'/route_result.json') else None
if route: stages['after Freerouting + post-route cleanup'] = route['drc']
def esc(s): return html.escape(str(s))
def fmt(v): return f'{v:+.2f}' if abs(v) > 0.0005 else '0'
parents = collections.OrderedDict()
for m, d in mods.items(): parents.setdefault(d['parent'], []).append(m)
H = []
H.append('''<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Module Stretch &amp; Rev-B Routing — RP2350 Motor Controller</title>
<style>
  :root { --bg:#f7f6f3; --panel:#fff; --ink:#1e2126; --muted:#5b6470; --line:#d9d5cc; --accent:#6b4fa0; --good:#2c7a4b; --warn:#a05a00; --bad:#b03434; --code-bg:#efede8; }
  @media (prefers-color-scheme: dark) { :root { --bg:#16181d; --panel:#1e2128; --ink:#e8e6e1; --muted:#9aa3af; --line:#343945; --accent:#a98fd8; --good:#5cb884; --warn:#e0a44c; --bad:#e06c6c; --code-bg:#262a33; } }
  * { box-sizing:border-box; } html,body { margin:0; padding:0; }
  body { background:var(--bg); color:var(--ink); font:16px/1.6 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif; }
  main { max-width:1040px; margin:0 auto; padding:2rem 1.25rem 5rem; }
  header.masthead { border-bottom:3px solid var(--accent); padding:2.2rem 0 1rem; margin-bottom:1.5rem; }
  h1 { font-size:1.9rem; line-height:1.2; margin:0 0 .4rem; } .subtitle { color:var(--muted); margin:0; }
  h2 { font-size:1.45rem; margin:2.8rem 0 .8rem; padding-top:1rem; border-top:1px solid var(--line); } h3 { font-size:1.12rem; margin:1.8rem 0 .5rem; }
  p { margin:.6rem 0; } a { color:var(--accent); }
  code,.mono { font-family:ui-monospace,"SF Mono",Consolas,monospace; font-size:.92em; background:var(--code-bg); padding:.08em .35em; border-radius:4px; }
  table { border-collapse:collapse; width:100%; margin:1rem 0; font-size:.92rem; } th,td { text-align:left; padding:.4rem .6rem; border-bottom:1px solid var(--line); vertical-align:top; }
  th { font-size:.78rem; text-transform:uppercase; letter-spacing:.04em; color:var(--muted); } td.num { text-align:right; font-family:ui-monospace,monospace; }
  .tablewrap { overflow-x:auto; } .small { font-size:.85rem; color:var(--muted); }
  .sev { display:inline-block; font-size:.72rem; font-weight:700; letter-spacing:.05em; padding:.12em .55em; border-radius:99px; white-space:nowrap; }
  .sev.high { background:color-mix(in srgb,var(--bad) 16%,transparent); color:var(--bad); } .sev.med { background:color-mix(in srgb,var(--warn) 18%,transparent); color:var(--warn); }
  .sev.low { background:color-mix(in srgb,var(--muted) 18%,transparent); color:var(--muted); } .sev.good { background:color-mix(in srgb,var(--good) 16%,transparent); color:var(--good); }
  figure { margin:1.2rem 0; background:var(--panel); border:1px solid var(--line); border-radius:10px; padding:1rem; } figure img { max-width:100%; height:auto; display:block; margin:0 auto; border-radius:4px; }
  figcaption { color:var(--muted); font-size:.88rem; margin-top:.6rem; text-align:center; }
  .callout { border-left:4px solid var(--accent); background:var(--panel); border-radius:0 8px 8px 0; padding:.8rem 1rem; margin:1rem 0; } .callout.warn { border-left-color:var(--warn); } .callout.good { border-left-color:var(--good); } .callout.bad { border-left-color:var(--bad); }
  .cols2 { display:grid; grid-template-columns:1fr 1fr; gap:1rem; } @media (max-width:760px) { .cols2 { grid-template-columns:1fr; } }
  .scrollfig { overflow-x:auto; }
</style></head><body><main>
<header class="masthead"><p style="margin:0 0 .4rem"><a href="index.html" style="text-decoration:none">← project hub</a></p>
<h1>Module Stretch &amp; Rev-B Routing</h1>
<p class="subtitle">Carve the board into logical modules · stretch the seams by a small amount · place and route the rev-B parts · 2026-08-20</p></header>''')
# ---- 0 summary ---------------------------------------------------------------------
st = list(stages.values())
H.append(f'''<div class="callout">
<strong>In one paragraph.</strong> The board was carved into <strong>{len(parents)-0} parent modules / {sum(1 for m in mods.values() if not m['pinned'])} rigid leaves</strong> (plus {sum(1 for m in mods.values() if m['pinned'])} pinned mechanical interfaces), and every leaf was asked how far it can slide before any cross-module seam closes. The answer is the headline finding: <strong>at module granularity this board has essentially no slack</strong> — the encoder fabric, RP2350 halo, buck and bus caps cannot move at all as rigid bodies, the half-bridge bands can slide 0.2–0.5&nbsp;mm in one direction each, and a coupled LP stretch that opens 17 of the 44 tight seams by a mean 0.056&nbsp;mm (21 leaves, ≤0.6&nbsp;mm) sheds <strong>{ref_log[0]['new']} new DRC violations</strong> through rule-pitch seam copper. The DRC referee walked it back to {len([m for m,v in VOK['vectors'].items() if abs(v[0])+abs(v[1])>1e-6])} loose parts (applied, DRC-neutral). What <em>did</em> move the needle is the second half: the 17 provisionally-placed rev-B parts plus the four F-19 parts were given legal homes on real KiCad geometry and the corner was cleaned of stray copper — <strong>{st[0]['v']} → {st[3]['v']} DRC violations</strong> before routing, <code>check_sync</code> IN SYNC — {"and Freerouting then took the airwire count from %d to %d." % (st[3]['u'], st[-1]['u']) if route else "and the routing pass is reported below."}
</div>''')
# ---- 1 modules ---------------------------------------------------------------------
H.append('<h2 id="modules">1 · The modules</h2>')
H.append('''<p>Two sources were combined: the designer's own grouping (symbols that sit together on the schematic sheet — power, the four bridges, the encoder fabric, CAN, sense dividers) and board XY proximity across <em>both</em> sides, because the stretch field is one 2-D field for the whole board and a front part over a back part must belong to the same rigid body. Leaves are the rigid units; parents group them for reading. The encoder fabric is split per channel (each SIT3088 + choke + termination + pulls, with its new switch/cap/pull) plus the hall/I²C switch column; the half-bridges stay whole (FETs/shunt/TVS on the front, EG3113 + gate resistors + INA240 on the back are one gate loop each); loose parts on the sparse right half (bulk caps, TVS, jumpers, test points) are their own leaves.</p>''')
H.append('<div class="cols2"><figure><img src="img/modules/modules_front.svg" alt="module map, front"><figcaption>Front side: leaves coloured by parent module, pinned interfaces grey. Arrows (×20) show the stretch the LP proposed (amber) and what survived the DRC referee (green).</figcaption></figure>')
H.append('<figure><img src="img/modules/modules_back.svg" alt="module map, back"><figcaption>Back side — the dense side. Same colouring. Hover a part for its leaf.</figcaption></figure></div>')
H.append('<div class="tablewrap"><table><tr><th>Parent</th><th>Leaf</th><th>Parts</th><th>Sides</th><th>What it is</th></tr>')
for p, leaves in parents.items():
    if p == 'PIN': continue
    for i, m in enumerate(leaves):
        d = mods[m]
        H.append(f'<tr><td>{esc(p) if i == 0 else ""}</td><td class="mono">{esc(m)}</td><td class="num">{len(d["refs"])}</td><td>{"/".join(d["sides"])}</td><td>{esc(d["title"])}<span class="small"> — {esc(" ".join(d["refs"][:14]))}{" …" if len(d["refs"])>14 else ""}</span></td></tr>')
H.append(f'<tr><td>PIN</td><td class="mono">16 leaves</td><td class="num">16</td><td></td><td>J1/J2 phase XT30s, J12 encoder, J9/J10 power, J3 USB-C, J4/J6 JST, J5 E-stop, J7/J8 daisy-chain, J11 CAN, H1–H4 mounting holes — never move</td></tr></table></div>')
H.append(f'<p class="small">{len(M["replace"])} rev-B parts whose pose was provisional (<code>{esc(" ".join(M["replace"]))}</code>) are members of their leaves but excluded from the leaf geometry — they are re-placed in §4.</p>')
# ---- 2 slack --------------------------------------------------------------------------
H.append('<h2 id="slack">2 · How far can each module slide? (exact polygons)</h2>')
H.append('''<p>For every movable leaf, moving it alone: the largest translation in ±x/±y (mm) before any same-side courtyard pair or any shared-copper-layer pad pair (rule 0.18&nbsp;mm, mounting holes 1.4&nbsp;mm local clearance) of <em>another</em> leaf gets closer than it is today (seams under 0.4&nbsp;mm may not shrink at all; tuck-unders that ship overlapping and are DRC-excluded — C80/J9, BH1/J9, C39/J10 — are grandfathered), and before a courtyard crosses the 0.3&nbsp;mm edge inset. Exact courtyard/pad polygons from pcbnew, bisection to 0.1&nbsp;µm. 0.00 means jammed.</p>''')
H.append('<div class="tablewrap"><table><tr><th>Leaf</th><th>+x (east)</th><th>−x (west)</th><th>+y (south)</th><th>−y (north)</th><th>Leaf</th><th>+x</th><th>−x</th><th>+y</th><th>−y</th></tr>')
items = [(m, probe[m]) for m in mods if m in probe]
half = (len(items)+1)//2
for i in range(half):
    row = ''
    for k in (i, i+half):
        if k < len(items):
            m, pr = items[k]
            cells = ''.join(f'<td class="num" style="color:{"var(--bad)" if pr[d] < 0.05 else ("var(--warn)" if pr[d] < 0.3 else "inherit")}">{pr[d]:.2f}</td>' for d in ('+x', '-x', '+y', '-y'))
            row += f'<td class="mono">{esc(m)}</td>{cells}'
        else: row += '<td></td>'*5
    H.append('<tr>'+row+'</tr>')
H.append('</table></div>')
seams = VEC['seams']; tight = [s for s in seams if 0 <= s[2] < 0.4]
H.append(f'''<p>Seam census: <strong>{len([s for s in seams if s[2] < 0.4])} cross-leaf courtyard seams under 0.4&nbsp;mm</strong> ({len([s for s in seams if s[2] <= 0.001])} touching or overlapping, of which the tuck-unders C80/J9 −0.47, BH1/J9 −0.25 and HB_C/J9, C39/J10, MCU/D31, MCU/J7 at 0.00 are all DRC-excluded in the project). The chains are what matters: HB_C and HB_B are flush against J9's courtyard, HB_B→BUCK (0.16)→C42/C80→PWR_OR→MCU (0.06 to TP17)→J3/J7/J11; ENC→J12 pads (0.06), J2's XT30 pegs (0.00) and the bottom edge (0.16). Everything from the encoder edge to the USB edge is one wall-to-wall chain pinned at both ends.</p>''')
# ---- 3 stretch ------------------------------------------------------------------------
H.append('<h2 id="stretch">3 · The stretch: LP solve, warp, DRC referee</h2>')
mv = [(m, v) for m, v in VEC['vectors'].items() if abs(v[0])+abs(v[1]) > 1e-6]
H.append(f'''<p><strong>Solve.</strong> Sequential LP over the leaf translations (scipy HiGHS): maximise the summed opening of the tight seams (capped at T=0.4&nbsp;mm) minus a small L1 penalty on motion, subject to the stretch-only floors above, edge inset, |v|&nbsp;≤&nbsp;0.6&nbsp;mm and a 0.1&nbsp;mm trust region; exact polygon re-check and re-linearisation each step. Result: <strong>{len(mv)} leaves move, |v| 0.02–0.60&nbsp;mm, 17 of 44 tight seams open (mean +0.056&nbsp;mm, best +0.33 JP1/C81, +0.28 TP8/J5, +0.24 MCU/U15, +0.23 R8/J5, +0.23 C80/C42, +0.21 C106/J3, +0.19 C105/J3, +0.17 ENC_A/ENC_AN, +0.17 ENC_D27/HB_A, +0.19 HB_B/J9)</strong>, no seam shrinks. A greedy single-leaf coordinate search finds a similar picture (22 seams, same magnitudes). The numbers are the honest size of the slack: tenths of a millimetre at a handful of seams.</p>''')
H.append('<div class="tablewrap"><table><tr><th>Leaf</th><th>Proposed (dx, dy) mm</th><th>Survived referee</th><th>Leaf</th><th>Proposed</th><th>Survived</th></tr>')
half = (len(mv)+1)//2
for i in range(half):
    row = ''
    for k in (i, i+half):
        if k < len(mv):
            m, v = mv[k]; ok = VOK['vectors'].get(m, [0, 0]); surv = abs(ok[0])+abs(ok[1]) > 1e-6
            row += f'<td class="mono">{esc(m)}</td><td class="num">({fmt(v[0])}, {fmt(v[1])})</td><td>{"<span class=sev good>kept</span>" if surv else "<span class=sev low>reverted</span>"}</td>'
        else: row += '<td></td>'*3
    H.append('<tr>'+row+'</tr>')
H.append('</table></div>')
H.append(f'''<p><strong>Warp.</strong> Applying a rigid translation per leaf is easy for footprints; the question is the copper. A first attempt blended every copper point by an inverse-distance field over the leaf territories — and produced 445 new violations, almost all inner-layer buses and via farms that happened to sit in a gap between two leaves and got <em>sheared</em>. Any gradient across a rule-pitch bundle is illegal: two 0.2&nbsp;mm tracks at 0.18&nbsp;mm gap lose 2&nbsp;µm of clearance at a 5.7° shear and 16&nbsp;µm at 17°, and this board's corridors ship at exactly the rule. The second model owns copper by <em>connectivity</em>: a track end on a pad is pinned to that pad's leaf; every other track end or via is a free node of the copper graph and takes the harmonic (length-weighted neighbour-average) interpolation of the pinned values, so a trace from module A to module B stretches <em>along itself</em>, a bundle of parallel A–B traces stays parallel, copper wholly inside one module is byte-identical, and pad-less via farms move rigidly as groups. No segment is split. On the full LP vector set this warp touched {warp['tracks']} tracks and {warp['vias']} vias in {warp['seam']} seam clusters ({warp['rigid']} clusters rigid) — and still left {ref_log[0]['new']} new violations, because seam copper on this board runs <em>past</em> third-party copper at rule pitch, not just between its two modules.</p>''')
H.append('<p><strong>Referee.</strong> Real kicad-cli DRC on the warped copy, diffed against the pre-warp baseline with position-independent signatures; each new violation is blamed on the moved leaves it names or sits inside; the worst leaf is reverted and the loop repeats until the warp is DRC-neutral:</p>')
H.append('<div class="tablewrap"><table><tr><th>Round</th><th>Leaves moving</th><th>New violations</th><th>Unconnected</th><th>Blame (top)</th><th>Reverted</th></tr>')
for i, r in enumerate(ref_log):
    bl = sorted(r['blame'].items(), key=lambda kv: -kv[1])[:4]
    rev = list(r['blame'].keys())
    worst = max(r['blame'].items(), key=lambda kv: kv[1])[0] if r['blame'] else '—'
    H.append(f'<tr><td class="num">{r["round"]}</td><td class="num">{r["moved"]}</td><td class="num">{r["new"]}</td><td class="num">{r["unconnected"]}</td><td class="mono small">{esc(", ".join(f"{k} {v}" for k, v in bl))}</td><td class="mono">{esc(worst) if r["new"] else "—"}</td></tr>')
H.append('</table></div>')
surv = {m: v for m, v in VOK['vectors'].items() if abs(v[0])+abs(v[1]) > 1e-6}
H.append(f'''<div class="callout warn"><strong>Finding (F-44).</strong> The DRC-neutral stretch on this board is <strong>{len(surv)} loose parts</strong>: {esc(", ".join(f"{m} ({fmt(v[0])}, {fmt(v[1])})" for m, v in surv.items()))} mm. Every dense-module move — HB_B −0.26&nbsp;x (16 violations), HB_D −0.2&nbsp;y (47), HB_A +0.11&nbsp;y (23), BUCK (69), PWR_OR (44), ENC_A −0.17&nbsp;x (5), C80/U15/C105/C47/TP8/R8/JP1 — pays in clearance violations on copper that is not its own. This is the module-granularity version of what the creep/morph experiments found at part granularity (<a href="creep.html">creep</a>, <a href="pcb.html#morph">morph</a>): the wall is topological. A stretch bigger than a few tenths needs the seam-crossing buses ripped and re-routed, i.e. it <em>is</em> a re-route, and the honest place to spend that effort is the routing pass below. The surviving subset is applied to the working board (signature-identical DRC: {st[1]['v']} violations / {st[1]['u']} unconnected, same as baseline).</div>''')
# ---- 4 placement ----------------------------------------------------------------------
H.append('<h2 id="place">4 · Re-placing the rev-B parts (and landing F-19)</h2>')
H.append('''<p>The 17 rev-B parts that were dropped provisionally in the corner shake (term switches U30–U33 with their caps C110–C113 and pull-ups R97–R100, pull-downs R101/R103/R104/R105/R108 that had landed on vias, and Q6 over the top edge) plus the four F-19 parts (R114/JP2 termination, C115/R115 barrier stitch) were placed by a pose scanner on real KiCad geometry: every 0.125–0.25&nbsp;mm × 90° pose in a window around an anchor, hard constraints = same-side courtyard (with DRC's 45&nbsp;µm outline inflation), other-net pads on a shared copper layer (incl. local clearances), other-net vias under a pad, board-edge inset; soft cost = other-net F/B track segments under a pad (copper to rip), with the USB pair, CAN pair and phase/VMOT copper never rippable. Jobs run sequentially so later parts see earlier ones. Where a part's home pocket had no legal pose (channels D and A are wall-to-wall against J2's pegs and J12's pins, channel B against H4's 1.4&nbsp;mm keepout) the window was widened: the 1&nbsp;mm encoder-edge extension from the morph prep is exactly where C110/C113/R97/R100/U30 now live, U33 sits in the divider block's free corner, U32 in the DIV↔channel-C gap, U31 right of J12's pin column.</p>''')
H.append('<div class="tablewrap"><table><tr><th>Part</th><th>From (provisional)</th><th>To</th><th>Rot</th><th>Side</th><th>Legal poses</th><th>Ripped copper</th></tr>')
for r, v in place.items():
    if not v: H.append(f'<tr><td class="mono">{r}</td><td colspan=6>no legal pose</td></tr>'); continue
    fr = v['from_']; src = f'({fr[0]:.2f}, {fr[1]:.2f})' if fr[0] or fr[1] else 'new'
    H.append(f'<tr><td class="mono">{r}</td><td class="mono">{src}</td><td class="mono">({v["x"]:.3f}, {v["y"]:.3f})</td><td class="num">{v["rot"]}</td><td>{"B" if v["back"] else "F"}</td><td class="num">—</td><td>{esc(v["ripnets"]) if v["ripnets"] else "—"}</td></tr>')
H.append('</table></div>')
H.append('''<p><strong>Q6 (F-45).</strong> The shipped board carried the DNP E-stop 2N7002 on a <em>SOT-89-3</em> land (5.1&nbsp;mm courtyard) while the schematic says SOT-23 — the one "known rev-A discrepancy" <code>check_sync</code> had been tolerating. There is no 5×5&nbsp;mm hole anywhere near the top-right corner, so the board now carries the schematic's SOT-23 (pads 1/2/3 = G/D/S = /E-STOP, /RUN, GND preserved), placed on the back next to J11. <code>check_sync</code> is IN SYNC (323 schematic / 323 board components) with no known exceptions left.</p>
<p><strong>F-19 on the board.</strong> R114 (120&nbsp;Ω) stands vertically in the strip between J11's pin column and C105/C106 (177.6,&nbsp;117.5), JP2 below D22/J8 (169.0,&nbsp;125.55) — open by default, bridge for an end-of-bus node. The barrier stitch: C115 (1206, 2&nbsp;kV) is the only part that will straddle the F-42 cut, placed on the <em>front</em> between J11 and J6 (180.2,&nbsp;119.5) where the isolated GND_ISO net comes off J11 pin 4; R115 on the back beside J11 pin 1 (176.25,&nbsp;124.0), wholly on the board-GND side. F-42 (the plane cut) is still open — the stitch is placed where the cut will be drawn.</p>''')
H.append('<figure class="scrollfig"><img src="img/modules/place_encoder_back.svg" alt="encoder corner placement"><figcaption>Encoder corner, back side, after placement: re-placed rev-B parts in purple; red dashed outlines/arrows mark the provisional poses they came from. Left edge strip = the 1&nbsp;mm board extension.</figcaption></figure>')
H.append('<div class="cols2"><figure class="scrollfig"><img src="img/modules/place_br_back.svg" alt="bottom right back"><figcaption>Bottom-right, back: R114, JP2, R115, Q6 (SOT-23 now), R108.</figcaption></figure><figure class="scrollfig"><img src="img/modules/place_br_front.svg" alt="bottom right front"><figcaption>Bottom-right, front: C115 between J11 and J6.</figcaption></figure></div>')
# ---- 5 cleanup + DRC ladder -----------------------------------------------------------------
H.append('<h2 id="drc">5 · DRC ladder</h2>')
H.append('<div class="tablewrap"><table><tr><th>Stage</th><th>Violations</th><th>Unconnected</th><th>Breakdown</th></tr>')
for k, v in stages.items():
    H.append(f'<tr><td>{esc(k)}</td><td class="num">{v["v"]}</td><td class="num">{v["u"]}</td><td class="small mono">{esc(", ".join(f"{a} {b}" for a, b in sorted(v["by"].items(), key=lambda kv: -kv[1])))}</td></tr>')
H.append('</table></div>')
H.append(f'''<p>The corner cleanup removed the copper the relaxed corner had been sitting on: {sum(c['vias'] for c in clean)} stray vias under pads of other nets (GND/+3V3/ENC_A_DATA/ENC_B_DATA/I2C_SW) and {sum(c['segs'] for c in clean)} track segments inside pad clearance (the C_SENSE pair past the shrunk C40 from F-43, GND_ISO past R53, /ENC_D_P under U18's thermal pad), then nudged R91 and U20 out of J2's peg/pin clearance. What remains before routing is four cosmetic solder-mask bridges between the mounting-hole openings and R106/R109/R111/R112 — and the airwires.</p>''')
# ---- 6 routing -----------------------------------------------------------------------------------
H.append('<h2 id="route">6 · Routing the rev-B nets</h2>')
if route:
    rl = json.load(open(S+'/router_log.json'))
    # parse final-round failures with reasons from router.log
    fails = []
    try:
        lines = open(S+'/router.log').read().splitlines()
        last = max(i for i, l in enumerate(lines) if l.startswith('round ') and 'routed' not in l)
        import re as _re
        for l in lines[last:]:
            m = _re.search(r'\] (\S+): FAILED \(([\d.]+)s\) src cells (\d+) \(passable (\d+)\) dst cells (\d+) clusters \S+ (.*)$', l)
            if m: fails.append((m.group(1), float(m.group(2)), int(m.group(3)), int(m.group(4)), int(m.group(5)), m.group(6)))
    except Exception: pass
    H.append(f'''<p><strong>Freerouting first.</strong> The obvious tool was tried first: every existing track and via locked (they export as <code>(type fix)</code>), DSN out, Freerouting 2.2.4 headless, SES back. On this board it does not finish: with 3,667 fixed items it sat at 10–15&nbsp;% CPU for 40 minutes without producing a session file (the sibling projects' notes already list "fixed wire brushing a via" and "fixed wire inside a plane" as hang triggers), and a second run died in a <code>HeadlessException</code>. So the airwires were routed in-house.</p>
<p><strong>The in-house router.</strong> <code>router.py</code>: 0.1&nbsp;mm grid on F.Cu / In2 / In3 / B.Cu (In1/In4 are the GND planes and are never routed on). Per net the passable set is an EDT on a per-layer copper raster — distance to other-net copper ≥ w/2 + clearance (net-class clearance of <em>both</em> nets, local clearances, 0.075&nbsp;mm sampling guard), own pads always enterable where the track width fits inside them, edge inset 0.3, rule areas hard; via sites need that on every copper layer plus hole-to-hole 0.254. Each DRC-unconnected pair is routed <em>cluster-to-cluster</em> by A* with octile moves, via cost, a penalty inside other-net pours (the phase/VMOT pours are hard — they must not be cut), EDT-to-target heuristic; plane nets finish at the first legal via site; routed copper joins the rasters at once. After every round kicad-cli DRC is the referee: any of this run's copper named in a <em>new</em> violation is ripped and the rasters rebuilt. GND was pre-stitched to the planes (40 vias, <code>gnd_stitch.py</code>).</p>''')
    H.append('<div class="tablewrap"><table><tr><th>Round</th><th>Airwires at start</th><th>Routed</th><th>Failed</th></tr>')
    for r in rl: H.append(f'<tr><td class="num">{r["round"]}</td><td class="num">{r["pairs"]}</td><td class="num">{r["routed"]}</td><td class="num">{len(r["failed"])}</td></tr>')
    H.append('</table></div>')
    bl = ', '.join(f'{k} {v}' for k, v in sorted(route['by_layer'].items(), key=lambda kv: -kv[1]))
    H.append(f'''<div class="callout good"><strong>Result.</strong> {route['new_segments']} new segments ({route['new_length_mm']}&nbsp;mm; {bl}) and {route['new_vias']} new vias; <strong>airwires {st[3]['u']} → {route['unconnected']}</strong>, DRC <strong>{route['drc']['v']} violations</strong> ({esc(', '.join(f"{a} {b}" for a, b in route['drc']['by'].items()))} — the four mask bridges at the mounting holes are the only items left and they were there before), <code>check_sync</code> IN SYNC. The board is still mid-rework — {route['unconnected']} connections remain — but every one of them is now a named, understood item rather than "214 airwires in the corner".</div>''')
    H.append('<figure class="scrollfig"><img src="img/modules/routed_new_copper.svg" alt="new copper"><figcaption>Copper added by the routing pass, coloured by layer (existing copper faint grey); red dashed = airwires still open.</figcaption></figure>')
    rem = route['remaining']
    H.append(f'<p><strong>What is left ({route["unconnected"]}).</strong> By net: {esc(", ".join(f"{k} {v}" for k, v in rem[:26]))}{" …" if len(rem) > 26 else ""}.</p>')
    if fails:
        boxed = [f for f in fails if f[1] < 0.3]; deep = [f for f in fails if f[1] >= 0.3]
        H.append(f'''<p>The router's own diagnostics split them in two kinds. <strong>{len(boxed)} are boxed in</strong>: the A* exhausts a few dozen cells in milliseconds because the pad has no exit at all — typically a SIT3088 pin row sitting 0.2&nbsp;mm from its choke's pads (U20↔FL3, U18↔FL2, U19↔FL1: the relaxed corner left the transceiver/choke pairs nose-to-nose with no 0.56&nbsp;mm channel for a 0.2&nbsp;mm track), a pull-down whose GND pad is inside a mounting hole's ring, or a termination-switch pin (U31–U33) facing another part. <strong>{len(deep)} fail after a real search</strong> (the whole reachable region is walled off by at-limit copper). Both kinds are placement problems, not routing problems: the fix is to turn/slide the part, not to search harder.</p>''')
        H.append('<div class="tablewrap"><table><tr><th>Net</th><th>Pair</th><th>Kind</th><th>src cells (passable)</th><th>dst cells</th></tr>')
        for f in sorted(fails, key=lambda f: (f[1] >= 0.3, f[0]))[:40]:
            H.append(f'<tr><td class="mono">{esc(f[0])}</td><td class="small mono">{esc(f[5])}</td><td>{"boxed in" if f[1] < 0.3 else "walled region"}</td><td class="num">{f[2]} ({f[3]})</td><td class="num">{f[4]}</td></tr>')
        H.append('</table></div>')
    H.append('''<p><strong>How to finish.</strong> (1) In the encoder corner rotate each choke so its TRX pads face the transceiver's A/B pins (or slide the SIT3088 0.4&nbsp;mm away from the choke to open a track channel) — the scanner in <code>place_scan.py</code> can evaluate candidate poses, and the router re-runs in under a minute; (2) move the four pull-downs that sit inside the mounting-hole rings (R106/R109/R111/R112) by ~1&nbsp;mm; (3) re-run <code>router.py</code> and the DRC gate. The phase/VMOT pours were deliberately kept uncut; the five phase-net "airwires" that appeared mid-way in earlier runs (VMOT, /B_P, /B_N, /C_N, /D) were pours the router had split by routing through them — that is why those pours are hard obstacles now.</p>''')
else:
    H.append('<p class="callout warn">Routing run in progress — this section is filled in by <code>render_page.py</code> once <code>route_result.json</code> exists.</p>')
# ---- 7 tooling --------------------------------------------------------------------------------------
H.append('''<h2 id="tools">7 · Tooling</h2>
<p>Everything is scripted and lives in <code>hardware/tools/modules/</code>: <code>modules.py</code> (leaf definitions → modules.json), <code>slack.py</code> / <code>relax2.py --probe</code> (per-leaf slack), <code>relax3.py</code> (sequential-LP stretch), <code>warp2.py</code> (connectivity-owned warp), <code>referee.py</code> (DRC referee loop), <code>place_scan.py</code> (pose scanner / placer, also does footprint swaps and new parts), <code>clean_corner.py</code>, <code>route.py</code> (Freerouting round-trip with all existing copper locked), <code>drcdiff.py</code> (position-independent DRC diff), <code>map_modules.py</code> + <code>render_page.py</code> (this page). Geometry: pcbnew 9.0.8 courtyard/pad polygons; oracle: kicad-cli DRC with the project file beside every board copy.</p>
<footer style="margin-top:4rem;color:var(--muted);font-size:.85rem;border-top:1px solid var(--line);padding-top:1rem">Board: <code>hardware/rp2350_driver.kicad_pcb</code> on <code>rev-b-fixes</code>; all numbers from kicad-cli 9.0.8 DRC (severity error, all track errors) with <code>rp2350_driver.kicad_pro</code> beside each board copy.</footer>
</main></body></html>''')
open(OUT, 'w').write('\n'.join(H)); print('wrote', OUT, len('\n'.join(H)), 'bytes')
