#!/usr/bin/env python3
"""Carve the board into logical modules (leaf = rigid body for the stretch).
Parents group leaves for reporting. Pinned = mechanical interfaces (never move).
REPLACE = rev-B parts whose current pose is provisional (re-placed after the
stretch), excluded from every leaf's geometry. Writes modules.json + map SVG."""
import json, math, collections, re, sys, os
S = os.path.dirname(os.path.abspath(__file__))
B = json.load(open(S + "/board.json")); N = json.load(open(S + "/netlist.json"))
fp = {f['ref']: f for f in B['fps']}
def R(a, b):
    p = re.match(r'[A-Z]+', a).group(0); return [f'{p}{i}' for i in range(int(a[len(p):]), int(b[len(p):]) + 1)]
REPLACE = set('U30 C110 R97 U33 C113 R100 U32 C112 R99 U31 C111 R98 R101 R103 R104 R105 R108 Q6'.split())
# leaf: (parent, title, refs)
LEAVES = collections.OrderedDict([
 ('ENC_D',  ('ENC', 'channel D: SIT3088 + choke + term/pulls + TVS (+U33/C113/R100)', ['U21','FL4','R85','R90','R91','C63','D26','U33','C113','R100'])),
 ('ENC_C',  ('ENC', 'channel C: SIT3088 + choke + term/pulls (+U32/C112/R99)', ['U20','FL3','R84','R92','R93','C62','U32','C112','R99'])),
 ('ENC_A',  ('ENC', 'channel A: SIT3088 + choke + term/pulls (+U30/C110/R97)', ['U18','FL2','R82','R88','R89','C60','U30','C110','R97'])),
 ('ENC_B',  ('ENC', 'channel B: SIT3088 + choke + term/pulls (+U31/C111/R98)', ['U19','FL1','R83','R86','R87','C61','U31','C111','R98'])),
 ('ENC_SW', ('ENC', 'hall/I2C switch column: 6× TS5A3159, filter caps, 2× TVS', ['U22','U23','U26','U27','C64','C65','C67','C68','C69','C70','C72','C73','C74','C75','D23','D24','R94','R95','R96'])),
 ('ENC_AN', ('ENC', 'ext-analog switches U24/U25 + caps', ['U24','U25','C71','C66'])),
 ('ENC_D27',('ENC', 'I2C/MOT_TH TVS', ['D27'])),
 ('ENC_D25',('ENC', 'channel C TVS', ['D25'])),
 ('DIV',    ('DIV', 'phase/bus voltage-sense dividers (0.1%) + TP7', R('R59','R64') + R('R67','R72') + ['TP7'])),
 ('HB_C',   ('HB', 'half-bridge C: FETs/shunt/TVS front, EG3113 + gate R + INA240 back', ['CL1','CH1','R56','R55','D17','C44','C34','C33','R52','R48','C8','U7','U10','Q3','C40','C19','C36','C27','TP5','TP6','D14','D15','D16','C31','R109'] + R('R41','R47'))),
 ('HB_B',   ('HB', 'half-bridge B', ['BL1','BH1','R40','R39','D13','C26','C25','R38','R37','C1','U6','U9','Q2','C28','C23','C29','TP3','TP4','D10','D11','D12','C15','TH1','R107','R104','R25'] + R('R27','R31') + ['R33'])),
 ('HB_A',   ('HB', 'half-bridge A', ['AL1','AH1','R23','R20','D9','C9','C7','R18','R17','U1','U5','Q1','C10','C2','C6','C16','TP1','TP2','D3','D6','D7','R103','R101','R102'] + R('R9','R15'))),
 ('HB_D',   ('HB', 'half-bridge D (brake-chopper leg)', ['CL2','CH2','D21','C54','C53','R81','R80','U12','Q4','C48','C51','C55','TP9','TP10','D18','D19','D20','R105','R106'] + R('R73','R79'))),
 ('C80',    ('BUS', 'bulk electrolytic', ['C80'])), ('C42', ('BUS', 'bulk electrolytic', ['C42'])), ('C39', ('BUS', 'bulk electrolytic', ['C39'])),
 ('D5',     ('BUS', 'bus TVS SMDJ64A', ['D5'])), ('TP8', ('BUS', 'test point', ['TP8'])), ('C47', ('BUS', '10 µF', ['C47'])),
 ('BUCK',   ('BUCK', 'Vdrive buck XL7005A + D4 + L3 + C11 + feedback', ['U28','D4','L3','C11','C3','R5','R6'])),
 ('PWR_OR', ('PWR', 'LTC4359 + PowerPAK FET + C78', ['U29','Q5','C78'])),
 ('JP1',    ('PWR', 'power-path jumper', ['JP1'])), ('C81', ('PWR', '1 µF 100 V', ['C81'])),
 ('MCU',    ('MCU', 'RP2350 + crystal + flash + decoupling + USB ESD + PWM pulldowns/test points', 
             ['U8','Y1','C13','C14','R26','U3','C4','R34','L2','R50','R51','C17','C18','C21','C22','C24','C30','C32','C35','C37','C38','C41','C43','C45','C46','C20','SW1','SW2','D1','R57','R58','R24','R32','C52',
              'R1','R2','R21','R22','R35','R36','R65','R66','TP12','TP13','TP14','TP15','TP16','TP17','TP18','TP19','U11','R53','R54'])),
 ('AUX_LDO',('AUX', '3.3 V LDO SPX3819 + caps', ['U4','C5','C12'])),
 ('AUX_SW', ('AUX', 'ext-analog ADC switches U13/U14 + caps', ['U13','U14','C49','C50'])),
 ('R112',   ('AUX', 'TERM_SW pull', ['R112'])), ('C114', ('AUX', 'LED_VDD cap', ['C114'])), ('D30', ('AUX', 'WS2812 supply diode', ['D30'])),
 ('R113',   ('AUX', 'E-stop pull', ['R113'])), ('Q6', ('AUX', 'E-stop FET (DNP)', ['Q6'])), ('D28', ('AUX', 'daisy UART TVS', ['D28'])),
 ('R49',    ('AUX', 'thermistor divider', ['R49'])), ('TP20', ('AUX', 'TERM_SW test point', ['TP20'])), ('U15', ('AUX', 'thermistor switch', ['U15'])),
 ('CAN',    ('CAN', 'CA-IS3052 isolator + decoupling (F-19 R114/JP2/C115/R115 to land here)', ['U16','C56','C57','C58'])),
 ('USB_PD', ('USBPD', 'FUSB302 + CC/decoupling caps', ['U101','C107','C108','C76','C77'])),
 ('C105',   ('USBPD', 'VBUS cap', ['C105'])), ('C106', ('USBPD', 'VBUS cap', ['C106'])), ('R110', ('USBPD', 'ENC_C_SW pull', ['R110'])),
 ('R111',   ('USBPD', 'ENC_D_SW pull', ['R111'])), ('D31', ('USBPD', 'VBUS TVS SMF24A', ['D31'])),
 ('VENC',   ('BR', 'V_ENC switch U17 + C59', ['U17','C59'])), ('R108', ('BR', 'ENC_A_SW pull', ['R108'])),
 ('D29',    ('BR', 'daisy UART TVS', ['D29'])), ('R8', ('BR', 'E-stop conn pull', ['R8'])), ('D22', ('BR', 'CAN ESD diode', ['D22'])),
])
PINNED = ['J1','J2','J12','J9','J10','J3','J4','J6','J5','J7','J8','J11','H1','H2','H3','H4']
for r in PINNED:
    LEAVES[r] = ('PIN', f'{r} {fp[r]["value"]} (pinned interface)', [r])
assign = {}
for m, (par, t, refs) in LEAVES.items():
    for r in refs:
        if r not in fp: print('seed not on board:', r, '(', m, ')'); continue
        if r in assign: print('DUP', r, assign[r], m)
        assign[r] = m
left = sorted(r for r in fp if r not in assign)
if left: print('UNASSIGNED:', left)
mods = collections.OrderedDict()
def rects(f):
    rs = []
    for side, polys in f['courtyard'].items():
        for poly in polys:
            xs = [p[0] for p in poly]; ys = [p[1] for p in poly]; rs.append((min(xs), min(ys), max(xs), max(ys)))
    return rs or [tuple(f['bbox'])]
for m, (par, t, refs) in LEAVES.items():
    mem = [r for r in refs if r in fp]
    geo = [r for r in mem if r not in REPLACE]
    rr = [rc for r in geo for rc in rects(fp[r])] or [rc for r in mem for rc in rects(fp[r])]
    bb = (min(r[0] for r in rr), min(r[1] for r in rr), max(r[2] for r in rr), max(r[3] for r in rr))
    mods[m] = dict(parent=par, title=t, pinned=(par == 'PIN'), refs=mem, geo_refs=geo, bbox=[round(v, 3) for v in bb],
                   sides=sorted({fp[r]['layer'] for r in mem}))
json.dump(dict(modules=mods, assign=assign, replace=sorted(REPLACE)), open(S + '/modules.json', 'w'), indent=1)
print(len(mods), 'leaves;', sum(1 for m in mods.values() if not m['pinned']), 'movable;', len(assign), 'parts assigned')
