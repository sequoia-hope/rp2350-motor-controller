"""kicad_write.py -- write KiCad 9 schematics and push them to a board, headless.

Two jobs KiCad's CLI can't do:

    Schematic(...)   place library symbols, wires, labels and power symbols at
                     coordinates you chose, and save a .kicad_sch
    update_pcb(...)  "Update PCB from Schematic": footprints, fields, DNP/BOM
                     flags and pad nets from the schematic's netlist, keeping
                     the position of every part already on the board

The coordinates are the design (see tools/kicad.md, "Schematic authoring").
Plan each block on paper first; never place symbols in a loop with a fixed
step. Check the result with `kicad-cli sch erc` and by reading the exported
SVG/PDF, not by trusting this module.
"""
from __future__ import annotations

import math
import re
import subprocess
import tempfile
import uuid as _uuid
import xml.etree.ElementTree as ET
from pathlib import Path

from kicad_files import Sym, child, children, head, parse_sexpr

SYSTEM_SYMBOLS = Path("/usr/share/kicad/symbols")
SYSTEM_FOOTPRINTS = Path("/usr/share/kicad/footprints")
GRID = 1.27


def new_uuid() -> str:
    return str(_uuid.uuid4())


# ---------------------------------------------------------------- S-expression output

def _atom(x) -> str:
    if isinstance(x, Sym):
        return str(x)
    if isinstance(x, bool):
        return "yes" if x else "no"
    if isinstance(x, (int, float)):
        return fmt(x)
    s = str(x).replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
    return f'"{s}"'


def fmt(v: float) -> str:
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def dump(node, indent: int = 0) -> str:
    if not isinstance(node, list):
        return _atom(node)
    flat = all(not isinstance(c, list) for c in node[1:])
    if flat or (len(node) <= 4 and all(not isinstance(c, list) or len(c) <= 3 for c in node[1:])
                and head(node) in ("xy", "at", "size", "font", "pts", "stroke", "pin", "effects")):
        return "(" + " ".join(dump(c) for c in node) + ")"
    pad = "\t" * (indent + 1)
    parts = [dump(node[0])]
    for c in node[1:]:
        parts.append("\n" + pad + dump(c, indent + 1) if isinstance(c, list) else " " + dump(c))
    return "(" + parts[0] + "".join(parts[1:]) + "\n" + "\t" * indent + ")"


def S(*items):
    """Build a node: S("at", 1, 2) -> (at 1 2). Strings are quoted unless Sym()."""
    head_, *rest = items
    return [Sym(head_), *rest]


def yes(b: bool):
    return Sym("yes" if b else "no")


# ---------------------------------------------------------------- symbol libraries

class SymbolLib:
    """Library symbols by "Lib:Name", flattened (extends resolved), from the system
    library folder plus any extra .kicad_sym files the project passes."""

    def __init__(self, extra: list[Path] | None = None):
        self.files = {p.stem: p for p in SYSTEM_SYMBOLS.glob("*.kicad_sym")}
        for p in extra or []:
            self.files[p.stem] = Path(p)
        self._libs: dict[str, dict[str, list]] = {}

    def _lib(self, lib: str) -> dict[str, list]:
        if lib not in self._libs:
            tree = parse_sexpr(self.files[lib].read_text(encoding="utf-8"))
            self._libs[lib] = {s[1]: s for s in children(tree, "symbol")}
        return self._libs[lib]

    def get(self, lib_id: str) -> list:
        """The symbol as it is embedded in a schematic's lib_symbols."""
        lib, name = lib_id.split(":", 1)
        syms = self._lib(lib)
        if name not in syms:
            raise KeyError(f"{lib_id} not found in {self.files[lib]}")
        sym = syms[name]
        ext = child(sym, "extends")
        if ext:
            parent = self.get(f"{lib}:{ext[1]}")
            props = {p[1]: p for p in children(sym, "property")}
            out = [Sym("symbol"), lib_id]
            for c in parent[2:]:
                if head(c) == "property" and c[1] in props:
                    out.append(props.pop(c[1]))
                elif head(c) == "symbol":   # body units keep the parent's name prefix
                    out.append([c[0], re.sub(rf"^{re.escape(ext[1])}_", f"{name}_", c[1]), *c[2:]])
                else:
                    out.append(c)
            out.extend(props.values())
            for c in sym[2:]:
                if head(c) not in ("property", "extends") and head(c) not in [head(x) for x in out[2:]]:
                    out.append(c)
            return out
        return [Sym("symbol"), lib_id, *sym[2:]]


def symbol_pins(sym: list, unit: int = 1, style: int = 1) -> dict[str, dict]:
    """Pins of one unit: number -> {name, x, y, angle, type} in library coordinates (y up)."""
    pins = {}
    for sub in children(sym, "symbol"):
        m = re.search(r"_(\d+)_(\d+)$", sub[1])
        if not m or int(m.group(1)) not in (0, unit) or int(m.group(2)) not in (0, style):
            continue
        for p in children(sub, "pin"):
            at = child(p, "at")
            pins[child(p, "number")[1]] = {
                "name": child(p, "name")[1], "type": str(p[1]),
                "x": float(at[1]), "y": float(at[2]), "angle": float(at[3]) if len(at) > 3 else 0.0,
            }
    return pins


def place(px: float, py: float, at: tuple[float, float], rot: float, mirror: str | None):
    """Library pin point -> schematic point. Library y is up, schematic y is down;
    KiCad rotates counter-clockwise on screen first, then mirrors."""
    x, y = px, -py
    a = math.radians(-rot)
    x, y = x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)
    if mirror == "x":
        y = -y
    elif mirror == "y":
        x = -x
    return (round(at[0] + x, 4), round(at[1] + y, 4))


# ---------------------------------------------------------------- schematic writer

class Part:
    def __init__(self, sch: "Schematic", node: list, pins: dict, at, rot, mirror):
        self.sch, self.node, self._pins, self.at, self.rot, self.mirror = sch, node, pins, at, rot, mirror

    @property
    def ref(self) -> str:
        return self.node and next(p[2] for p in children(self.node, "property") if p[1] == "Reference")

    def pin(self, key: str) -> tuple[float, float]:
        """Connection point of a pin, by number or by unique name."""
        p = self._pins.get(str(key))
        if p is None:
            named = [q for q in self._pins.values() if q["name"] == key]
            if len({(q["x"], q["y"]) for q in named}) == 1:   # stacked pins share one point
                named = named[:1]
            if len(named) != 1:
                raise KeyError(f"{self.ref}: no unique pin {key!r} "
                               f"(have {sorted(self._pins)} / {sorted(q['name'] for q in self._pins.values())})")
            p = named[0]
        return place(p["x"], p["y"], self.at, self.rot, self.mirror)

    def pins(self) -> dict[str, tuple[float, float]]:
        return {n: place(p["x"], p["y"], self.at, self.rot, self.mirror) for n, p in self._pins.items()}


class Schematic:
    def __init__(self, project: str, title: str = "", rev: str = "", date: str = "",
                 company: str = "", comments: list[str] | None = None, paper: str = "A4",
                 libs: SymbolLib | None = None):
        self.project = project
        self.uuid = new_uuid()
        self.libs = libs or SymbolLib()
        self.paper = paper
        self.title = [S("title", title), S("date", date), S("rev", rev), S("company", company)]
        for i, c in enumerate(comments or [], 1):
            self.title.append(S("comment", i, c))
        self.lib_symbols: dict[str, list] = {}
        self.items: list[list] = []
        self.parts: list[Part] = []
        self._pwr = 0

    # -- symbols
    def _field(self, name, value, at, hide=False, rot=0, justify=None, size=GRID):
        eff = [Sym("effects"), S("font", S("size", size, size))]
        if justify:
            eff.append([Sym("justify"), *[Sym(j) for j in justify.split()]])
        if hide:
            eff.append(S("hide", yes(True)))
        return S("property", name, value, S("at", at[0], at[1], rot), eff)

    def symbol(self, lib_id: str, ref: str, value: str, at, rot: float = 0, mirror: str | None = None,
               footprint: str = "", fields: dict[str, str] | None = None, unit: int = 1,
               dnp: bool = False, in_bom: bool = True, on_board: bool = True,
               ref_at=None, value_at=None, ref_rot: float | None = None, datasheet: str = "",
               description: str = "", justify: str | None = None) -> Part:
        """Place a symbol. ref_at / value_at are absolute positions for the two
        visible fields; by default they go to the right of the symbol's origin.
        KiCad turns field text with the symbol, so by default the fields of a
        symbol rotated 90 or 270 get the same angle, which reads horizontally."""
        if ref_rot is None:
            ref_rot = rot if rot in (90, 270) else 0
        sym = self.lib_symbols.setdefault(lib_id, self.libs.get(lib_id))
        pins = symbol_pins(sym, unit)
        lib_props = {p[1]: p[2] for p in children(sym, "property")}
        is_power = any(head(c) == "power" for c in sym[2:])
        hide_ref = is_power or ref.startswith("#")
        ref_at = ref_at or (at[0] + 2.54, at[1] - 1.27)
        value_at = value_at or (at[0] + 2.54, at[1] + 1.27)
        j = justify if justify is not None else ("" if is_power else "left")
        if mirror == "y" and j:   # KiCad mirrors field justification with the symbol
            j = j.replace("left", "@").replace("right", "left").replace("@", "right")
        node = [Sym("symbol"), S("lib_id", lib_id), S("at", at[0], at[1], rot)]
        if mirror:
            node.append(S("mirror", Sym(mirror)))
        node += [S("unit", unit), S("exclude_from_sim", yes(False)), S("in_bom", yes(in_bom)),
                 S("on_board", yes(on_board)), S("dnp", yes(dnp)), S("uuid", new_uuid()),
                 self._field("Reference", ref, ref_at, hide=hide_ref, rot=ref_rot, justify=j or None),
                 self._field("Value", value, value_at, rot=ref_rot, justify=j or None),
                 self._field("Footprint", footprint or lib_props.get("Footprint", ""), at, hide=True),
                 self._field("Datasheet", datasheet or lib_props.get("Datasheet", ""), at, hide=True),
                 self._field("Description", description or lib_props.get("Description", ""), at, hide=True)]
        for k, v in (fields or {}).items():
            node.append(self._field(k, v, at, hide=True))
        all_pins = set()
        for sub in children(sym, "symbol"):
            m = re.search(r"_(\d+)_(\d+)$", sub[1])
            if m and int(m.group(1)) in (0, unit):
                all_pins |= {child(p, "number")[1] for p in children(sub, "pin")}
        for n in sorted(all_pins):
            node.append(S("pin", n, S("uuid", new_uuid())))
        node.append(S("instances", S("project", self.project,
                                     S("path", "/" + self.uuid, S("reference", ref), S("unit", unit)))))
        self.items.append(node)
        part = Part(self, node, pins, at, rot, mirror)
        self.parts.append(part)
        return part

    def power(self, lib_id: str, at, rot: float = 0, value: str | None = None) -> Part:
        """A power or ground symbol; its value is the net name."""
        self._pwr += 1
        name = value or lib_id.split(":")[1]
        vat = (at[0], at[1] - 3.81) if rot == 0 and "GND" not in name else (at[0], at[1] + 3.81)
        if rot in (90, 270):
            vat = (at[0] + (3.81 if rot == 270 else -3.81), at[1])
        return self.symbol(lib_id, f"#PWR{self._pwr:02d}", name, at, rot=rot,
                           ref_at=at, value_at=vat, in_bom=False, on_board=False, justify="")

    # -- connections
    def wire(self, *pts):
        for a, b in zip(pts, pts[1:]):
            if a == b:
                continue
            self.items.append(S("wire", S("pts", S("xy", *a), S("xy", *b)),
                                S("stroke", S("width", 0), S("type", Sym("default"))), S("uuid", new_uuid())))

    def route(self, a, b, via: str = "h"):
        """Wire a to b with one bend: "h" goes horizontal first, "v" vertical first."""
        corner = (b[0], a[1]) if via == "h" else (a[0], b[1])
        self.wire(a, corner, b)

    def junction(self, at):
        self.items.append(S("junction", S("at", *at), S("diameter", 0),
                            S("color", 0, 0, 0, 0), S("uuid", new_uuid())))

    def no_connect(self, at):
        self.items.append(S("no_connect", S("at", *at), S("uuid", new_uuid())))

    def label(self, name: str, at, rot: float = 0, kind: str = "label", shape: str = "bidirectional"):
        """kind: label (local), global_label or hierarchical_label. Text runs away from the
        connection point: rot 0 to the right, 180 to the left."""
        just = "left bottom" if rot in (0, 90) else "right bottom"
        if kind != "label":
            just = "left" if rot in (0, 90) else "right"
        node = [Sym(kind), name]
        if kind != "label":
            node.append(S("shape", Sym(shape)))
        node += [S("at", at[0], at[1], rot),
                 [Sym("effects"), S("font", S("size", GRID, GRID)), [Sym("justify"), *[Sym(j) for j in just.split()]]],
                 S("uuid", new_uuid())]
        self.items.append(node)

    def text(self, s: str, at, size: float = GRID, rot: float = 0):
        self.items.append(S("text", s, S("exclude_from_sim", yes(False)), S("at", at[0], at[1], rot),
                            [Sym("effects"), S("font", S("size", size, size)), S("justify", Sym("left"))],
                            S("uuid", new_uuid())))

    def box(self, a, b, label: str | None = None):
        """A dashed rectangle around a functional block, optionally titled."""
        self.items.append(S("rectangle", S("start", *a), S("end", *b),
                            S("stroke", S("width", 0), S("type", Sym("dash"))),
                            S("fill", S("type", Sym("none"))), S("uuid", new_uuid())))
        if label:
            self.text(label, (a[0] + 1.27, a[1] + 2.54), size=1.778)

    # -- output
    def _cleanup(self):
        """What the schematic editor does on every edit: split each wire wherever
        another connection point lies on it (KiCad only connects at wire ends; a
        junction or pin mid-segment is silently left unconnected), then put a
        junction wherever three or more wire ends and pins meet."""
        def pt(n):
            return (round(float(n[1]), 4), round(float(n[2]), 4))
        wires = [w for w in self.items if head(w) == "wire"]
        points = set()
        for w in wires:
            points |= {pt(xy) for xy in children(child(w, "pts"), "xy")}
        for p in self.parts:
            points |= set(p.pins().values())
        for it in self.items:
            if head(it) in ("junction", "label", "global_label", "hierarchical_label", "no_connect"):
                points.add(pt(child(it, "at")))

        def between(a, b, p):
            if p in (a, b):
                return False
            if a[0] == b[0] == p[0]:
                return min(a[1], b[1]) < p[1] < max(a[1], b[1])
            if a[1] == b[1] == p[1]:
                return min(a[0], b[0]) < p[0] < max(a[0], b[0])
            return False

        out, segs = [], []
        for it in self.items:
            if head(it) != "wire":
                out.append(it)
                continue
            a, b = (pt(xy) for xy in children(child(it, "pts"), "xy"))
            cuts = sorted([p for p in points if between(a, b, p)],
                          key=lambda p: abs(p[0] - a[0]) + abs(p[1] - a[1]))
            chain = [a, *cuts, b]
            for u, v in zip(chain, chain[1:]):
                segs.append((u, v))
                out.append(S("wire", S("pts", S("xy", *u), S("xy", *v)),
                             S("stroke", S("width", 0), S("type", Sym("default"))), S("uuid", new_uuid())))
        self.items = out

        degree: dict[tuple, int] = {}
        for u, v in segs:
            degree[u] = degree.get(u, 0) + 1
            degree[v] = degree.get(v, 0) + 1
        for p in self.parts:
            for q in p.pins().values():
                if q in degree:
                    degree[q] += 1
        have = {pt(child(it, "at")) for it in self.items if head(it) == "junction"}
        for q, n in degree.items():
            if n >= 3 and q not in have:
                self.junction(q)

    def save(self, path: Path):
        self._cleanup()
        tree = [Sym("kicad_sch"), S("version", Sym("20250114")), S("generator", "eeschema"),
                S("generator_version", "9.0"), S("uuid", self.uuid), S("paper", self.paper),
                [Sym("title_block"), *[t for t in self.title if t[-1] != ""]],
                [Sym("lib_symbols"), *self.lib_symbols.values()],
                *self.items,
                S("sheet_instances", S("path", "/", S("page", "1"))),
                S("embedded_fonts", yes(False))]
        Path(path).write_text(dump(tree) + "\n", encoding="utf-8")


# ---------------------------------------------------------------- schematic -> board

def footprint_dirs(project_dir: Path) -> dict[str, Path]:
    """Footprint library nickname -> .pretty folder: system libraries, then the
    project's fp-lib-table (which wins)."""
    libs = {p.stem: p for p in SYSTEM_FOOTPRINTS.glob("*.pretty")}
    table = project_dir / "fp-lib-table"
    if table.exists():
        for lib in children(parse_sexpr(table.read_text()), "lib"):
            name, uri = child(lib, "name")[1], child(lib, "uri")[1]
            libs[name] = Path(uri.replace("${KIPRJMOD}", str(project_dir)))
    return libs


def netlist(sch: Path) -> ET.Element:
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "net.xml"
        r = subprocess.run(["kicad-cli", "sch", "export", "netlist", "--format", "kicadxml",
                            "-o", str(out), str(sch)], capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError(r.stderr)
        return ET.parse(out).getroot()


def update_pcb(sch: Path, pcb: Path, new_parts_at=(0.0, 0.0), log=print,
               remove_extra_fields: bool = False) -> None:
    """Push the schematic to the board, like KiCad's Update PCB from Schematic.

    Footprints are matched by reference. A part whose footprint changed is
    replaced in place; new parts are stacked at new_parts_at for placement;
    parts no longer in the schematic are removed (board-only parts are kept).
    Fields that exist only on the board are kept and logged, unless
    remove_extra_fields. Tracks and zones are left alone.
    """
    import pcbnew

    sch, pcb = Path(sch), Path(pcb)
    root = netlist(sch)
    libs = footprint_dirs(pcb.parent)
    board = pcbnew.LoadBoard(str(pcb)) if pcb.exists() else pcbnew.NewBoard(str(pcb))
    existing = {}
    for fp in board.GetFootprints():
        if fp.IsBoardOnly():
            continue
        if fp.GetReference() in existing:
            log(f"  WARNING: {fp.GetReference()} appears twice on the board; only one is updated")
        existing[fp.GetReference()] = fp
    wanted = set()

    # The board keeps its own title block; seed it from the root sheet's.
    tb, stb = board.GetTitleBlock(), root.find("design/sheet/title_block")
    if stb is not None:
        for tag, get, put in [("title", tb.GetTitle, tb.SetTitle), ("rev", tb.GetRevision, tb.SetRevision),
                              ("date", tb.GetDate, tb.SetDate), ("company", tb.GetCompany, tb.SetCompany)]:
            if not get() and stb.findtext(tag):
                put(stb.findtext(tag))

    for comp in root.iter("comp"):
        ref = comp.get("ref")
        props = {p.get("name"): p.get("value", "") for p in comp.findall("property")}
        if "exclude_from_board" in props:
            continue
        wanted.add(ref)
        fpid = comp.findtext("footprint") or ""
        if ":" not in fpid:
            raise ValueError(f"{ref} has no footprint (got {fpid!r})")
        lib, name = fpid.split(":", 1)
        fp = existing.get(ref)
        if fp is not None and fp.GetFPIDAsString() != fpid:
            pos, rot, flipped = fp.GetPosition(), fp.GetOrientation(), fp.IsFlipped()
            board.Remove(fp)
            log(f"  {ref}: footprint {fp.GetFPIDAsString()} -> {fpid}")
            fp = None
        else:
            pos, rot, flipped = None, None, False
        if fp is None:
            if lib not in libs:
                raise KeyError(f"{ref}: footprint library {lib!r} not found")
            fp = pcbnew.FootprintLoad(str(libs[lib]), name)
            if fp is None:
                raise KeyError(f"{ref}: {name} not in {libs[lib]}")
            fp.SetFPID(pcbnew.LIB_ID(lib, name))
            board.Add(fp)
            if pos is not None:
                fp.SetPosition(pos)
                if flipped:  # flip first: flipping negates the angle
                    fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
                fp.SetOrientation(rot)
            else:
                fp.SetPosition(pcbnew.VECTOR2I_MM(*new_parts_at))
                log(f"  {ref}: added {fpid}")
        fp.SetReference(ref)
        fp.SetValue(comp.findtext("value") or "")
        keep = {"Reference", "Value", "Footprint", "Datasheet", "Description"}
        fields = {f.get("name"): (f.text or "") for f in comp.iter("field")}
        for f in list(fp.GetFields()):
            if f.GetName() not in keep and f.GetName() not in fields:
                if remove_extra_fields:
                    fp.RemoveField(f.GetName())
                    log(f"  {ref}: removed board-only field {f.GetName()}")
                else:
                    log(f"  {ref}: field {f.GetName()} is on the board but not in the schematic (kept)")
        for k, v in fields.items():
            if k in ("Footprint",):
                continue
            if fp.HasFieldByName(k):
                fp.GetFieldByName(k).SetText(v)
            else:
                fp.SetField(k, v)
                fp.GetFieldByName(k).SetVisible(False)
                fp.GetFieldByName(k).SetLayer(pcbnew.F_Fab)
        fp.SetDNP("dnp" in props)
        fp.SetExcludedFromBOM("exclude_from_bom" in props)
        sp = comp.find("sheetpath")
        fp.SetPath(pcbnew.KIID_PATH(sp.get("tstamps") + comp.findtext("tstamps").split()[0]))
        fp.SetSheetname(sp.get("names"))
        fp.SetSheetfile(props.get("Sheetfile", sch.name))

    for ref, fp in existing.items():
        if ref not in wanted and fp.GetParent() is not None:
            board.Remove(fp)
            log(f"  {ref}: removed (not in schematic)")

    fps = {fp.GetReference(): fp for fp in board.GetFootprints()
           if fp.GetReference() in wanted and not fp.IsBoardOnly()}
    for pad in [p for fp in fps.values() for p in fp.Pads()]:   # board-only parts keep their nets
        pad.SetNetCode(0)
    for n in root.iter("net"):
        name = n.get("name")
        net = board.FindNet(name)
        if net is None:
            net = pcbnew.NETINFO_ITEM(board, name)
            board.Add(net)
        for node in n.findall("node"):
            fp = fps.get(node.get("ref"))
            if fp is None:
                continue
            for pad in fp.Pads():
                if pad.GetNumber() == node.get("pin"):
                    pad.SetNet(net)
    board.BuildConnectivity()
    pcbnew.SaveBoard(str(pcb), board)
