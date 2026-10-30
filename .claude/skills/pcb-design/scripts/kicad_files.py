"""kicad_files.py -- read KiCad 9 projects for the check scripts.

The schematic side comes from `kicad-cli sch export netlist` (it resolves
hierarchy, annotation and DNP for us); the board side is parsed straight from
the .kicad_pcb S-expression. Nothing here writes a KiCad file.
"""
from __future__ import annotations

import json
import re
import subprocess
import sys
import tempfile
import tomllib
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

# ---------------------------------------------------------------- S-expressions

_TOKEN = re.compile(r'\s*(?:(\()|(\))|"((?:[^"\\]|\\.)*)"|([^\s()"]+))', re.S)


class Sym(str):
    """An unquoted atom, so `(hide yes)` and `"yes"` stay distinguishable."""


def parse_sexpr(text: str) -> list:
    stack: list[list] = [[]]
    pos = 0
    n = len(text)
    while pos < n:
        m = _TOKEN.match(text, pos)
        if not m:
            if text[pos:].strip():
                raise ValueError(f"bad S-expression near offset {pos}")
            break
        pos = m.end()
        if m.group(1):
            stack.append([])
        elif m.group(2):
            if len(stack) == 1:
                line = text.count("\n", 0, pos) + 1
                raise ValueError(f"unbalanced ')' at line {line}: the file is malformed "
                                 "(open and re-save it in KiCad)")
            done = stack.pop()
            stack[-1].append(done)
        elif m.group(3) is not None:
            stack[-1].append(re.sub(r'\\(.)', lambda e: '\n' if e.group(1) == 'n' else e.group(1), m.group(3)))
        else:
            stack[-1].append(Sym(m.group(4)))
    if len(stack) != 1:
        raise ValueError("unbalanced parentheses")
    return stack[0][0]


def head(node) -> str | None:
    return node[0] if isinstance(node, list) and node else None


def children(node, name: str) -> list[list]:
    return [c for c in node[1:] if head(c) == name]


def child(node, name: str):
    for c in node[1:]:
        if head(c) == name:
            return c
    return None


def flag(node, name: str) -> bool:
    """True for `(name yes)`, `(name)` or a bare `name` atom inside node."""
    for c in node[1:]:
        if c == name:
            return True
        if head(c) == name:
            return len(c) == 1 or c[1] in ("yes", "true")
    return False


# ---------------------------------------------------------------- schematic

@dataclass
class SchPart:
    ref: str
    value: str
    footprint: str
    fields: dict[str, str]
    dnp: bool
    exclude_from_bom: bool
    exclude_from_board: bool
    lib: str


@dataclass
class SchNet:
    name: str
    netclass: str
    nodes: list[tuple[str, str, str]]  # (ref, pin, pintype)

    @property
    def named(self) -> bool:
        """KiCad invents names like Net-(R1-Pad1) and unconnected-(U1-NC-Pad5)."""
        return not re.match(r'^/?(.*/)?(Net-\(|unconnected-\()', self.name)


@dataclass
class Schematic:
    parts: dict[str, SchPart]
    nets: list[SchNet]


def load_schematic(sch: Path) -> Schematic:
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "net.xml"
        r = subprocess.run(
            ["kicad-cli", "sch", "export", "netlist", "--format", "kicadxml",
             "-o", str(out), str(sch)],
            capture_output=True, text=True)
        if r.returncode != 0 or not out.exists():
            sys.exit(f"kicad-cli could not export a netlist from {sch}:\n{r.stderr}")
        root = ET.parse(out).getroot()

    parts = {}
    for c in root.iter("comp"):
        props = {p.get("name"): p.get("value", "") for p in c.findall("property")}
        fields = {f.get("name"): (f.text or "") for f in c.iter("field")}
        lib = c.find("libsource")
        parts[c.get("ref")] = SchPart(
            ref=c.get("ref"),
            value=(c.findtext("value") or ""),
            footprint=(c.findtext("footprint") or ""),
            fields=fields,
            dnp="dnp" in props,
            exclude_from_bom="exclude_from_bom" in props,
            exclude_from_board="exclude_from_board" in props,
            lib=f'{lib.get("lib")}:{lib.get("part")}' if lib is not None else "",
        )
    nets = []
    for n in root.iter("net"):
        nets.append(SchNet(
            name=n.get("name"),
            netclass=n.get("class", "Default"),
            nodes=[(x.get("ref"), x.get("pin"), x.get("pintype", "")) for x in n.findall("node")],
        ))
    return Schematic(parts, nets)


# ---------------------------------------------------------------- board

@dataclass
class PcbFootprint:
    ref: str
    lib_id: str
    value: str
    properties: dict[str, str]
    layer: str
    at: tuple[float, float, float]
    attrs: set[str]
    node: list = field(repr=False)

    @property
    def dnp(self) -> bool:
        return "dnp" in self.attrs

    @property
    def exclude_from_bom(self) -> bool:
        return "exclude_from_bom" in self.attrs

    @property
    def board_only(self) -> bool:
        return "board_only" in self.attrs


@dataclass
class Board:
    footprints: dict[str, PcbFootprint]
    duplicates: list[str]
    tree: list = field(repr=False)


def load_board(pcb: Path) -> Board:
    try:
        tree = parse_sexpr(pcb.read_text(encoding="utf-8"))
    except ValueError as e:
        sys.exit(f"{pcb}: {e}")
    fps: dict[str, PcbFootprint] = {}
    dups = []
    for fp in children(tree, "footprint"):
        props = {p[1]: (p[2] if len(p) > 2 else "") for p in children(fp, "property")}
        for ft in children(fp, "fp_text"):  # KiCad 7 and older keep these two as fp_text
            if ft[1] in ("reference", "value") and len(ft) > 2:
                props.setdefault(ft[1].capitalize(), ft[2])
        at = child(fp, "at")
        attr = child(fp, "attr")
        ref = props.get("Reference", "")
        f = PcbFootprint(
            ref=ref,
            lib_id=str(fp[1]),
            value=props.get("Value", ""),
            properties=props,
            layer=str(child(fp, "layer")[1]),
            at=(float(at[1]), float(at[2]), float(at[3]) if len(at) > 3 else 0.0),
            attrs={str(a) for a in attr[1:]} if attr else set(),
            node=fp,
        )
        if ref in fps and not (f.board_only or fps[ref].board_only):
            dups.append(ref)
        if ref not in fps or not f.board_only:
            fps[ref] = f
    return Board(fps, dups, tree)


# ---------------------------------------------------------------- project

def find_project(path: Path, need_pcb: bool = True) -> tuple[Path, Path, Path]:
    """Accept a .kicad_pro, .kicad_sch, .kicad_pcb or a variant folder; return (pro, sch, pcb)."""
    path = path.resolve()
    if path.is_dir():
        pros = sorted(path.glob("*.kicad_pro"))
        if len(pros) != 1:
            sys.exit(f"{path}: expected exactly one .kicad_pro, found {len(pros)}")
        path = pros[0]
    stem = path.with_suffix("")
    pro, sch, pcb = (stem.with_suffix(s) for s in (".kicad_pro", ".kicad_sch", ".kicad_pcb"))
    for p in (sch, pcb) if need_pcb else (sch,):
        if not p.exists():
            sys.exit(f"{p} not found")
    return pro, sch, pcb


def load_pro(pro: Path) -> dict:
    return json.loads(pro.read_text()) if pro.exists() else {}


def repo_root(start: Path) -> Path:
    """The project root: the nearest folder above `start` that has design/."""
    for p in [start.resolve(), *start.resolve().parents]:
        if (p / "design").is_dir():
            return p
    return start.resolve().parent


# Defaults for design/checks.toml. A project overrides a key by setting it
# there, with a comment giving the reason (log it in decisions.md too).
DEFAULTS = {
    "fab": {
        "min_track_mm": 0.127,       # JLCPCB 2/4-layer standard (0.09 is their absolute floor)
        "min_clearance_mm": 0.127,
        "min_via_drill_mm": 0.3,
        "min_silk_height_mm": 0.8,   # JLC: 1.0 recommended, 0.8 legible
        "min_silk_width_mm": 0.15,
    },
    "copper": {
        "outer_oz": 1.0,
        "inner_oz": 0.5,
        "via_drill_mm": 0.3,
        "via_plating_mm": 0.020,     # IPC-6012 class 2 average barrel plating
    },
    "nets": {
        "temp_rise_c": 10.0,
        "via_safety": 1.5,
        "clearance_table": "B2",     # IPC-2221 Table 6-1: B2 external uncoated, B1 internal
    },
    "style": {
        "label_wire_mm": 25.0,       # two labels of one net closer than this: draw the wire
        "decoupler_mm": 40.0,        # a Decouples cap drawn farther than this from its pin
    },
    "silk": {
        "connector_prefixes": ["J", "P", "CN", "USB"],
        "connector_label_mm": 6.0,   # a label must start within this distance of the connector
        "max_rotations": 2,
        "identity": ["name", "rev", "date"],  # plus "repo" and "license" for open source
    },
}


def load_checks(root: Path) -> dict:
    cfg = json.loads(json.dumps(DEFAULTS))
    path = root / "design" / "checks.toml"
    if path.exists():
        with path.open("rb") as f:
            user = tomllib.load(f)
        for section, values in user.items():
            cfg.setdefault(section, {}).update(values)
    return cfg


# ---------------------------------------------------------------- reporting

class Report:
    """Collects findings; prints a summary for the gate file and optional JSON."""

    def __init__(self, name: str, target: str):
        self.name = name
        self.target = target
        self.errors: list[dict] = []
        self.warnings: list[dict] = []
        self.info: list[str] = []
        self.sections: list[tuple[str, list[str]]] = []

    def error(self, rule: str, msg: str, **kw):
        self.errors.append({"rule": rule, "msg": msg, **kw})

    def warn(self, rule: str, msg: str, **kw):
        self.warnings.append({"rule": rule, "msg": msg, **kw})

    def note(self, msg: str):
        self.info.append(msg)

    def table(self, title: str, lines: list[str]):
        self.sections.append((title, lines))

    def finish(self, json_path: str | None) -> int:
        ok = not self.errors
        print(f"{self.name}: {self.target}")
        for msg in self.info:
            print(f"  {msg}")
        for title, lines in self.sections:
            print(f"\n  {title}")
            for line in lines:
                print(f"    {line}")
        if self.errors:
            print(f"\n  ERRORS ({len(self.errors)})")
            for e in self.errors:
                print(f"    [{e['rule']}] {e['msg']}")
        if self.warnings:
            print(f"\n  WARNINGS ({len(self.warnings)})")
            for w in self.warnings:
                print(f"    [{w['rule']}] {w['msg']}")
        print(f"\n  {'PASS' if ok else 'FAIL'}: {len(self.errors)} errors, {len(self.warnings)} warnings")
        if json_path:
            Path(json_path).write_text(json.dumps({
                "check": self.name, "target": self.target, "pass": ok,
                "errors": self.errors, "warnings": self.warnings, "info": self.info,
            }, indent=2) + "\n")
        return 0 if ok else 1
