#!/usr/bin/env python3
"""jlc_parts.py -- look parts up in the JLCPCB assembly library.

    jlc_parts.py search "WS2812B-2020"          keyword search, best stock first
    jlc_parts.py search "100nF 0402" --basic    basic parts only
    jlc_parts.py show C52917434 C1525           one line per part, with lifecycle
    jlc_parts.py fetch C52917434 -o design/datasheets/ws2812b-2020.pdf
    jlc_parts.py fetch https://example.com/x.pdf -o design/datasheets/x.pdf
    jlc_parts.py fetch-list design/datasheets/sources.csv   every row: file,source

`show` is what goes in a part audit's Supplier line: LCSC number, library type
(basic or extended), stock, lifecycle and the date checked. A part whose
lifecycle is not "normal" or whose stock is below what the build needs fails
phase 1 even if the part is perfect. `fetch` downloads the datasheet LCSC
links, which works when the manufacturer's site blocks scripts; for a plain URL
it falls back to wget, which some sites accept when they refuse urllib.

The endpoints are the ones jlcpcb.com and lcsc.com use for their own pages.
They are not a published API and may change; when they break, look the part
up by hand and say so in the audit.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import json
import shutil
import subprocess
import sys
import urllib.request
from pathlib import Path

JLC_SEARCH = "https://jlcpcb.com/api/overseas-pcb-order/v1/shoppingCart/smtGood/selectSmtComponentList"
LCSC_DETAIL = "https://wmsc.lcsc.com/ftps/wm/product/detail?productCode={}"
UA = {"User-Agent": "Mozilla/5.0", "Content-Type": "application/json"}


def _get(url: str, body: dict | None = None, timeout=30) -> dict:
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, headers=UA)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)


def search(keyword: str, size: int = 25) -> list[dict]:
    d = _get(JLC_SEARCH, {"keyword": keyword, "currentPage": 1, "pageSize": size})
    rows = ((d.get("data") or {}).get("componentPageInfo") or {}).get("list") or []
    return [{
        "lcsc": c.get("componentCode"),
        "mpn": c.get("componentModelEn"),
        "mfr": c.get("componentBrandEn"),
        "type": "basic" if c.get("componentLibraryType") == "base" else
                "preferred" if c.get("preferredComponentFlag") else "extended",
        "stock": c.get("stockCount") or 0,
        "package": c.get("componentSpecificationEn"),
        "desc": c.get("describe") or "",
    } for c in rows]


def detail(code: str) -> dict:
    r = (_get(LCSC_DETAIL.format(code)).get("result")) or {}
    jlc = [p for p in search(code, 5) if p["lcsc"] == code]
    return {
        "lcsc": code,
        "mpn": r.get("productModel"),
        "mfr": r.get("brandNameEn"),
        "package": r.get("encapStandard"),
        "lifecycle": r.get("productCycle") or "unknown",
        "lcsc_stock": r.get("stockNumber"),
        "jlc_stock": jlc[0]["stock"] if jlc else None,
        "type": jlc[0]["type"] if jlc else "not in JLC library",
        "datasheet": r.get("pdfUrl"),
        "params": {p.get("paramNameEn"): p.get("paramValueEn") for p in (r.get("paramVOList") or [])},
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("search")
    s.add_argument("keyword")
    s.add_argument("--basic", action="store_true", help="basic and preferred parts only")
    s.add_argument("-n", type=int, default=15)
    sh = sub.add_parser("show")
    sh.add_argument("codes", nargs="+")
    sh.add_argument("--params", action="store_true", help="also print LCSC's parameter table")
    sh.add_argument("--json")
    f = sub.add_parser("fetch")
    f.add_argument("code")
    f.add_argument("-o", "--out", type=Path, required=True)
    fl = sub.add_parser("fetch-list")
    fl.add_argument("list", type=Path)
    fl.add_argument("--force", action="store_true", help="download files that are already present")
    args = ap.parse_args()

    if args.cmd == "search":
        rows = search(args.keyword, 50)
        if args.basic:
            rows = [r for r in rows if r["type"] != "extended"]
        rows.sort(key=lambda r: -r["stock"])
        for r in rows[:args.n]:
            print(f'{r["lcsc"]:<12} {r["type"]:<9} {r["stock"]:>9}  {r["mpn"]}  [{r["package"]}]  {r["mfr"] or ""}')
        return 0

    if args.cmd == "show":
        today = datetime.date.today().isoformat()
        out, bad = [], 0
        for code in args.codes:
            d = detail(code)
            out.append(d)
            flag = "" if d["lifecycle"] == "normal" and (d["jlc_stock"] or 0) > 0 else "  <-- CHECK"
            bad += bool(flag)
            print(f'{code} {d["mpn"]} ({d["mfr"]}, {d["package"]}): {d["type"]}, '
                  f'JLC stock {d["jlc_stock"]}, lifecycle {d["lifecycle"]}, checked {today}{flag}')
            if args.params:
                for k, v in d["params"].items():
                    print(f"    {k}: {v}")
        if args.json:
            Path(args.json).write_text(json.dumps(out, indent=2) + "\n")
        return 1 if bad else 0

    if args.cmd == "fetch":
        print(fetch(args.code, args.out))
        return 0

    root = args.list.parent
    bad = 0
    with args.list.open() as f:
        for row in csv.DictReader(f):
            out = root / row["file"]
            if out.exists() and not args.force:
                print(f"{row['file']}: present")
                continue
            try:
                print(fetch(row["source"].strip(), out))
            except (OSError, SystemExit, ValueError) as e:
                bad += 1
                print(f"{row['file']}: FAILED ({e}); fetch it by hand from {row['source']}")
    return 1 if bad else 0


def _download(url: str) -> bytes:
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
        with urllib.request.urlopen(req, timeout=60) as r:
            return r.read()
    except OSError:
        if not shutil.which("wget"):
            raise
        r = subprocess.run(["wget", "-q", "--timeout=60", "--tries=2", "-U", "Mozilla/5.0", "-O", "-", url],
                           capture_output=True)
        if r.returncode:
            raise OSError(f"wget exit {r.returncode}")
        return r.stdout


def fetch(source: str, out: Path) -> str:
    """source is an LCSC number (C123) or a URL."""
    if source.startswith("http"):
        url, name = source, source.rsplit("/", 1)[-1]
    else:
        d = detail(source)
        if not d["datasheet"]:
            raise ValueError(f"{source}: LCSC lists no datasheet")
        url, name = d["datasheet"], f"{source} {d['mpn']}"
    data = _download(url)
    if not data.startswith(b"%PDF"):
        raise ValueError(f"{url} did not return a PDF")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(data)
    return f"{name}: {len(data) // 1024} kB -> {out}"

if __name__ == "__main__":
    sys.exit(main())
