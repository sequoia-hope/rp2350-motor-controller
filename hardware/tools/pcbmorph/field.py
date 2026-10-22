"""Per-layer 2D harmonic warp field (Beltrami / Laplace solver) and multi-layer stack.

Solves Laplace's equation on each copper layer:
    ∇²r_L = 0   on Ω \\ ∪ I_{k,L}
    r_L|_{I_{k,L}} = q_{k,L}(p) = p + v_{k,L}     (Dirichlet on rigid/steered inclusions)
    r_L|_{∂Ω}      = q_∞(p)                       (ambient boundary condition)

Because the discrete Laplacian matrix A depends only on the grid geometry and
inclusion mask (owner >= 0), while the right-hand side depends linearly on the
inclusion displacement vectors v_{k,L}, we factor A once with SuperLU (splu)
and re-solve for updated footprint displacements in ~1-2 ms per copper layer.

LayerFieldStack couples per-layer WarpFields across all active copper layers:
- Every drill hole (static or dynamic pad drill, static via) is pinned across ALL layers.
- Footprint SMD pads only constrain the copper layers they exist on; a trace on
  In1.Cu under an F.Cu SMD pad flows unimpeded unless a via or through-hole pin
  couples the layers.
- Dynamic vias are solved in a 2-pass barrel-coupled update: Pass 1 evaluates the
  layer-averaged harmonic displacement at each free via's initial coordinate, and
  Pass 2 pins or blends that coupled barrel vector across all layers.
"""
import math
import numpy as np
from scipy import sparse
from scipy.sparse.linalg import spsolve, splu
from scipy.ndimage import binary_dilation, binary_closing

from .common import COPPER_LAYERS, F_CU, B_CU, LAYER_NAMES


# ---------------------------------------------------------------------------
# Ambient boundary maps & analytical dipoles
# ---------------------------------------------------------------------------

def identity():
    def q(x, y):
        return np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    return q


def dilation(lam_x, lam_y=None, cx=0.0, cy=0.0):
    """Uniform (or anisotropic) scaling about (cx, cy)."""
    if lam_y is None:
        lam_y = lam_x
    def q(x, y):
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        return cx + lam_x * (x - cx), cy + lam_y * (y - cy)
    return q


def dipole(sx, sy, dx, dy, radius=8.0, decay=2.0):
    """Smooth localized push field ("create space here in direction (dx, dy)").

    Used either as an ambient field or to seed target displacements for parts
    near (sx, sy) with smoothly decaying influence.
    """
    r2 = max(radius * radius, 1e-6)
    def q(x, y):
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        dist2 = (x - sx) ** 2 + (y - sy) ** 2
        w = (1.0 + dist2 / r2) ** (-0.5 * decay)
        return x + dx * w, y + dy * w
    return q


def compose_ambient(spec, bbox):
    """Build an ambient boundary map q(x, y) from a config dict."""
    if not spec:
        return identity()
    kind = spec.get('type', 'identity')
    x0, y0, x1, y1 = bbox
    cx = spec.get('cx', 0.5 * (x0 + x1))
    cy = spec.get('cy', 0.5 * (y0 + y1))
    if kind == 'dilation':
        lx = float(spec.get('lam_x', spec.get('scale', 1.0)))
        ly = float(spec.get('lam_y', lx))
        return dilation(lx, ly, cx, cy)
    if kind == 'shift':
        dx = float(spec.get('dx', 0.0))
        dy = float(spec.get('dy', 0.0))
        return lambda x, y: (np.asarray(x, dtype=float) + dx,
                             np.asarray(y, dtype=float) + dy)
    if kind == 'dipole':
        return dipole(float(spec.get('x', cx)), float(spec.get('y', cy)),
                      float(spec.get('dx', 0.0)), float(spec.get('dy', 0.0)),
                      float(spec.get('radius', 8.0)), float(spec.get('decay', 2.0)))
    return identity()


# ---------------------------------------------------------------------------
# 2x2 singular values (vectorized)
# ---------------------------------------------------------------------------

def sigma_min_2x2(a, b, c, d):
    """Smallest singular value of [[a, b], [c, d]], elementwise."""
    s1 = a * a + b * b + c * c + d * d
    det = a * d - b * c
    disc = np.maximum(s1 * s1 - 4.0 * det * det, 0.0)
    lam2 = 0.5 * np.maximum(s1 - np.sqrt(disc), 0.0)
    return np.sqrt(lam2)


def sigma_max_2x2(a, b, c, d):
    s1 = a * a + b * b + c * c + d * d
    det = a * d - b * c
    disc = np.maximum(s1 * s1 - 4.0 * det * det, 0.0)
    lam1 = 0.5 * (s1 + np.sqrt(disc))
    return np.sqrt(np.maximum(lam1, 0.0))


# ---------------------------------------------------------------------------
# Single-layer WarpField
# ---------------------------------------------------------------------------

class WarpField:
    """2D harmonic warp field on a regular grid over [x0, x1] x [y0, y1] (mm).

    Supports both one-shot solving (`solve()`) and fast repeated solves with
    cached LU factorization (`factorize()` + `update_vectors()`) when inclusion
    masks are fixed and only inclusion displacement vectors change.
    """

    def __init__(self, bbox, h=0.25, ambient=None):
        x0, y0, x1, y1 = bbox
        self.x0, self.y0 = float(x0), float(y0)
        self.h = float(h)
        self.nx = int(math.ceil((x1 - x0) / h)) + 1
        self.ny = int(math.ceil((y1 - y0) / h)) + 1
        self.x1 = self.x0 + (self.nx - 1) * self.h
        self.y1 = self.y0 + (self.ny - 1) * self.h

        self.xs = self.x0 + np.arange(self.nx) * self.h
        self.ys = self.y0 + np.arange(self.ny) * self.h
        self.X, self.Y = np.meshgrid(self.xs, self.ys)  # shape (ny, nx)

        self.ambient = ambient if ambient is not None else identity()

        # owner[j, i] == -1 for free nodes, >=0 for inclusion index
        self.owner = np.full((self.ny, self.nx), -1, dtype=np.int32)
        self.inclusions = []   # list of dicts: {name, mask, vx, vy, group}

        self.rx = None
        self.ry = None
        self.ux = None
        self.uy = None

        # Cached linear system structures for fast vector updates
        self._lu = None
        self._owner_flat = None
        self._border_idx = None
        self._border_rx = None
        self._border_ry = None
        self._X_flat = None
        self._Y_flat = None

    # ---- Inclusion registration --------------------------------------------

    def _claim(self, mask, vx, vy, name="", group=None, overwrite=False):
        idx = len(self.inclusions)
        if overwrite:
            claim = np.asarray(mask, dtype=bool)
        else:
            claim = np.asarray(mask, dtype=bool) & (self.owner < 0)
        self.owner[claim] = idx
        self.inclusions.append({
            "name": name,
            "mask": claim,
            "vx": float(vx),
            "vy": float(vy),
            "group": group if group is not None else name,
        })
        self._lu = None  # invalidate factorization when geometry changes
        return idx

    def add_mask(self, mask, vx=0.0, vy=0.0, name="", group=None, overwrite=False):
        return self._claim(mask, vx, vy, name=name, group=group, overwrite=overwrite)

    def add_rect(self, cx, cy, w, h, vx=0.0, vy=0.0, pad=0.0, name="", group=None):
        hw = 0.5 * w + pad
        hh = 0.5 * h + pad
        mask = (np.abs(self.X - cx) <= hw) & (np.abs(self.Y - cy) <= hh)
        if not mask.any():
            i = int(round((cx - self.x0) / self.h))
            j = int(round((cy - self.y0) / self.h))
            if 0 <= i < self.nx and 0 <= j < self.ny:
                mask[j, i] = True
        return self._claim(mask, vx, vy, name=name, group=group)

    def add_disk(self, cx, cy, radius, vx=0.0, vy=0.0, name="", group=None):
        r = max(radius, 0.5 * self.h)
        mask = (self.X - cx) ** 2 + (self.Y - cy) ** 2 <= r * r
        if not mask.any():
            i = int(round((cx - self.x0) / self.h))
            j = int(round((cy - self.y0) / self.h))
            if 0 <= i < self.nx and 0 <= j < self.ny:
                mask[j, i] = True
        return self._claim(mask, vx, vy, name=name, group=group)

    def rasterize_rect(self, cx, cy, w, h, pad=0.0):
        hw = 0.5 * w + pad
        hh = 0.5 * h + pad
        mask = (np.abs(self.X - cx) <= hw) & (np.abs(self.Y - cy) <= hh)
        if not mask.any():
            i = int(round((cx - self.x0) / self.h))
            j = int(round((cy - self.y0) / self.h))
            if 0 <= i < self.nx and 0 <= j < self.ny:
                mask[j, i] = True
        return mask

    def rasterize_disk(self, cx, cy, radius):
        r = max(radius, 0.5 * self.h)
        mask = (self.X - cx) ** 2 + (self.Y - cy) ** 2 <= r * r
        if not mask.any():
            i = int(round((cx - self.x0) / self.h))
            j = int(round((cy - self.y0) / self.h))
            if 0 <= i < self.nx and 0 <= j < self.ny:
                mask[j, i] = True
        return mask

    def close_mask(self, mask, bridge_mm=1.0):
        rad = max(1, int(round(bridge_mm / (2.0 * self.h))))
        if rad <= 0 or not mask.any():
            return mask
        yy, xx = np.ogrid[-rad:rad + 1, -rad:rad + 1]
        se = (xx * xx + yy * yy) <= rad * rad
        return binary_closing(mask, structure=se)

    def resolve_touching_inclusions(self, tol=1e-4):
        """Ensure a >=1-cell free buffer between inclusions with distinct vectors."""
        struct = np.ones((3, 3), dtype=bool)
        for idx, inc in enumerate(self.inclusions):
            m = inc["mask"]
            if not m.any():
                continue
            dil = binary_dilation(m, structure=struct) & ~m
            nbr_owners = self.owner[dil]
            bad = False
            for o in np.unique(nbr_owners):
                if o < 0 or o == idx:
                    continue
                other = self.inclusions[o]
                if abs(other["vx"] - inc["vx"]) > tol or abs(other["vy"] - inc["vy"]) > tol:
                    bad = True
                    break
            if bad:
                conflict = np.zeros_like(m)
                for o in np.unique(nbr_owners):
                    if o < 0 or o == idx:
                        continue
                    other = self.inclusions[o]
                    if abs(other["vx"] - inc["vx"]) > tol or abs(other["vy"] - inc["vy"]) > tol:
                        conflict |= self.owner == o
                adj_to_conflict = binary_dilation(conflict, structure=struct) & m
                keep = m & ~adj_to_conflict
                if keep.any():
                    self.owner[adj_to_conflict] = -1
                    inc["mask"] = keep
        self._lu = None

    # ---- Elliptic solver (with cached SuperLU factorization) ---------------

    def _build_matrix(self):
        ny, nx = self.ny, self.nx
        N = ny * nx

        border = np.zeros((ny, nx), dtype=bool)
        border[0, :] = border[-1, :] = True
        border[:, 0] = border[:, -1] = True

        dirichlet = border | (self.owner >= 0)
        free = ~dirichlet

        idx = np.arange(N, dtype=np.int32).reshape(ny, nx)
        f_idx = idx[free]
        d_idx = idx[dirichlet]

        rows = np.concatenate([
            d_idx,
            f_idx,
            f_idx, f_idx, f_idx, f_idx,
        ])
        cols = np.concatenate([
            d_idx,
            f_idx,
            f_idx - 1, f_idx + 1, f_idx - nx, f_idx + nx,
        ])
        vals = np.concatenate([
            np.ones(len(d_idx), dtype=np.float64),
            np.full(len(f_idx), 4.0, dtype=np.float64),
            np.full(len(f_idx), -1.0, dtype=np.float64),
            np.full(len(f_idx), -1.0, dtype=np.float64),
            np.full(len(f_idx), -1.0, dtype=np.float64),
            np.full(len(f_idx), -1.0, dtype=np.float64),
        ])
        A = sparse.csc_matrix((vals, (rows, cols)), shape=(N, N))

        self._owner_flat = self.owner.ravel()
        self._X_flat = self.X.ravel()
        self._Y_flat = self.Y.ravel()

        b_flat = border.ravel() & (self._owner_flat < 0)
        self._border_idx = np.nonzero(b_flat)[0]
        self._border_rx, self._border_ry = self.ambient(
            self._X_flat[self._border_idx], self._Y_flat[self._border_idx]
        )
        return A

    def factorize(self):
        """Pre-factor the discrete Laplace operator for fast repeated vector updates."""
        A = self._build_matrix()
        self._lu = splu(A)
        return self

    def _assemble_rhs(self):
        ny, nx = self.ny, self.nx
        N = ny * nx
        rhsx = np.zeros(N, dtype=np.float64)
        rhsy = np.zeros(N, dtype=np.float64)

        if len(self._border_idx):
            rhsx[self._border_idx] = self._border_rx
            rhsy[self._border_idx] = self._border_ry

        inc_mask = self._owner_flat >= 0
        if inc_mask.any():
            vx_lut = np.array([inc["vx"] for inc in self.inclusions], dtype=np.float64)
            vy_lut = np.array([inc["vy"] for inc in self.inclusions], dtype=np.float64)
            owners = self._owner_flat[inc_mask]
            rhsx[inc_mask] = self._X_flat[inc_mask] + vx_lut[owners]
            rhsy[inc_mask] = self._Y_flat[inc_mask] + vy_lut[owners]
        return rhsx, rhsy

    def solve(self, cache_lu=False):
        """Solve ∇²rx = 0, ∇²ry = 0 on the grid."""
        if self._lu is None:
            A = self._build_matrix()
            if cache_lu:
                self._lu = splu(A)
        rhsx, rhsy = self._assemble_rhs()
        if self._lu is not None:
            rx = self._lu.solve(rhsx).reshape(self.ny, self.nx)
            ry = self._lu.solve(rhsy).reshape(self.ny, self.nx)
        else:
            rx = spsolve(A, rhsx).reshape(self.ny, self.nx)
            ry = spsolve(A, rhsy).reshape(self.ny, self.nx)
        self.rx = rx
        self.ry = ry
        self.ux = rx - self.X
        self.uy = ry - self.Y
        return self

    def update_vectors(self, group_vecs):
        """Update inclusion (vx, vy) by group name and re-solve using cached LU."""
        for inc in self.inclusions:
            g = inc["group"]
            if g in group_vecs:
                vx, vy = group_vecs[g]
                inc["vx"] = float(vx)
                inc["vy"] = float(vy)
        if self._lu is None:
            self.factorize()
        rhsx, rhsy = self._assemble_rhs()
        self.rx = self._lu.solve(rhsx).reshape(self.ny, self.nx)
        self.ry = self._lu.solve(rhsy).reshape(self.ny, self.nx)
        self.ux = self.rx - self.X
        self.uy = self.ry - self.Y
        return self

    # ---- Bilinear evaluation & diagnostics ---------------------------------

    def _bilerp(self, F, x, y):
        x_arr = np.asarray(x, dtype=np.float64)
        y_arr = np.asarray(y, dtype=np.float64)
        scalar = x_arr.ndim == 0 and y_arr.ndim == 0
        x_arr = np.atleast_1d(x_arr)
        y_arr = np.atleast_1d(y_arr)

        fx = np.clip((x_arr - self.x0) / self.h, 0.0, self.nx - 1 - 1e-9)
        fy = np.clip((y_arr - self.y0) / self.h, 0.0, self.ny - 1 - 1e-9)
        i0 = fx.astype(np.int32)
        j0 = fy.astype(np.int32)
        tx = fx - i0
        ty = fy - j0

        v00 = F[j0, i0]
        v10 = F[j0, i0 + 1]
        v01 = F[j0 + 1, i0]
        v11 = F[j0 + 1, i0 + 1]
        out = ((1 - tx) * (1 - ty) * v00 +
               tx * (1 - ty) * v10 +
               (1 - tx) * ty * v01 +
               tx * ty * v11)
        return float(out[0]) if scalar else out

    def eval(self, x, y):
        """Evaluate deformed position r(x, y) via bilinear interpolation."""
        return self._bilerp(self.rx, x, y), self._bilerp(self.ry, x, y)

    def eval_u(self, x, y):
        """Evaluate displacement u(x, y) = r(x, y) - (x, y) via bilinear interpolation."""
        return self._bilerp(self.ux, x, y), self._bilerp(self.uy, x, y)

    def jacobian_grid(self):
        drx_dx = np.gradient(self.rx, self.h, axis=1)
        drx_dy = np.gradient(self.rx, self.h, axis=0)
        dry_dx = np.gradient(self.ry, self.h, axis=1)
        dry_dy = np.gradient(self.ry, self.h, axis=0)
        det = drx_dx * dry_dy - drx_dy * dry_dx
        smin = sigma_min_2x2(drx_dx, drx_dy, dry_dx, dry_dy)
        smax = sigma_max_2x2(drx_dx, drx_dy, dry_dx, dry_dy)
        return drx_dx, drx_dy, dry_dx, dry_dy, det, smin, smax

    def check_admissibility(self):
        _, _, _, _, det, smin, smax = self.jacobian_grid()
        free = self.owner < 0
        if not free.any():
            return {"invertible": True, "min_det": 1.0, "min_sigma": 1.0, "max_sigma": 1.0, "folded_cells": 0}
        return {
            "invertible": bool(np.all(det[free] > 0)),
            "min_det": float(np.min(det[free])),
            "min_sigma": float(np.min(smin[free])),
            "p05_sigma": float(np.percentile(smin[free], 5)),
            "max_sigma": float(np.max(smax[free])),
            "folded_cells": int(np.sum(det[free] <= 0)),
        }


# ---------------------------------------------------------------------------
# Multi-layer coupled harmonic field stack for Settle
# ---------------------------------------------------------------------------

class LayerFieldStack:
    """Coupled per-copper-layer WarpField stack built from a pcbmorph model dict.

    Provides smooth, topology-preserving harmonic displacement targets U_L(x, y)
    for all free copper graph nodes (trace vertices and dynamic vias) as parts move.

    Key physics:
    1. Every footprint's pads (inflated by pad_inflate + closed across intra-part gaps)
       form a rigid inclusion on the specific copper layers those pads inhabit, moving
       with the footprint's displacement vector V_r.
    2. All through-hole drills and static vias are pinned across ALL copper layers
       so no layer's harmonic flow sweeps traces across a stationary barrel.
    3. Dynamic vias are coupled across layers: after solving each layer's harmonic
       field from the footprint vectors, each free via's displacement is averaged
       across the active copper layers it spans, so the via and its attached traces
       on both layers move toward a shared barrel target.
    """

    def __init__(self, model, h=0.25, pad_inflate=0.15, bridge_mm=1.0, ambient_spec=None):
        self.h = float(h)
        self.pad_inflate = float(pad_inflate)
        self.bridge_mm = float(bridge_mm)

        # Compute bounding box from board outline or pads/nodes
        outline = model.get('outline', [])
        if outline:
            xs = [p[0] for p in outline]
            ys = [p[1] for p in outline]
            bbox = (min(xs) - 1.0, min(ys) - 1.0, max(xs) + 1.0, max(ys) + 1.0)
        else:
            xs = [n['x'] for n in model['nodes']] + [p['x'] for p in model['pads']]
            ys = [n['y'] for n in model['nodes']] + [p['y'] for p in model['pads']]
            bbox = (min(xs) - 5.0, min(ys) - 5.0, max(xs) + 5.0, max(ys) + 5.0)
        self.bbox = bbox
        self.ambient = compose_ambient(ambient_spec, bbox)

        # Determine active copper layers present in the graph
        active_layers = sorted({
            s['layer'] for s in model.get('segs', [])
        } | {
            z['layer'] for z in model.get('zones', [])
        })
        if not active_layers:
            active_layers = [F_CU, B_CU]
        self.layers = active_layers

        # Identify movable refs and initial node coordinates
        self.movable_refs = set(model.get('parts', {}).keys())
        self.node_p0 = [(float(n['x']), float(n['y'])) for n in model['nodes']]
        self.node_fixed = {int(n['id']) for n in model['nodes'] if n.get('fixed')}
        self.node_part = {int(n['id']): n['part'] for n in model['nodes'] if n.get('part')}
        self.node_via = {int(n['id']) for n in model['nodes'] if n.get('kind') == 'via'}

        # Determine layers incident on each node from segs and attachments
        self.node_layers = [set() for _ in model['nodes']]
        for s in model.get('segs', []):
            l = int(s['layer'])
            self.node_layers[int(s['a'])].add(l)
            self.node_layers[int(s['b'])].add(l)
        for a in model.get('attachments', []):
            if a.get('node') is not None and a.get('layer', -1) in self.layers:
                self.node_layers[int(a['node'])].add(int(a['layer']))
        for nid in self.node_via:
            if not self.node_layers[nid]:
                self.node_layers[nid] = set(self.layers)

        # Build and factorize one WarpField per active copper layer
        self.fields = {}
        self.edge_vecs = {}
        self._build_layer_fields(model)

        # Cached harmonic displacement per node (ux, uy)
        self.node_u = [(0.0, 0.0)] * len(model['nodes'])
        self.last_vectors = {}

    def _build_layer_fields(self, model):
        # Group pads by ref
        pads_by_ref = {}
        for pad in model.get('pads', []):
            pads_by_ref.setdefault(pad['ref'], []).append(pad)

        # Collect static via coordinates (fixed via nodes + static vias)
        static_vias = []
        for n in model.get('nodes', []):
            if n.get('kind') == 'via' and n.get('fixed'):
                static_vias.append((float(n['x']), float(n['y']), 0.5 * float(n.get('size', 0.6))))
        for sv in model.get('static_vias', []):
            static_vias.append((float(sv['x']), float(sv['y']), 0.5 * float(sv.get('size', 0.6))))

        # Collect deformed board-outline segments (if any)
        o0 = model.get('outline', [])
        o1 = model.get('target_outline', o0)
        edge_segs = []
        if o0 and o1 and len(o0) == len(o1):
            n_ol = len(o0)
            for k in range(n_ol):
                p0a, p0b = o0[k], o0[(k + 1) % n_ol]
                p1a, p1b = o1[k], o1[(k + 1) % n_ol]
                dxa, dya = p1a[0] - p0a[0], p1a[1] - p0a[1]
                dxb, dyb = p1b[0] - p0b[0], p1b[1] - p0b[1]
                if math.hypot(dxa, dya) > 1e-4 or math.hypot(dxb, dyb) > 1e-4:
                    gname = f"__edge_{k}__"
                    # Gain > 1.0 on edge Dirichlet condition so copper interior at ~0.8 mm
                    # from the edge moves inward by >= the full notch depth + edge clearance
                    self.edge_vecs[gname] = (0.5 * (dxa + dxb) * 1.25, 0.5 * (dya + dyb) * 1.25)
                    edge_segs.append((gname, p0a, p0b))

        for layer in self.layers:
            wf = WarpField(self.bbox, h=self.h, ambient=self.ambient)

            # 1. Register movable footprints FIRST so their pads (and any drills) get their group
            for ref in sorted(self.movable_refs):
                rpads = pads_by_ref.get(ref, [])
                mask = np.zeros((wf.ny, wf.nx), dtype=bool)
                for p in rpads:
                    on_layer = (layer in p.get('layers', [])) or (p.get('drill', 0.0) > 0)
                    if not on_layer:
                        continue
                    poly = p.get('poly', [])
                    if poly:
                        xs = [pt[0] for pt in poly]
                        ys = [pt[1] for pt in poly]
                        w = max(xs) - min(xs)
                        h = max(ys) - min(ys)
                        mask |= wf.rasterize_rect(p['x'], p['y'], w, h, pad=self.pad_inflate)
                    elif p.get('drill', 0.0) > 0:
                        mask |= wf.rasterize_disk(p['x'], p['y'], 0.5 * p['drill'] + self.pad_inflate)
                if mask.any():
                    if self.bridge_mm > 0:
                        mask = wf.close_mask(mask, bridge_mm=self.bridge_mm)
                    wf.add_mask(mask, vx=0.0, vy=0.0, name=ref, group=ref)

            # 1b. Register deformed board-edge segments across all layers
            for gname, p0a, p0b in edge_segs:
                emask = np.zeros((wf.ny, wf.nx), dtype=bool)
                seg_len = math.hypot(p0b[0] - p0a[0], p0b[1] - p0a[1])
                n_s = max(2, int(math.ceil(seg_len / (0.5 * self.h))) + 1)
                for si in range(n_s):
                    t = si / max(1, n_s - 1)
                    sx = p0a[0] + t * (p0b[0] - p0a[0])
                    sy = p0a[1] + t * (p0b[1] - p0a[1])
                    emask |= wf.rasterize_disk(sx, sy, 0.35)
                if emask.any():
                    wf.add_mask(emask, vx=0.0, vy=0.0, name=gname, group=gname)

            # 2. Register all static pads (on this layer) and all static drills (across ALL layers)
            static_mask = np.zeros((wf.ny, wf.nx), dtype=bool)
            for ref, rpads in pads_by_ref.items():
                if ref in self.movable_refs:
                    continue
                for p in rpads:
                    on_layer = (layer in p.get('layers', [])) or (p.get('drill', 0.0) > 0)
                    if not on_layer:
                        continue
                    poly = p.get('poly', [])
                    if poly:
                        xs = [pt[0] for pt in poly]
                        ys = [pt[1] for pt in poly]
                        w = max(xs) - min(xs)
                        h = max(ys) - min(ys)
                        static_mask |= wf.rasterize_rect(p['x'], p['y'], w, h, pad=0.05)
                    elif p.get('drill', 0.0) > 0:
                        static_mask |= wf.rasterize_disk(p['x'], p['y'], 0.5 * p['drill'] + 0.05)

            # 3. Register all static vias across all layers
            for vx, vy, vr in static_vias:
                static_mask |= wf.rasterize_disk(vx, vy, vr + 0.05)

            if static_mask.any():
                wf.add_mask(static_mask, vx=0.0, vy=0.0, name="__static__", group="__static__")

            wf.resolve_touching_inclusions()
            wf.factorize()
            wf.solve()
            self.fields[layer] = wf

    def update(self, part_vecs, s_edge=0.0):
        """Re-solve all copper layers for the given {ref: (vx, vy)} displacement map.

        Also evaluates the coupled harmonic displacement u(nid) for every node,
        averaging across active layers for dynamic vias (barrel coupling).
        """
        all_vecs = dict(part_vecs)
        if self.edge_vecs:
            for gname, (evx, evy) in self.edge_vecs.items():
                all_vecs[gname] = (evx * s_edge, evy * s_edge)
        for layer, wf in self.fields.items():
            wf.update_vectors(all_vecs)

        # Evaluate node displacements in batch per layer
        n_nodes = len(self.node_p0)
        xs = np.array([p[0] for p in self.node_p0], dtype=np.float64)
        ys = np.array([p[1] for p in self.node_p0], dtype=np.float64)

        layer_u = {}
        for layer, wf in self.fields.items():
            ux, uy = wf.eval_u(xs, ys)
            layer_u[layer] = (ux, uy)

        out = [(0.0, 0.0)] * n_nodes
        for nid in range(n_nodes):
            if nid in self.node_fixed:
                continue
            ref = self.node_part.get(nid)
            if ref is not None:
                out[nid] = part_vecs.get(ref, (0.0, 0.0))
                continue
            ls = [l for l in self.node_layers[nid] if l in layer_u]
            if not ls:
                ls = list(self.layers)
            if len(ls) == 1:
                l = ls[0]
                out[nid] = (float(layer_u[l][0][nid]), float(layer_u[l][1][nid]))
            else:
                # Dynamic via or multi-layer node: barrel-couple across incident layers
                ux_m = sum(float(layer_u[l][0][nid]) for l in ls) / len(ls)
                uy_m = sum(float(layer_u[l][1][nid]) for l in ls) / len(ls)
                out[nid] = (ux_m, uy_m)

        self.node_u = out
        self.last_vectors = dict(part_vecs)
        return out

    def target_pos(self, nid):
        """Return harmonic equilibrium position P_harm(nid) = P_0(nid) + U(nid)."""
        x0, y0 = self.node_p0[nid]
        ux, uy = self.node_u[nid]
        return x0 + ux, y0 + uy

    def admissibility_report(self):
        """Return per-layer Jacobian admissibility metrics (invertibility, min_sigma)."""
        rep = {}
        for layer, wf in self.fields.items():
            lname = LAYER_NAMES.get(layer, f"L{layer}")
            rep[lname] = wf.check_admissibility()
        return rep
