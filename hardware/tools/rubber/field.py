#!/usr/bin/env python3
"""Spatial warp fields: a displacement field u(x,y) over the board plane.

    u(x) = ambient(x) + r(x)

  * u = v_i everywhere inside rigid inclusion i (a footprint: its pads must keep
    their relative geometry, so the whole territory translates as one),
  * r is harmonic (Laplace) in free space, so the rigid-inclusion lag decays
    through SPACE,
  * r = 0 on the domain boundary, so the far field is exactly `ambient`.

Why the space part matters (F-46).  The previous engine interpolated the same
residual along each net's own copper graph.  Two traces 0.2 mm apart on
different nets then got completely uncorrelated displacements -- one pinned to a
lagging footprint, its neighbour riding near-pure ambient -- and the gap between
them changed by the difference of two unrelated residuals.  That is the whole
observed violation census: every site is a different-net pair, deficits sit in
the rigid-lag range, the site count is flat in k (topology picks the set) while
the depth scales with k.  A field that is a function of position cannot do that:
neighbouring copper moves together whatever nets it belongs to.

Admissibility.  The map is F(x) = x + u(x), so a gap between two nearby points
survives iff the smallest singular value of J = I + grad u is >= 1.  `sigma_min`
returns that map -- the honest, geometry-only answer to "does this field close
anything anywhere", independent of what copper happens to be there.  For one
circular inclusion of radius a in a dilation field the harmonic residual gives
sigma_min = 1 + (k-1)(1 - a^2/r^2) >= 1 exactly, with equality on the inclusion
boundary: the construction is non-contracting by design, and the grid map says
how far real (non-circular, interacting) footprints stray from that.

Linearity.  The solve is linear, so if `ambient` and every inclusion vector are
proportional to a scalar s, then so is u.  Solve once at s = 1 and pass
`scale=s` to evaluate any other amount for free -- that is what drives the
tolerance sweep and the animation.

Reuse: nothing here knows about pcbnew.  `ambient` may be any callable (or None
for a pure "push these apart" field), and inclusion vectors are arbitrary, which
is the hook for the planned steerable "add space here" fields.
"""
import math
import numpy as np
from scipy import sparse, ndimage
from scipy.sparse.linalg import splu
from matplotlib.path import Path


def dilation(cx, cy, kx, ky):
    """Ambient field of a dilation by (kx, ky) about (cx, cy)."""
    def f(X, Y):
        return ((kx - 1.0) * (X - cx), (ky - 1.0) * (Y - cy))
    return f


def sigma_min_2x2(a, b, c, d):
    """Smallest singular value of [[a,b],[c,d]], elementwise over arrays."""
    E = (a + d) / 2.0; F = (a - d) / 2.0
    G = (c + b) / 2.0; H = (c - b) / 2.0
    Q = np.hypot(E, H); R = np.hypot(F, G)
    return np.abs(Q - R)


class WarpField:
    """Harmonic displacement field with rigid inclusions on a regular grid."""

    def __init__(self, bounds, h=0.15, margin=3.0, ambient=None, close=1.0,
                 enclose='hull'):
        """bounds: (x0, y0, x1, y1) region of interest, in mm (the board).
        margin: how far the solve domain extends past it -- the residual decays
        to zero on the outer boundary instead of being clamped at the board
        edge, so edge copper lags smoothly and the outline (pure ambient) pulls
        away from it rather than into it.
        close: morphological closing radius (mm) applied to each inclusion's own
        territory.  A part's pads are rigid, and so in practice is the space
        *enclosed by* them -- a trace threading a connector's pin field is held
        on both sides and cannot do anything but move with the part.  Without
        this the field sags toward ambient between the pins while the pins hold,
        and shears exactly the traces that run through them.
        enclose: how that enclosed space is defined -- 'hull' (convex hull of
        the inclusion's own shapes, scale-free), 'close' (the fixed `close`
        radius), 'both', or 'none'.  A fixed radius cannot serve a board with
        both 0.5 mm-pitch pin fields and connectors whose pads are 12.8 mm
        apart: on this board a 1 mm closing left 43 um of sag inside J9's body,
        which is a shear against its own mounting holes."""
        x0, y0, x1, y1 = bounds
        self.h = float(h)
        self.close = float(close)
        self.enclose = enclose
        self.x0 = x0 - margin; self.y0 = y0 - margin
        self.nx = int(math.ceil((x1 + margin - self.x0) / self.h)) + 1
        self.ny = int(math.ceil((y1 + margin - self.y0) / self.h)) + 1
        self.ambient = ambient
        self.X = self.x0 + self.h * np.arange(self.nx)[None, :]
        self.Y = self.y0 + self.h * np.arange(self.ny)[:, None]
        self._incl = []                                  # (polys, vec, prio, tag)
        self.owner = np.full((self.ny, self.nx), -1, np.int32)
        self.RX = self.RY = None

    # ---- inclusions ---------------------------------------------------------
    def add_inclusion(self, polys, vec, priority=0.0, tag=None):
        """polys: list of point lists [(x,y), ...]; vec: (dx,dy) this territory
        translates by; priority: heavier wins contested cells."""
        self._incl.append((polys, tuple(vec), float(priority), tag))

    def _rasterize(self, grow=1):
        """Mark inclusion territories on the grid, in four rounds, so that no
        inclusion can ever lose its own shapes or their immediate surroundings
        to a neighbour:

          1. every inclusion's raw shapes (its pads),
          2. a protective `grow`-cell halo around those,
          3. the space each inclusion encloses -- convex hull and/or closing,
          4. a halo around that.

        Rounds 1-2 run for ALL inclusions before round 3, which is what lets a
        connector's hull span its own 12.8 mm body without swallowing the cells
        around a small part that happens to sit inside it.  Within a round,
        heavier inclusions go first and nothing already owned is overwritten.
        The halo exists so the four grid corners around any pad interior belong
        to that footprint, making bilinear evaluation there exact."""
        order = sorted(range(len(self._incl)), key=lambda i: -self._incl[i][2])
        use_hull = self.enclose in ('hull', 'both')
        hulls = {idx: (self._hull(self._incl[idx][0]) if use_hull else None) for idx in order}
        # the window must hold the hull too, or a 12.8 mm body gets clipped to
        # the extent of the pads that generated it
        wins = {idx: self._window(self._incl[idx][0], extra=hulls[idx]) for idx in order}
        nc = int(round(self.close / self.h))
        disk = None
        if nc > 0 and self.enclose in ('close', 'both'):
            yy, xx = np.ogrid[-nc:nc + 1, -nc:nc + 1]
            disk = (xx * xx + yy * yy) <= nc * nc

        def claim(idx, m, i0, j0):
            if m is None or not m.any():
                return
            sub = self.owner[j0:j0 + m.shape[0], i0:i0 + m.shape[1]]
            sub[m & (sub < 0)] = idx

        def enclosed(idx):
            """the inclusion's territory including whatever its shapes enclose"""
            m, i0, j0 = wins[idx]
            if m is None or not m.any():
                return None, i0, j0
            full = m.copy()
            if disk is not None:
                full |= ndimage.binary_closing(m, structure=disk)
            if hulls[idx] is not None:
                full |= self._rast_into(hulls[idx], i0, j0, m.shape)
            return full, i0, j0

        for idx in order:                                   # 1. the pads themselves
            m, i0, j0 = wins[idx]
            claim(idx, m, i0, j0)
        for idx in order:                                   # 2. protect their surroundings
            m, i0, j0 = wins[idx]
            if m is None or not m.any(): continue
            claim(idx, ndimage.binary_dilation(m, iterations=grow) & ~m, i0, j0)
        encl = {}
        for idx in order:                                   # 3. what they enclose
            full, i0, j0 = enclosed(idx)
            encl[idx] = (full, i0, j0)
            if full is None: continue
            claim(idx, full & ~wins[idx][0], i0, j0)
        for idx in order:                                   # 4. halo around that
            full, i0, j0 = encl[idx]
            if full is None: continue
            claim(idx, ndimage.binary_dilation(full, iterations=grow) & ~full, i0, j0)
        return sum(1 for idx in wins if wins[idx][0] is not None and wins[idx][0].any())

    def _hull(self, polys):
        """Convex hull of an inclusion's own shapes, as a point list."""
        pts = np.array([p for shape in polys if len(shape) >= 3 for p in shape], float)
        if len(pts) < 3:
            return None
        try:
            from scipy.spatial import ConvexHull
            return [tuple(pts[i]) for i in ConvexHull(pts).vertices]
        except Exception:
            return None                    # collinear / degenerate: closing covers it

    def _rast_into(self, poly, i0, j0, shape):
        """Rasterise one polygon into an existing window at (i0, j0)."""
        gx, gy = np.meshgrid(self.x0 + self.h * (i0 + np.arange(shape[1])),
                             self.y0 + self.h * (j0 + np.arange(shape[0])))
        return Path(np.asarray(poly, float)).contains_points(
            np.column_stack([gx.ravel(), gy.ravel()])).reshape(shape)

    def _window(self, polys, pad=None, extra=None):
        """(mask, i0, j0): the union of `polys` rasterised on a local window.
        `extra` points widen the window without being rasterised into it, so a
        territory's enclosure can be pasted in later without clipping."""
        pts_all = [p for pts in polys if len(pts) >= 3 for p in pts]
        if not pts_all:
            return None, 0, 0
        if pad is None:
            pad = int(round(self.close / self.h)) + 2
        ext = list(pts_all) + (list(extra) if extra else [])
        xs = [p[0] for p in ext]; ys = [p[1] for p in ext]
        i0 = max(0, int((min(xs) - self.x0) / self.h) - pad)
        i1 = min(self.nx - 1, int((max(xs) - self.x0) / self.h) + pad)
        j0 = max(0, int((min(ys) - self.y0) / self.h) - pad)
        j1 = min(self.ny - 1, int((max(ys) - self.y0) / self.h) + pad)
        if i1 < i0 or j1 < j0:
            return None, 0, 0
        gx, gy = np.meshgrid(self.x0 + self.h * np.arange(i0, i1 + 1),
                             self.y0 + self.h * np.arange(j0, j1 + 1))
        pnts = np.column_stack([gx.ravel(), gy.ravel()])
        m = np.zeros(gx.shape, bool)
        for pts in polys:
            if len(pts) < 3:
                continue
            m |= Path(np.asarray(pts, float)).contains_points(pnts).reshape(gx.shape)
        return m, i0, j0

    # ---- solve --------------------------------------------------------------
    def solve(self, verbose=True):
        """Laplace on the residual: r = v_i - ambient(x) inside inclusions,
        r = 0 on the domain boundary, harmonic in between."""
        n_terr = self._rasterize()
        ny, nx = self.ny, self.nx
        AX, AY = self._ambient_grid()
        fixed = self.owner >= 0
        fixed[0, :] = fixed[-1, :] = True
        fixed[:, 0] = fixed[:, -1] = True
        bx = np.zeros((ny, nx)); by = np.zeros((ny, nx))
        for idx, (polys, vec, prio, tag) in enumerate(self._incl):
            sel = self.owner == idx
            if sel.any():
                bx[sel] = vec[0] - AX[sel]
                by[sel] = vec[1] - AY[sel]
        edge = np.zeros((ny, nx), bool)
        edge[0, :] = edge[-1, :] = True; edge[:, 0] = edge[:, -1] = True
        bx[edge & (self.owner < 0)] = 0.0
        by[edge & (self.owner < 0)] = 0.0

        idx_map = np.full((ny, nx), -1, np.int64)
        free = ~fixed
        nf = int(free.sum())
        idx_map[free] = np.arange(nf)
        if verbose:
            print(f'  field grid {nx}x{ny} @ {self.h} mm ({nx*ny} nodes, '
                  f'{nf} free, {n_terr} territories)')

        rows = [np.arange(nf)]; cols = [np.arange(nf)]; vals = [np.full(nf, 4.0)]
        rhsx = np.zeros(nf); rhsy = np.zeros(nf)
        fj, fi = np.nonzero(free)
        for dj, di in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nj, ni = fj + dj, fi + di
            nb_free = free[nj, ni]
            rows.append(idx_map[fj[nb_free], fi[nb_free]])
            cols.append(idx_map[nj[nb_free], ni[nb_free]])
            vals.append(np.full(int(nb_free.sum()), -1.0))
            nb_fix = ~nb_free
            np.add.at(rhsx, idx_map[fj[nb_fix], fi[nb_fix]], bx[nj[nb_fix], ni[nb_fix]])
            np.add.at(rhsy, idx_map[fj[nb_fix], fi[nb_fix]], by[nj[nb_fix], ni[nb_fix]])
        A = sparse.coo_matrix((np.concatenate(vals),
                               (np.concatenate(rows), np.concatenate(cols))),
                              shape=(nf, nf)).tocsc()
        lu = splu(A)
        rx = np.array(bx); ry = np.array(by)
        rx[free] = lu.solve(rhsx); ry[free] = lu.solve(rhsy)
        self.RX, self.RY = rx, ry
        self.fixed = fixed
        return self

    def _ambient_grid(self):
        if self.ambient is None:
            z = np.zeros((self.ny, self.nx))
            return z, z.copy()
        X = np.broadcast_to(self.X, (self.ny, self.nx))
        Y = np.broadcast_to(self.Y, (self.ny, self.nx))
        ax, ay = self.ambient(X, Y)
        return np.asarray(ax, float) * np.ones((self.ny, self.nx)), \
               np.asarray(ay, float) * np.ones((self.ny, self.nx))

    # ---- evaluation ---------------------------------------------------------
    def __call__(self, x, y, scale=1.0):
        """Total displacement (ambient + residual) at (x, y); arrays welcome.
        `scale` is exact only when ambient and every inclusion vector are
        proportional to it (see the module docstring)."""
        xa = np.atleast_1d(np.asarray(x, float))
        ya = np.atleast_1d(np.asarray(y, float))
        fx = (xa - self.x0) / self.h; fy = (ya - self.y0) / self.h
        i0 = np.clip(np.floor(fx).astype(int), 0, self.nx - 2)
        j0 = np.clip(np.floor(fy).astype(int), 0, self.ny - 2)
        tx = np.clip(fx - i0, 0.0, 1.0); ty = np.clip(fy - j0, 0.0, 1.0)
        w00 = (1 - tx) * (1 - ty); w10 = tx * (1 - ty)
        w01 = (1 - tx) * ty;       w11 = tx * ty
        def bil(R):
            return (w00 * R[j0, i0] + w10 * R[j0, i0 + 1] +
                    w01 * R[j0 + 1, i0] + w11 * R[j0 + 1, i0 + 1])
        ux, uy = bil(self.RX), bil(self.RY)
        if self.ambient is not None:
            ax, ay = self.ambient(xa, ya)
            ux = ux + ax; uy = uy + ay
        ux, uy = ux * scale, uy * scale
        if np.isscalar(x) or np.asarray(x).ndim == 0:
            return float(ux[0]), float(uy[0])
        return ux, uy

    # ---- admissibility ------------------------------------------------------
    def sigma_min(self, scale=1.0):
        """Grid map of the smallest singular value of J = I + grad u.
        >= 1 everywhere means the field closes no gap anywhere, at any spacing."""
        AX, AY = self._ambient_grid()
        UX = (self.RX + AX) * scale; UY = (self.RY + AY) * scale
        dUXdy, dUXdx = np.gradient(UX, self.h, self.h)
        dUYdy, dUYdx = np.gradient(UY, self.h, self.h)
        return sigma_min_2x2(1.0 + dUXdx, dUXdy, dUYdx, 1.0 + dUYdy)

    def report(self, scale=1.0, mask=None, tol=1e-4):
        """Contraction census over `mask` (default: everywhere but the border)."""
        s = self.sigma_min(scale)
        if mask is None:
            mask = np.ones_like(s, bool)
            mask[0, :] = mask[-1, :] = mask[:, 0] = mask[:, -1] = False
        bad = mask & (s < 1.0 - tol)
        out = dict(sigma_min=float(s[mask].min()), n_contracting=int(bad.sum()),
                   n_cells=int(mask.sum()), sites=[])
        if bad.any():
            lab, n = ndimage.label(bad)
            for k in range(1, n + 1):
                sel = lab == k
                j, i = np.nonzero(sel)
                out['sites'].append(dict(
                    x=float(self.x0 + self.h * i.mean()),
                    y=float(self.y0 + self.h * j.mean()),
                    sigma_min=float(s[sel].min()), cells=int(sel.sum())))
            out['sites'].sort(key=lambda d: d['sigma_min'])
        return out
