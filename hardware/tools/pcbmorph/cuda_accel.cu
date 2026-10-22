/*
 * CUDA GPU Accelerator for pcbmorph (RTX 3060 / sm_86+).
 *
 * Accelerates:
 *   1. Batched all-pairs candidate edge evaluation (`cuda_eval_edges`) for `build_edges()`
 *      and final slack verification across all 25 edge types (SS, SP, SV, SST, SSV,
 *      SH_P, SH_V, SH_SV, VP, VV, VSV, VST, VH_P, HH_VV, HH_VSV, HH_VP, PP, PST, PSV,
 *      SZ, VZ, PZ, SE, VE, PE).
 *   2. Graph-colored parallel `copper_pass` (`cuda_copper_pass`) with per-node
 *      displacement-Laplacian spring, harmonic warp-field drift, stretch vector,
 *      pressure vector, and exact 8-step feasibility-projected binary line search.
 *   3. Fast trial subset evaluation (`cuda_check_edges_subset`) for `part_pass`,
 *      coupled boundary-layer co-moves (`mv_coupled`), and `shape_pass`.
 */

#include <cuda_runtime.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Edge kind IDs (must match accel.py KIND_MAP)
enum EdgeKind {
    K_SS = 0,
    K_SP = 1,
    K_SV = 2,
    K_SST = 3,
    K_SSV = 4,
    K_SH_P = 5,
    K_SH_V = 6,
    K_SH_SV = 7,
    K_VP = 8,
    K_VV = 9,
    K_VSV = 10,
    K_VST = 11,
    K_VH_P = 12,
    K_HH_VV = 13,
    K_HH_VSV = 14,
    K_HH_VP = 15,
    K_PP = 16,
    K_PST = 17,
    K_PSV = 18,
    K_SZ = 19,
    K_VZ = 20,
    K_PZ = 21,
    K_SE = 22,
    K_VE = 23,
    K_PE = 24
};

// ---------------------------------------------------------------------------
// Device + Host 2D Exact Geometry Kernels (IEEE-754 double precision)
// ---------------------------------------------------------------------------

__host__ __device__ __forceinline__ double d_pt_seg_dist(
    double px, double py, double ax, double ay, double bx, double by) {
    double dx = bx - ax, dy = by - ay;
    double L2 = dx * dx + dy * dy;
    if (L2 < 1e-18) return hypot(px - ax, py - ay);
    double t = ((px - ax) * dx + (py - ay) * dy) / L2;
    if (t < 0.0) t = 0.0;
    else if (t > 1.0) t = 1.0;
    return hypot(px - ax - t * dx, py - ay - t * dy);
}

__host__ __device__ __forceinline__ double d_orient(
    double x1, double y1, double x2, double y2, double x3, double y3) {
    return (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
}

__host__ __device__ __forceinline__ double d_seg_seg_dist(
    double ax, double ay, double bx, double by,
    double cx, double cy, double dx, double dy) {
    double d1 = d_orient(cx, cy, dx, dy, ax, ay);
    double d2 = d_orient(cx, cy, dx, dy, bx, by);
    double d3 = d_orient(ax, ay, bx, by, cx, cy);
    double d4 = d_orient(ax, ay, bx, by, dx, dy);
    if (((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0))) {
        return 0.0;
    }
    double m1 = fmin(d_pt_seg_dist(ax, ay, cx, cy, dx, dy),
                     d_pt_seg_dist(bx, by, cx, cy, dx, dy));
    double m2 = fmin(d_pt_seg_dist(cx, cy, ax, ay, bx, by),
                     d_pt_seg_dist(dx, dy, ax, ay, bx, by));
    return fmin(m1, m2);
}

__host__ __device__ __forceinline__ bool d_point_in_poly_shifted(
    double x, double y, const double* pts, int n, double sx, double sy) {
    bool inside = false;
    int j = n - 1;
    for (int i = 0; i < n; ++i) {
        double xi = pts[2 * i] + sx, yi = pts[2 * i + 1] + sy;
        double xj = pts[2 * j] + sx, yj = pts[2 * j + 1] + sy;
        if (((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)) {
            inside = !inside;
        }
        j = i;
    }
    return inside;
}

__host__ __device__ __forceinline__ double d_poly_pt_dist_shifted(
    const double* pts, int n, double sx, double sy, double x, double y) {
    double d = 1e30;
    for (int i = 0; i < n; ++i) {
        int j = (i + 1 == n) ? 0 : i + 1;
        double dist = d_pt_seg_dist(
            x, y,
            pts[2 * i] + sx, pts[2 * i + 1] + sy,
            pts[2 * j] + sx, pts[2 * j + 1] + sy);
        if (dist < d) d = dist;
    }
    if (d > 0.0 && d_point_in_poly_shifted(x, y, pts, n, sx, sy)) {
        return 0.0;
    }
    return d;
}

__host__ __device__ __forceinline__ double d_poly_seg_dist_shifted(
    const double* pts, int n, double sx, double sy,
    double x1, double y1, double x2, double y2) {
    double d = 1e30;
    for (int i = 0; i < n; ++i) {
        int j = (i + 1 == n) ? 0 : i + 1;
        double dist = d_seg_seg_dist(
            x1, y1, x2, y2,
            pts[2 * i] + sx, pts[2 * i + 1] + sy,
            pts[2 * j] + sx, pts[2 * j + 1] + sy);
        if (dist < d) d = dist;
    }
    if (d > 0.0 && d_point_in_poly_shifted(0.5 * (x1 + x2), 0.5 * (y1 + y2), pts, n, sx, sy)) {
        return 0.0;
    }
    return d;
}

__host__ __device__ __forceinline__ double d_poly_poly_dist_shifted(
    const double* A, int na, double sax, double say,
    const double* B, int nb, double sbx, double sby) {
    double d = 1e30;
    for (int i = 0; i < na; ++i) {
        int i2 = (i + 1 == na) ? 0 : i + 1;
        double ax1 = A[2 * i] + sax, ay1 = A[2 * i + 1] + say;
        double ax2 = A[2 * i2] + sax, ay2 = A[2 * i2 + 1] + say;
        for (int j = 0; j < nb; ++j) {
            int j2 = (j + 1 == nb) ? 0 : j + 1;
            double bx1 = B[2 * j] + sbx, by1 = B[2 * j + 1] + sby;
            double bx2 = B[2 * j2] + sbx, by2 = B[2 * j2 + 1] + sby;
            double dist = d_seg_seg_dist(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2);
            if (dist < d) d = dist;
        }
    }
    if (d > 0.0) {
        if (d_point_in_poly_shifted(A[0] + sax, A[1] + say, B, nb, sbx, sby) ||
            d_point_in_poly_shifted(B[0] + sbx, B[1] + sby, A, na, sax, say)) {
            return 0.0;
        }
    }
    return d;
}

__host__ __device__ __forceinline__ double d_inside_dist(
    double x, double y, const double* pts, int n) {
    double d = 1e30;
    for (int i = 0; i < n; ++i) {
        int j = (i + 1 == n) ? 0 : i + 1;
        double dist = d_pt_seg_dist(x, y, pts[2 * i], pts[2 * i + 1], pts[2 * j], pts[2 * j + 1]);
        if (dist < d) d = dist;
    }
    return d_point_in_poly_shifted(x, y, pts, n, 0.0, 0.0) ? d : -d;
}

__host__ __device__ __forceinline__ double d_inside_seg_dist(
    double x1, double y1, double x2, double y2, const double* pts, int n) {
    double mx = 0.5 * (x1 + x2), my = 0.5 * (y1 + y2);
    if (!(d_point_in_poly_shifted(x1, y1, pts, n, 0.0, 0.0) &&
          d_point_in_poly_shifted(x2, y2, pts, n, 0.0, 0.0) &&
          d_point_in_poly_shifted(mx, my, pts, n, 0.0, 0.0))) {
        double d1 = d_inside_dist(x1, y1, pts, n);
        double d2 = d_inside_dist(x2, y2, pts, n);
        double dm = d_inside_dist(mx, my, pts, n);
        return fmin(d1, fmin(d2, dm));
    }
    double d = 1e30;
    for (int i = 0; i < n; ++i) {
        int j = (i + 1 == n) ? 0 : i + 1;
        double dist = d_seg_seg_dist(x1, y1, x2, y2, pts[2 * i], pts[2 * i + 1], pts[2 * j], pts[2 * j + 1]);
        if (dist < d) d = dist;
    }
    return d;
}

__host__ __device__ __forceinline__ double d_inside_poly_dist_shifted(
    const double* poly, int np, double sx, double sy, const double* pts, int n) {
    for (int i = 0; i < np; ++i) {
        if (!d_point_in_poly_shifted(poly[2 * i] + sx, poly[2 * i + 1] + sy, pts, n, 0.0, 0.0)) {
            double m = 1e30;
            for (int k = 0; k < np; ++k) {
                double dk = d_inside_dist(poly[2 * k] + sx, poly[2 * k + 1] + sy, pts, n);
                if (dk < m) m = dk;
            }
            return m;
        }
    }
    double d = 1e30;
    for (int i = 0; i < np; ++i) {
        int i2 = (i + 1 == np) ? 0 : i + 1;
        double ax1 = poly[2 * i] + sx, ay1 = poly[2 * i + 1] + sy;
        double ax2 = poly[2 * i2] + sx, ay2 = poly[2 * i2 + 1] + sy;
        for (int j = 0; j < n; ++j) {
            int j2 = (j + 1 == n) ? 0 : j + 1;
            double dist = d_seg_seg_dist(ax1, ay1, ax2, ay2, pts[2 * j], pts[2 * j + 1], pts[2 * j2], pts[2 * j2 + 1]);
            if (dist < d) d = dist;
        }
    }
    return d;
}

// ---------------------------------------------------------------------------
// Unified Device Edge Evaluator (supports optional trial override for 1 node)
// ---------------------------------------------------------------------------

struct BoardDevView {
    const double* P;             // [N, 2]
    const int32_t* seg_ab;       // [S, 2] (a, b)
    const double* seg_hw;        // [S] half-width
    const double* via_r;         // [N, 2] (vr, dr)
    const double* pad_xy0;       // [NP, 2]
    const double* pad_dr;        // [NP] half-drill
    const double* pad_shift;     // [NP, 2] current D[ref] shift for each pad
    const double* pad_verts0;    // [NP_V, 2]
    const int32_t* pad_offsets;  // [NP + 1]
    const double* st_tracks;     // [ST, 5] (x1, y1, x2, y2, hr)
    const double* st_vias;       // [SV, 4] (vx, vy, vr, dr)
    const double* zone_verts;    // [NZ_V, 2]
    const int32_t* zone_offsets; // [NZ + 1]
    const double* outline;       // [NO, 2]
    int32_t n_outline;
};

__host__ __device__ __forceinline__ void get_node_xy(
    const BoardDevView& v, int nid, int override_nid, double ox, double oy,
    double& x, double& y) {
    if (nid == override_nid) {
        x = ox;
        y = oy;
    } else {
        x = v.P[2 * nid];
        y = v.P[2 * nid + 1];
    }
}

__host__ __device__ double d_eval_edge_one(
    const BoardDevView& v, int kind, int a, int b,
    int override_nid, double ox, double oy) {
    switch (kind) {
        case K_SS: {
            int s1a = v.seg_ab[2 * a], s1b = v.seg_ab[2 * a + 1];
            int s2a = v.seg_ab[2 * b], s2b = v.seg_ab[2 * b + 1];
            double p1x, p1y, p2x, p2y, q1x, q1y, q2x, q2y;
            get_node_xy(v, s1a, override_nid, ox, oy, p1x, p1y);
            get_node_xy(v, s1b, override_nid, ox, oy, p2x, p2y);
            get_node_xy(v, s2a, override_nid, ox, oy, q1x, q1y);
            get_node_xy(v, s2b, override_nid, ox, oy, q2x, q2y);
            return d_seg_seg_dist(p1x, p1y, p2x, p2y, q1x, q1y, q2x, q2y) - (v.seg_hw[a] + v.seg_hw[b]);
        }
        case K_SP: {
            int sa = v.seg_ab[2 * a], sb = v.seg_ab[2 * a + 1];
            double p1x, p1y, p2x, p2y;
            get_node_xy(v, sa, override_nid, ox, oy, p1x, p1y);
            get_node_xy(v, sb, override_nid, ox, oy, p2x, p2y);
            int off = v.pad_offsets[b], npts = v.pad_offsets[b + 1] - off;
            double sx = v.pad_shift[2 * b], sy = v.pad_shift[2 * b + 1];
            return d_poly_seg_dist_shifted(v.pad_verts0 + 2 * off, npts, sx, sy, p1x, p1y, p2x, p2y) - v.seg_hw[a];
        }
        case K_SV: {
            int sa = v.seg_ab[2 * a], sb = v.seg_ab[2 * a + 1];
            double p1x, p1y, p2x, p2y, vx, vy;
            get_node_xy(v, sa, override_nid, ox, oy, p1x, p1y);
            get_node_xy(v, sb, override_nid, ox, oy, p2x, p2y);
            get_node_xy(v, b, override_nid, ox, oy, vx, vy);
            return d_pt_seg_dist(vx, vy, p1x, p1y, p2x, p2y) - v.seg_hw[a] - v.via_r[2 * b];
        }
        case K_SST: {
            int sa = v.seg_ab[2 * a], sb = v.seg_ab[2 * a + 1];
            double p1x, p1y, p2x, p2y;
            get_node_xy(v, sa, override_nid, ox, oy, p1x, p1y);
            get_node_xy(v, sb, override_nid, ox, oy, p2x, p2y);
            const double* st = v.st_tracks + 5 * b;
            return d_seg_seg_dist(p1x, p1y, p2x, p2y, st[0], st[1], st[2], st[3]) - v.seg_hw[a] - st[4];
        }
        case K_SSV: {
            int sa = v.seg_ab[2 * a], sb = v.seg_ab[2 * a + 1];
            double p1x, p1y, p2x, p2y;
            get_node_xy(v, sa, override_nid, ox, oy, p1x, p1y);
            get_node_xy(v, sb, override_nid, ox, oy, p2x, p2y);
            const double* sv = v.st_vias + 4 * b;
            return d_pt_seg_dist(sv[0], sv[1], p1x, p1y, p2x, p2y) - v.seg_hw[a] - sv[2];
        }
        case K_SH_P: {
            int sa = v.seg_ab[2 * a], sb = v.seg_ab[2 * a + 1];
            double p1x, p1y, p2x, p2y;
            get_node_xy(v, sa, override_nid, ox, oy, p1x, p1y);
            get_node_xy(v, sb, override_nid, ox, oy, p2x, p2y);
            double px = v.pad_xy0[2 * b] + v.pad_shift[2 * b];
            double py = v.pad_xy0[2 * b + 1] + v.pad_shift[2 * b + 1];
            return d_pt_seg_dist(px, py, p1x, p1y, p2x, p2y) - v.seg_hw[a] - v.pad_dr[b];
        }
        case K_SH_V: {
            int sa = v.seg_ab[2 * a], sb = v.seg_ab[2 * a + 1];
            double p1x, p1y, p2x, p2y, vx, vy;
            get_node_xy(v, sa, override_nid, ox, oy, p1x, p1y);
            get_node_xy(v, sb, override_nid, ox, oy, p2x, p2y);
            get_node_xy(v, b, override_nid, ox, oy, vx, vy);
            return d_pt_seg_dist(vx, vy, p1x, p1y, p2x, p2y) - v.seg_hw[a] - v.via_r[2 * b + 1];
        }
        case K_SH_SV: {
            int sa = v.seg_ab[2 * a], sb = v.seg_ab[2 * a + 1];
            double p1x, p1y, p2x, p2y;
            get_node_xy(v, sa, override_nid, ox, oy, p1x, p1y);
            get_node_xy(v, sb, override_nid, ox, oy, p2x, p2y);
            const double* sv = v.st_vias + 4 * b;
            return d_pt_seg_dist(sv[0], sv[1], p1x, p1y, p2x, p2y) - v.seg_hw[a] - sv[3];
        }
        case K_VP: {
            double vx, vy;
            get_node_xy(v, a, override_nid, ox, oy, vx, vy);
            int off = v.pad_offsets[b], npts = v.pad_offsets[b + 1] - off;
            double sx = v.pad_shift[2 * b], sy = v.pad_shift[2 * b + 1];
            return d_poly_pt_dist_shifted(v.pad_verts0 + 2 * off, npts, sx, sy, vx, vy) - v.via_r[2 * a];
        }
        case K_VV: {
            double v1x, v1y, v2x, v2y;
            get_node_xy(v, a, override_nid, ox, oy, v1x, v1y);
            get_node_xy(v, b, override_nid, ox, oy, v2x, v2y);
            return hypot(v1x - v2x, v1y - v2y) - v.via_r[2 * a] - v.via_r[2 * b];
        }
        case K_VSV: {
            double v1x, v1y;
            get_node_xy(v, a, override_nid, ox, oy, v1x, v1y);
            const double* sv = v.st_vias + 4 * b;
            return hypot(v1x - sv[0], v1y - sv[1]) - v.via_r[2 * a] - sv[2];
        }
        case K_VST: {
            double v1x, v1y;
            get_node_xy(v, a, override_nid, ox, oy, v1x, v1y);
            const double* st = v.st_tracks + 5 * b;
            return d_pt_seg_dist(v1x, v1y, st[0], st[1], st[2], st[3]) - v.via_r[2 * a] - st[4];
        }
        case K_VH_P: {
            double v1x, v1y;
            get_node_xy(v, a, override_nid, ox, oy, v1x, v1y);
            double px = v.pad_xy0[2 * b] + v.pad_shift[2 * b];
            double py = v.pad_xy0[2 * b + 1] + v.pad_shift[2 * b + 1];
            return hypot(v1x - px, v1y - py) - v.via_r[2 * a] - v.pad_dr[b];
        }
        case K_HH_VV: {
            double v1x, v1y, v2x, v2y;
            get_node_xy(v, a, override_nid, ox, oy, v1x, v1y);
            get_node_xy(v, b, override_nid, ox, oy, v2x, v2y);
            return hypot(v1x - v2x, v1y - v2y) - v.via_r[2 * a + 1] - v.via_r[2 * b + 1];
        }
        case K_HH_VSV: {
            double v1x, v1y;
            get_node_xy(v, a, override_nid, ox, oy, v1x, v1y);
            const double* sv = v.st_vias + 4 * b;
            return hypot(v1x - sv[0], v1y - sv[1]) - v.via_r[2 * a + 1] - sv[3];
        }
        case K_HH_VP: {
            double v1x, v1y;
            get_node_xy(v, a, override_nid, ox, oy, v1x, v1y);
            double px = v.pad_xy0[2 * b] + v.pad_shift[2 * b];
            double py = v.pad_xy0[2 * b + 1] + v.pad_shift[2 * b + 1];
            return hypot(v1x - px, v1y - py) - v.via_r[2 * a + 1] - v.pad_dr[b];
        }
        case K_PP: {
            int off_a = v.pad_offsets[a], na = v.pad_offsets[a + 1] - off_a;
            int off_b = v.pad_offsets[b], nb = v.pad_offsets[b + 1] - off_b;
            return d_poly_poly_dist_shifted(
                v.pad_verts0 + 2 * off_a, na, v.pad_shift[2 * a], v.pad_shift[2 * a + 1],
                v.pad_verts0 + 2 * off_b, nb, v.pad_shift[2 * b], v.pad_shift[2 * b + 1]);
        }
        case K_PST: {
            int off_a = v.pad_offsets[a], na = v.pad_offsets[a + 1] - off_a;
            const double* st = v.st_tracks + 5 * b;
            return d_poly_seg_dist_shifted(
                v.pad_verts0 + 2 * off_a, na, v.pad_shift[2 * a], v.pad_shift[2 * a + 1],
                st[0], st[1], st[2], st[3]) - st[4];
        }
        case K_PSV: {
            int off_a = v.pad_offsets[a], na = v.pad_offsets[a + 1] - off_a;
            const double* sv = v.st_vias + 4 * b;
            return d_poly_pt_dist_shifted(
                v.pad_verts0 + 2 * off_a, na, v.pad_shift[2 * a], v.pad_shift[2 * a + 1],
                sv[0], sv[1]) - sv[2];
        }
        case K_SZ: {
            int sa = v.seg_ab[2 * a], sb = v.seg_ab[2 * a + 1];
            double p1x, p1y, p2x, p2y;
            get_node_xy(v, sa, override_nid, ox, oy, p1x, p1y);
            get_node_xy(v, sb, override_nid, ox, oy, p2x, p2y);
            int off_z = v.zone_offsets[b], nz = v.zone_offsets[b + 1] - off_z;
            return d_poly_seg_dist_shifted(v.zone_verts + 2 * off_z, nz, 0.0, 0.0, p1x, p1y, p2x, p2y) - v.seg_hw[a];
        }
        case K_VZ: {
            double vx, vy;
            get_node_xy(v, a, override_nid, ox, oy, vx, vy);
            int off_z = v.zone_offsets[b], nz = v.zone_offsets[b + 1] - off_z;
            return d_poly_pt_dist_shifted(v.zone_verts + 2 * off_z, nz, 0.0, 0.0, vx, vy) - v.via_r[2 * a];
        }
        case K_PZ: {
            int off_a = v.pad_offsets[a], na = v.pad_offsets[a + 1] - off_a;
            int off_z = v.zone_offsets[b], nz = v.zone_offsets[b + 1] - off_z;
            return d_poly_poly_dist_shifted(
                v.pad_verts0 + 2 * off_a, na, v.pad_shift[2 * a], v.pad_shift[2 * a + 1],
                v.zone_verts + 2 * off_z, nz, 0.0, 0.0);
        }
        case K_SE: {
            int sa = v.seg_ab[2 * a], sb = v.seg_ab[2 * a + 1];
            double p1x, p1y, p2x, p2y;
            get_node_xy(v, sa, override_nid, ox, oy, p1x, p1y);
            get_node_xy(v, sb, override_nid, ox, oy, p2x, p2y);
            return d_inside_seg_dist(p1x, p1y, p2x, p2y, v.outline, v.n_outline) - v.seg_hw[a];
        }
        case K_VE: {
            double vx, vy;
            get_node_xy(v, a, override_nid, ox, oy, vx, vy);
            return d_inside_dist(vx, vy, v.outline, v.n_outline) - v.via_r[2 * a];
        }
        case K_PE: {
            int off_a = v.pad_offsets[a], na = v.pad_offsets[a + 1] - off_a;
            return d_inside_poly_dist_shifted(
                v.pad_verts0 + 2 * off_a, na, v.pad_shift[2 * a], v.pad_shift[2 * a + 1],
                v.outline, v.n_outline);
        }
        default:
            return 99.0;
    }
}

// ---------------------------------------------------------------------------
// Kernel 1: Batched All-Pairs / Active-Set Edge Evaluator
// ---------------------------------------------------------------------------

__global__ void kernel_eval_edges(
    BoardDevView v,
    const int32_t* __restrict__ edge_kab, // [E, 3] (kind, a, b)
    double* __restrict__ out_d,           // [E]
    int32_t n_edges) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= n_edges) return;
    int kind = edge_kab[3 * idx];
    int a = edge_kab[3 * idx + 1];
    int b = edge_kab[3 * idx + 2];
    out_d[idx] = d_eval_edge_one(v, kind, a, b, -1, 0.0, 0.0);
}

// ---------------------------------------------------------------------------
// Kernel 2: Graph-Colored Parallel `copper_pass` with 8-Step Binary Search
// ---------------------------------------------------------------------------

__global__ void kernel_copper_color_pass(
    BoardDevView v,
    double* __restrict__ P_mut,             // [N, 2] mutable node positions
    const double* __restrict__ P0,          // [N, 2] initial node positions
    const double* __restrict__ press,       // [N, 2] pressure vectors
    const double* __restrict__ node_u,      // [N, 2] harmonic field displacements
    const int32_t* __restrict__ color_nodes,// [n_color_nodes] node IDs in this color
    int32_t n_color_nodes,
    const int32_t* __restrict__ adj_indptr, // [N + 1]
    const int32_t* __restrict__ adj_indices,// [nnz_adj]
    const int32_t* __restrict__ ne_indptr,  // [N + 1]
    const int32_t* __restrict__ ne_indices, // [nnz_ne]
    const int32_t* __restrict__ edge_kab,   // [E, 3]
    const double* __restrict__ edge_d0,     // [E]
    const double* __restrict__ edge_req,    // [E] rule + GUARD
    const int32_t* __restrict__ at_indptr,  // [N + 1]
    const int32_t* __restrict__ at_pad,     // [n_at]
    const double* __restrict__ at_cap,      // [n_at]
    const int32_t* __restrict__ az_indptr,  // [N + 1]
    const int32_t* __restrict__ az_zone,    // [n_az]
    const double* __restrict__ node_len_slack, // [N] remaining net length slack for node's net
    double sub_mm,
    double step,
    double k_sp,
    double w_pr,
    double w_field,
    double dz_mm,
    int32_t has_field) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= n_color_nodes) return;

    int nid = color_nodes[tid];
    double x0 = P_mut[2 * nid];
    double y0 = P_mut[2 * nid + 1];
    double x_init = P0[2 * nid];
    double y_init = P0[2 * nid + 1];

    // 1. Displacement-Laplacian spring + harmonic anchor + stretch vector
    int a_beg = adj_indptr[nid], a_end = adj_indptr[nid + 1];
    int deg = a_end - a_beg;
    double sx = 0.0, sy = 0.0;
    double tx = 0.0, ty = 0.0;
    if (deg > 0) {
        double du_x = x0 - x_init;
        double du_y = y0 - y_init;
        double sum_dux = 0.0, sum_duy = 0.0;
        for (int k = a_beg; k < a_end; ++k) {
            int u = adj_indices[k];
            double ux = P_mut[2 * u], uy = P_mut[2 * u + 1];
            sum_dux += (ux - P0[2 * u]);
            sum_duy += (uy - P0[2 * u + 1]);
            double dx = ux - x0, dy = uy - y0;
            double dist = hypot(dx, dy);
            if (dist > sub_mm * 1.6) {
                double ex = (dist - sub_mm) / dist;
                tx += dx * ex * 0.5;
                ty += dy * ex * 0.5;
            }
        }
        double lap_x = sum_dux / deg - du_x;
        double lap_y = sum_duy / deg - du_y;
        double ax, ay;
        if (has_field) {
            double hx = x_init + node_u[2 * nid];
            double hy = y_init + node_u[2 * nid + 1];
            ax = (hx - x0) * 0.12;
            ay = (hy - y0) * 0.12;
        } else {
            ax = -du_x * 0.08;
            ay = -du_y * 0.08;
        }
        sx = lap_x * 0.45 + ax;
        sy = lap_y * 0.45 + ay;
    }

    // 2. Pressure vector
    double px = press[2 * nid], py = press[2 * nid + 1];
    double pn = hypot(px, py);
    if (pn > 1e-6) {
        double scale = fmin(step, pn * w_pr) / pn;
        px *= scale;
        py *= scale;
    }

    // 3. Harmonic field drift
    double fx = 0.0, fy = 0.0;
    if (has_field) {
        double ux = node_u[2 * nid], uy = node_u[2 * nid + 1];
        if (hypot(ux, uy) > dz_mm) {
            fx = (x_init + ux - x0) * w_field;
            fy = (y_init + uy - y0) * w_field;
        }
    }

    double vx = px + tx + fx + sx * k_sp;
    double vy = py + ty + fy + sy * k_sp;
    double vm = hypot(vx, vy);
    if (vm < 1e-5) return;
    if (vm > step) {
        vx = vx / vm * step;
        vy = vy / vm * step;
    }

    // Helper lambda to check feasibility at trial position (nx, ny)
    auto trial_ok = [&](double nx, double ny) -> bool {
        // Check pad attachments (AT)
        for (int k = at_indptr[nid]; k < at_indptr[nid + 1]; ++k) {
            int pi = at_pad[k];
            int off = v.pad_offsets[pi], npts = v.pad_offsets[pi + 1] - off;
            double d = d_poly_pt_dist_shifted(
                v.pad_verts0 + 2 * off, npts,
                v.pad_shift[2 * pi], v.pad_shift[2 * pi + 1], nx, ny);
            if (d > at_cap[k]) return false;
        }
        // Check zone attachments (AZ)
        for (int k = az_indptr[nid]; k < az_indptr[nid + 1]; ++k) {
            int zi = az_zone[k];
            int off = v.zone_offsets[zi], npts = v.zone_offsets[zi + 1] - off;
            if (!d_point_in_poly_shifted(nx, ny, v.zone_verts + 2 * off, npts, 0.0, 0.0)) {
                return false;
            }
        }
        // Check local net length delta against net's remaining slack
        double dlen = 0.0;
        for (int k = a_beg; k < a_end; ++k) {
            int u = adj_indices[k];
            double ux = P_mut[2 * u], uy = P_mut[2 * u + 1];
            dlen += hypot(nx - ux, ny - uy) - hypot(x0 - ux, y0 - uy);
        }
        if (dlen > node_len_slack[nid]) return false;

        // Check incident DRC edges
        for (int k = ne_indptr[nid]; k < ne_indptr[nid + 1]; ++k) {
            int ei = ne_indices[k];
            int kind = edge_kab[3 * ei];
            int ea = edge_kab[3 * ei + 1];
            int eb = edge_kab[3 * ei + 2];
            double d = d_eval_edge_one(v, kind, ea, eb, nid, nx, ny);
            if (!(d >= edge_d0[ei] || d >= edge_req[ei])) {
                return false;
            }
        }
        return true;
    };

    // 4. Exact 8-step binary line search
    double best_a = 0.0;
    if (trial_ok(x0 + vx, y0 + vy)) {
        best_a = 1.0;
    } else {
        double lo = 0.0, hi = 1.0;
        for (int iter = 0; iter < 8; ++iter) {
            double mid = 0.5 * (lo + hi);
            if (trial_ok(x0 + vx * mid, y0 + vy * mid)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        best_a = lo;
    }

    if (best_a > 0.0) {
        P_mut[2 * nid] = x0 + vx * best_a;
        P_mut[2 * nid + 1] = y0 + vy * best_a;
    }
}

// ---------------------------------------------------------------------------
// Persistent GPU Context & C-ABI Entrypoints for Python `ctypes`
// ---------------------------------------------------------------------------

struct GpuContext {
    double* d_P = nullptr;
    double* d_P0 = nullptr;
    double* d_press = nullptr;
    double* d_node_u = nullptr;
    double* d_node_len_slack = nullptr;
    int32_t* d_seg_ab = nullptr;
    double* d_seg_hw = nullptr;
    double* d_via_r = nullptr;
    double* d_pad_xy0 = nullptr;
    double* d_pad_dr = nullptr;
    double* d_pad_shift = nullptr;
    double* d_pad_verts0 = nullptr;
    int32_t* d_pad_offsets = nullptr;
    double* d_st_tracks = nullptr;
    double* d_st_vias = nullptr;
    double* d_zone_verts = nullptr;
    int32_t* d_zone_offsets = nullptr;
    double* d_outline = nullptr;

    int32_t* d_adj_indptr = nullptr;
    int32_t* d_adj_indices = nullptr;
    int32_t* d_ne_indptr = nullptr;
    int32_t* d_ne_indices = nullptr;
    int32_t* d_cand_kab = nullptr;
    int32_t* d_edge_kab = nullptr;
    double* d_edge_d0 = nullptr;
    double* d_edge_req = nullptr;
    double* d_edge_out = nullptr;

    int32_t* d_at_indptr = nullptr;
    int32_t* d_at_pad = nullptr;
    double* d_at_cap = nullptr;
    int32_t* d_az_indptr = nullptr;
    int32_t* d_az_zone = nullptr;
    int32_t* d_color_nodes = nullptr;

    int32_t cap_nodes = 0;
    int32_t cap_segs = 0;
    int32_t cap_cand = 0;
    int32_t cap_out = 0;
    int32_t cap_edges = 0;
    int32_t cap_ne = 0;
    int32_t cap_adj = 0;

    int32_t n_nodes = 0;
    int32_t n_segs = 0;
    int32_t n_pads = 0;
    int32_t n_outline = 0;
    int32_t n_edges = 0;
};

static GpuContext g_ctx;

template <typename T>
static void ensure_buf(T** ptr, int32_t* cur_cap, int32_t req, int32_t mult = 1) {
    if (req <= *cur_cap && *ptr != nullptr) return;
    if (*ptr != nullptr) cudaFree(*ptr);
    int32_t new_cap = (req < 1024) ? 1024 : (req * 3) / 2;
    cudaMalloc((void**)ptr, sizeof(T) * (size_t)new_cap * (size_t)mult);
    *cur_cap = new_cap;
}

template <typename T>
static void alloc_and_copy(T** ptr, const T* h_src, size_t count) {
    if (*ptr != nullptr) cudaFree(*ptr);
    if (count == 0) {
        *ptr = nullptr;
        return;
    }
    cudaMalloc((void**)ptr, sizeof(T) * count);
    cudaMemcpy(*ptr, h_src, sizeof(T) * count, cudaMemcpyHostToDevice);
}

extern "C" {

int pcbmorph_cuda_available() {
    int count = 0;
    cudaError_t err = cudaGetDeviceCount(&count);
    return (err == cudaSuccess && count > 0) ? 1 : 0;
}

void pcbmorph_cuda_init_static(
    int32_t n_pads,
    const double* h_pad_xy0,
    const double* h_pad_dr,
    int32_t n_pad_verts,
    const double* h_pad_verts0,
    const int32_t* h_pad_offsets,
    int32_t n_st_tracks,
    const double* h_st_tracks,
    int32_t n_st_vias,
    const double* h_st_vias,
    int32_t n_zones,
    int32_t n_zone_verts,
    const double* h_zone_verts,
    const int32_t* h_zone_offsets,
    int32_t n_outline,
    const double* h_outline) {

    g_ctx.n_pads = n_pads;
    g_ctx.n_outline = n_outline;

    alloc_and_copy(&g_ctx.d_pad_xy0, h_pad_xy0, (size_t)n_pads * 2);
    alloc_and_copy(&g_ctx.d_pad_dr, h_pad_dr, (size_t)n_pads);
    alloc_and_copy(&g_ctx.d_pad_shift, h_pad_xy0, (size_t)n_pads * 2);
    cudaMemset(g_ctx.d_pad_shift, 0, sizeof(double) * (size_t)n_pads * 2);
    alloc_and_copy(&g_ctx.d_pad_verts0, h_pad_verts0, (size_t)n_pad_verts * 2);
    alloc_and_copy(&g_ctx.d_pad_offsets, h_pad_offsets, (size_t)(n_pads + 1));

    alloc_and_copy(&g_ctx.d_st_tracks, h_st_tracks, (size_t)n_st_tracks * 5);
    alloc_and_copy(&g_ctx.d_st_vias, h_st_vias, (size_t)n_st_vias * 4);
    alloc_and_copy(&g_ctx.d_zone_verts, h_zone_verts, (size_t)n_zone_verts * 2);
    alloc_and_copy(&g_ctx.d_zone_offsets, h_zone_offsets, (size_t)(n_zones + 1));
    alloc_and_copy(&g_ctx.d_outline, h_outline, (size_t)n_outline * 2);
}

void pcbmorph_cuda_update_graph(
    int32_t n_nodes,
    const double* h_P,
    const double* h_P0,
    const double* h_via_r,
    int32_t n_segs,
    const int32_t* h_seg_ab,
    const double* h_seg_hw,
    const int32_t* h_adj_indptr,
    int32_t nnz_adj,
    const int32_t* h_adj_indices,
    const int32_t* h_at_indptr,
    int32_t n_at,
    const int32_t* h_at_pad,
    const double* h_at_cap,
    const int32_t* h_az_indptr,
    int32_t n_az,
    const int32_t* h_az_zone) {

    g_ctx.n_nodes = n_nodes;
    g_ctx.n_segs = n_segs;

    alloc_and_copy(&g_ctx.d_P, h_P, (size_t)n_nodes * 2);
    alloc_and_copy(&g_ctx.d_P0, h_P0, (size_t)n_nodes * 2);
    alloc_and_copy(&g_ctx.d_via_r, h_via_r, (size_t)n_nodes * 2);
    alloc_and_copy(&g_ctx.d_press, h_P, (size_t)n_nodes * 2);
    alloc_and_copy(&g_ctx.d_node_u, h_P, (size_t)n_nodes * 2);
    alloc_and_copy(&g_ctx.d_node_len_slack, h_P, (size_t)n_nodes);

    alloc_and_copy(&g_ctx.d_seg_ab, h_seg_ab, (size_t)n_segs * 2);
    alloc_and_copy(&g_ctx.d_seg_hw, h_seg_hw, (size_t)n_segs);

    alloc_and_copy(&g_ctx.d_adj_indptr, h_adj_indptr, (size_t)(n_nodes + 1));
    alloc_and_copy(&g_ctx.d_adj_indices, h_adj_indices, (size_t)nnz_adj);

    alloc_and_copy(&g_ctx.d_at_indptr, h_at_indptr, (size_t)(n_nodes + 1));
    alloc_and_copy(&g_ctx.d_at_pad, h_at_pad, (size_t)n_at);
    alloc_and_copy(&g_ctx.d_at_cap, h_at_cap, (size_t)n_at);

    alloc_and_copy(&g_ctx.d_az_indptr, h_az_indptr, (size_t)(n_nodes + 1));
    alloc_and_copy(&g_ctx.d_az_zone, h_az_zone, (size_t)n_az);
}

void pcbmorph_cuda_eval_candidates(
    const double* h_P,
    const double* h_pad_shift,
    const double* h_outline,
    int32_t n_cand,
    const int32_t* h_cand_kab,
    double* h_out_d) {

    if (n_cand <= 0) return;
    cudaMemcpy(g_ctx.d_P, h_P, sizeof(double) * (size_t)g_ctx.n_nodes * 2, cudaMemcpyHostToDevice);
    if (g_ctx.n_pads > 0 && h_pad_shift != nullptr) {
        cudaMemcpy(g_ctx.d_pad_shift, h_pad_shift, sizeof(double) * (size_t)g_ctx.n_pads * 2, cudaMemcpyHostToDevice);
    }
    if (g_ctx.n_outline > 0 && h_outline != nullptr) {
        cudaMemcpy(g_ctx.d_outline, h_outline, sizeof(double) * (size_t)g_ctx.n_outline * 2, cudaMemcpyHostToDevice);
    }

    ensure_buf(&g_ctx.d_cand_kab, &g_ctx.cap_cand, n_cand, 3);
    ensure_buf(&g_ctx.d_edge_out, &g_ctx.cap_out, n_cand, 1);

    cudaMemcpy(g_ctx.d_cand_kab, h_cand_kab, sizeof(int32_t) * (size_t)n_cand * 3, cudaMemcpyHostToDevice);

    BoardDevView view = {
        g_ctx.d_P, g_ctx.d_seg_ab, g_ctx.d_seg_hw, g_ctx.d_via_r,
        g_ctx.d_pad_xy0, g_ctx.d_pad_dr, g_ctx.d_pad_shift,
        g_ctx.d_pad_verts0, g_ctx.d_pad_offsets,
        g_ctx.d_st_tracks, g_ctx.d_st_vias,
        g_ctx.d_zone_verts, g_ctx.d_zone_offsets,
        g_ctx.d_outline, g_ctx.n_outline
    };

    int threads = 256;
    int blocks = (n_cand + threads - 1) / threads;
    kernel_eval_edges<<<blocks, threads>>>(view, g_ctx.d_cand_kab, g_ctx.d_edge_out, n_cand);
    cudaMemcpy(h_out_d, g_ctx.d_edge_out, sizeof(double) * (size_t)n_cand, cudaMemcpyDeviceToHost);
}

void pcbmorph_cuda_upload_edges(
    int32_t n_edges,
    const int32_t* h_edge_kab,
    const double* h_edge_d0,
    const double* h_edge_req,
    const int32_t* h_ne_indptr,
    int32_t nnz_ne,
    const int32_t* h_ne_indices,
    int32_t total_color_nodes,
    const int32_t* h_color_nodes) {

    g_ctx.n_edges = n_edges;
    alloc_and_copy(&g_ctx.d_edge_kab, h_edge_kab, (size_t)n_edges * 3);
    alloc_and_copy(&g_ctx.d_edge_d0, h_edge_d0, (size_t)n_edges);
    alloc_and_copy(&g_ctx.d_edge_req, h_edge_req, (size_t)n_edges);
    alloc_and_copy(&g_ctx.d_ne_indptr, h_ne_indptr, (size_t)(g_ctx.n_nodes + 1));
    alloc_and_copy(&g_ctx.d_ne_indices, h_ne_indices, (size_t)nnz_ne);
    alloc_and_copy(&g_ctx.d_color_nodes, h_color_nodes, (size_t)total_color_nodes);
}

void pcbmorph_cuda_copper_sweeps(
    double* h_P,
    const double* h_pad_shift,
    const double* h_outline,
    const double* h_press,
    const double* h_node_u,
    const double* h_node_len_slack,
    int32_t n_colors,
    const int32_t* h_color_offsets,
    int32_t n_sweeps,
    double sub_mm,
    double step,
    double k_sp,
    double w_pr,
    double w_field,
    double dz_mm,
    int32_t has_field) {

    size_t n2 = (size_t)g_ctx.n_nodes * 2;
    cudaMemcpy(g_ctx.d_P, h_P, sizeof(double) * n2, cudaMemcpyHostToDevice);
    if (g_ctx.n_pads > 0 && h_pad_shift != nullptr) {
        cudaMemcpy(g_ctx.d_pad_shift, h_pad_shift, sizeof(double) * (size_t)g_ctx.n_pads * 2, cudaMemcpyHostToDevice);
    }
    if (g_ctx.n_outline > 0 && h_outline != nullptr) {
        cudaMemcpy(g_ctx.d_outline, h_outline, sizeof(double) * (size_t)g_ctx.n_outline * 2, cudaMemcpyHostToDevice);
    }
    cudaMemcpy(g_ctx.d_press, h_press, sizeof(double) * n2, cudaMemcpyHostToDevice);
    if (has_field && h_node_u != nullptr) {
        cudaMemcpy(g_ctx.d_node_u, h_node_u, sizeof(double) * n2, cudaMemcpyHostToDevice);
    }
    cudaMemcpy(g_ctx.d_node_len_slack, h_node_len_slack, sizeof(double) * (size_t)g_ctx.n_nodes, cudaMemcpyHostToDevice);

    BoardDevView view = {
        g_ctx.d_P, g_ctx.d_seg_ab, g_ctx.d_seg_hw, g_ctx.d_via_r,
        g_ctx.d_pad_xy0, g_ctx.d_pad_dr, g_ctx.d_pad_shift,
        g_ctx.d_pad_verts0, g_ctx.d_pad_offsets,
        g_ctx.d_st_tracks, g_ctx.d_st_vias,
        g_ctx.d_zone_verts, g_ctx.d_zone_offsets,
        g_ctx.d_outline, g_ctx.n_outline
    };

    for (int s = 0; s < n_sweeps; ++s) {
        for (int c = 0; c < n_colors; ++c) {
            int off = h_color_offsets[c];
            int count = h_color_offsets[c + 1] - off;
            if (count <= 0) continue;
            int threads = 128;
            int blocks = (count + threads - 1) / threads;
            kernel_copper_color_pass<<<blocks, threads>>>(
                view,
                g_ctx.d_P,
                g_ctx.d_P0,
                g_ctx.d_press,
                g_ctx.d_node_u,
                g_ctx.d_color_nodes + off,
                count,
                g_ctx.d_adj_indptr,
                g_ctx.d_adj_indices,
                g_ctx.d_ne_indptr,
                g_ctx.d_ne_indices,
                g_ctx.d_edge_kab,
                g_ctx.d_edge_d0,
                g_ctx.d_edge_req,
                g_ctx.d_at_indptr,
                g_ctx.d_at_pad,
                g_ctx.d_at_cap,
                g_ctx.d_az_indptr,
                g_ctx.d_az_zone,
                g_ctx.d_node_len_slack,
                sub_mm, step, k_sp, w_pr, w_field, dz_mm, has_field);
        }
    }
    cudaMemcpy(h_P, g_ctx.d_P, sizeof(double) * n2, cudaMemcpyDeviceToHost);
}

// Host C++ fast evaluator for single-part / coupled / shape line searches (avoids PCIe roundtrip for tiny <200-edge subsets)
int pcbmorph_host_check_edges(
    const double* h_P,
    const int32_t* h_seg_ab,
    const double* h_seg_hw,
    const double* h_via_r,
    const double* h_pad_xy0,
    const double* h_pad_dr,
    const double* h_pad_shift,
    const double* h_pad_verts0,
    const int32_t* h_pad_offsets,
    const double* h_st_tracks,
    const double* h_st_vias,
    const double* h_zone_verts,
    const int32_t* h_zone_offsets,
    const double* h_outline,
    int32_t n_outline,
    const int32_t* h_edge_kab,
    const double* h_edge_d0,
    const double* h_edge_req,
    const int32_t* h_subset_indices,
    int32_t n_subset) {

    BoardDevView view = {
        h_P, h_seg_ab, h_seg_hw, h_via_r,
        h_pad_xy0, h_pad_dr, h_pad_shift,
        h_pad_verts0, h_pad_offsets,
        h_st_tracks, h_st_vias,
        h_zone_verts, h_zone_offsets,
        h_outline, n_outline
    };

    for (int i = 0; i < n_subset; ++i) {
        int ei = h_subset_indices[i];
        int kind = h_edge_kab[3 * ei];
        int a = h_edge_kab[3 * ei + 1];
        int b = h_edge_kab[3 * ei + 2];
        double d = d_eval_edge_one(view, kind, a, b, -1, 0.0, 0.0);
        if (!(d >= h_edge_d0[ei] || d >= h_edge_req[ei])) {
            return ei; // Return index of first blocking edge
        }
    }
    return -1; // All OK
}

} // extern "C"
