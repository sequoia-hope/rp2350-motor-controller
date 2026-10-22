"""CUDA GPU + C++ acceleration backend for `pcbmorph.settle`.

Loads `_cuda_accel.so` (auto-compiling `cuda_accel.cu` with `nvcc` if needed) and provides:
  - GPU all-pairs candidate distance evaluation (`eval_candidates`)
  - GPU graph-colored parallel `copper_pass` sweeps (`copper_sweeps`)
  - Fast C++/host subset edge checker (`check_edges_subset`) for `part_pass` and `shape_pass`
"""
import os
import ctypes
import subprocess
import numpy as np

KIND_LIST = [
    'SS', 'SP', 'SV', 'SST', 'SSV',
    'SH_P', 'SH_V', 'SH_SV',
    'VP', 'VV', 'VSV', 'VST',
    'VH_P', 'HH_VV', 'HH_VSV', 'HH_VP',
    'PP', 'PST', 'PSV',
    'SZ', 'VZ', 'PZ',
    'SE', 'VE', 'PE',
]
KIND_TO_ID = {k: i for i, k in enumerate(KIND_LIST)}
ID_TO_KIND = {i: k for i, k in enumerate(KIND_LIST)}

_LIB = None
_CUDA_OK = False


def _load_cuda_lib():
    global _LIB, _CUDA_OK
    if _LIB is not None:
        return _LIB if _CUDA_OK else None

    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    cu_path = os.path.join(pkg_dir, 'cuda_accel.cu')
    so_path = os.path.join(pkg_dir, '_cuda_accel.so')

    if os.path.exists(cu_path):
        need_build = (not os.path.exists(so_path) or
                      os.path.getmtime(cu_path) > os.path.getmtime(so_path))
        if need_build:
            try:
                subprocess.run(
                    ['nvcc', '-O3', '-arch=sm_86', '-shared',
                     '-Xcompiler', '-fPIC', cu_path, '-o', so_path],
                    check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
            except Exception:
                pass

    if not os.path.exists(so_path):
        _LIB = False
        _CUDA_OK = False
        return None

    try:
        lib = ctypes.CDLL(so_path)
        c_i32 = ctypes.c_int32
        c_dbl = ctypes.c_double
        p_i32 = ctypes.POINTER(ctypes.c_int32)
        p_dbl = ctypes.POINTER(ctypes.c_double)

        lib.pcbmorph_cuda_available.argtypes = []
        lib.pcbmorph_cuda_available.restype = ctypes.c_int

        lib.pcbmorph_cuda_init_static.argtypes = [
            c_i32, p_dbl, p_dbl,
            c_i32, p_dbl, p_i32,
            c_i32, p_dbl,
            c_i32, p_dbl,
            c_i32, c_i32, p_dbl, p_i32,
            c_i32, p_dbl,
        ]
        lib.pcbmorph_cuda_init_static.restype = None

        lib.pcbmorph_cuda_update_graph.argtypes = [
            c_i32, p_dbl, p_dbl, p_dbl,
            c_i32, p_i32, p_dbl,
            p_i32, c_i32, p_i32,
            p_i32, c_i32, p_i32, p_dbl,
            p_i32, c_i32, p_i32,
        ]
        lib.pcbmorph_cuda_update_graph.restype = None

        lib.pcbmorph_cuda_eval_candidates.argtypes = [
            p_dbl, p_dbl, p_dbl,
            c_i32, p_i32, p_dbl,
        ]
        lib.pcbmorph_cuda_eval_candidates.restype = None

        lib.pcbmorph_cuda_upload_edges.argtypes = [
            c_i32, p_i32, p_dbl, p_dbl,
            p_i32, c_i32, p_i32,
            c_i32, p_i32,
        ]
        lib.pcbmorph_cuda_upload_edges.restype = None

        lib.pcbmorph_cuda_copper_sweeps.argtypes = [
            p_dbl, p_dbl, p_dbl, p_dbl, p_dbl, p_dbl,
            c_i32, p_i32, c_i32,
            c_dbl, c_dbl, c_dbl, c_dbl, c_dbl, c_dbl, c_i32,
        ]
        lib.pcbmorph_cuda_copper_sweeps.restype = None

        lib.pcbmorph_host_check_edges.argtypes = [
            p_dbl, p_i32, p_dbl, p_dbl,
            p_dbl, p_dbl, p_dbl, p_dbl, p_i32,
            p_dbl, p_dbl, p_dbl, p_i32,
            p_dbl, c_i32,
            p_i32, p_dbl, p_dbl,
            p_i32, c_i32,
        ]
        lib.pcbmorph_host_check_edges.restype = ctypes.c_int

        _CUDA_OK = bool(lib.pcbmorph_cuda_available() == 1)
        _LIB = lib
        return lib if _CUDA_OK else None
    except Exception:
        _LIB = False
        _CUDA_OK = False
        return None


def _ptr_f64(arr):
    if arr is None or arr.size == 0:
        return ctypes.POINTER(ctypes.c_double)()
    return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_double))


def _ptr_i32(arr):
    if arr is None or arr.size == 0:
        return ctypes.POINTER(ctypes.c_int32)()
    return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_int32))


class CudaSettlerBackend:
    """Manages mirrored host/GPU arrays for fast CUDA evaluation in `run_settle`."""

    def __init__(self, pads, P_POLY0, P_XY0, P_DRILL, P_REF,
                 st_tracks, st_vias, zones, outline):
        self.lib = _load_cuda_lib()
        if self.lib is None:
            raise RuntimeError("CUDA library not available")

        self.n_pads = len(pads)
        self.pad_refs = list(P_REF)
        self.h_pad_xy0 = np.array(P_XY0, dtype=np.float64).reshape(-1, 2) if self.n_pads else np.zeros((0, 2), dtype=np.float64)
        self.h_pad_dr = np.array([0.5 * d for d in P_DRILL], dtype=np.float64)
        self.h_pad_shift = np.zeros((self.n_pads, 2), dtype=np.float64)

        pad_verts = []
        pad_offsets = [0]
        for poly in P_POLY0:
            for x, y in poly:
                pad_verts.append((x, y))
            pad_offsets.append(len(pad_verts))
        self.h_pad_verts0 = np.array(pad_verts, dtype=np.float64).reshape(-1, 2) if pad_verts else np.zeros((0, 2), dtype=np.float64)
        self.h_pad_offsets = np.array(pad_offsets, dtype=np.int32)

        self.h_st_tracks = np.array(
            [(t[0], t[1], t[2], t[3], t[4]) for t in st_tracks],
            dtype=np.float64
        ).reshape(-1, 5) if st_tracks else np.zeros((0, 5), dtype=np.float64)

        self.h_st_vias = np.array(
            [(v[0], v[1], v[2], v[3]) for v in st_vias],
            dtype=np.float64
        ).reshape(-1, 4) if st_vias else np.zeros((0, 4), dtype=np.float64)

        z_verts = []
        z_offsets = [0]
        for z in zones:
            for x, y in z['poly']:
                z_verts.append((x, y))
            z_offsets.append(len(z_verts))
        self.h_zone_verts = np.array(z_verts, dtype=np.float64).reshape(-1, 2) if z_verts else np.zeros((0, 2), dtype=np.float64)
        self.h_zone_offsets = np.array(z_offsets, dtype=np.int32)

        self.h_outline = np.array(outline, dtype=np.float64).reshape(-1, 2) if outline else np.zeros((0, 2), dtype=np.float64)

        # Pad indices per movable ref for fast pad_shift updates
        self.ref_pad_indices = {}
        for pi, r in enumerate(self.pad_refs):
            self.ref_pad_indices.setdefault(r, []).append(pi)
        for r in self.ref_pad_indices:
            self.ref_pad_indices[r] = np.array(self.ref_pad_indices[r], dtype=np.int32)

        self.lib.pcbmorph_cuda_init_static(
            self.n_pads, _ptr_f64(self.h_pad_xy0), _ptr_f64(self.h_pad_dr),
            len(pad_verts), _ptr_f64(self.h_pad_verts0), _ptr_i32(self.h_pad_offsets),
            len(st_tracks), _ptr_f64(self.h_st_tracks),
            len(st_vias), _ptr_f64(self.h_st_vias),
            len(zones), len(z_verts), _ptr_f64(self.h_zone_verts), _ptr_i32(self.h_zone_offsets),
            len(outline), _ptr_f64(self.h_outline),
        )

        # Dynamic graph buffers
        self.n_nodes = 0
        self.h_P = None
        self.h_P0 = None
        self.h_via_r = None
        self.h_seg_ab = None
        self.h_seg_hw = None
        self.h_edge_kab = None
        self.h_edge_d0 = None
        self.h_edge_req = None
        self.h_color_offsets = None
        self.part_edge_subsets = {}
        self.node_edge_subsets = {}
        self.shape_edge_subset = np.zeros(0, dtype=np.int32)

    def sync_state(self, P, D, outline):
        """Sync Python `P`, `D`, and `outline` into contiguous host arrays."""
        self.h_P[:] = P
        for r, (dx, dy) in D.items():
            idx = self.ref_pad_indices.get(r)
            if idx is not None:
                self.h_pad_shift[idx, 0] = dx
                self.h_pad_shift[idx, 1] = dy
        if outline:
            self.h_outline[:] = outline

    def update_graph(self, P, P0, is_via, segs, adj, AT, AZ):
        """Upload copper graph topology (nodes, segs, CSR adjacency, attachments) to GPU."""
        n_nodes = len(P)
        self.n_nodes = n_nodes
        self.h_P = np.array(P, dtype=np.float64).reshape(n_nodes, 2)
        self.h_P0 = np.array(P0, dtype=np.float64).reshape(n_nodes, 2)
        self.h_via_r = np.zeros((n_nodes, 2), dtype=np.float64)
        for vid, (vr, dr) in is_via.items():
            self.h_via_r[vid, 0] = vr
            self.h_via_r[vid, 1] = dr

        n_segs = len(segs)
        self.h_seg_ab = np.array([(s[0], s[1]) for s in segs], dtype=np.int32).reshape(n_segs, 2)
        self.h_seg_hw = np.array([0.5 * s[2] for s in segs], dtype=np.float64)

        adj_indptr = [0]
        adj_indices = []
        for nid in range(n_nodes):
            for u in sorted(adj[nid]):
                adj_indices.append(u)
            adj_indptr.append(len(adj_indices))
        h_adj_indptr = np.array(adj_indptr, dtype=np.int32)
        h_adj_indices = np.array(adj_indices, dtype=np.int32)

        at_by_node = {}
        for nid, pi, cap in AT:
            at_by_node.setdefault(nid, []).append((pi, cap))
        at_indptr = [0]
        at_pad = []
        at_cap = []
        for nid in range(n_nodes):
            for pi, cap in at_by_node.get(nid, ()):
                at_pad.append(pi)
                at_cap.append(cap)
            at_indptr.append(len(at_pad))
        h_at_indptr = np.array(at_indptr, dtype=np.int32)
        h_at_pad = np.array(at_pad, dtype=np.int32)
        h_at_cap = np.array(at_cap, dtype=np.float64)

        az_by_node = {}
        for nid, zi, _ in AZ:
            az_by_node.setdefault(nid, []).append(zi)
        az_indptr = [0]
        az_zone = []
        for nid in range(n_nodes):
            for zi in az_by_node.get(nid, ()):
                az_zone.append(zi)
            az_indptr.append(len(az_zone))
        h_az_indptr = np.array(az_indptr, dtype=np.int32)
        h_az_zone = np.array(az_zone, dtype=np.int32)

        self.lib.pcbmorph_cuda_update_graph(
            n_nodes, _ptr_f64(self.h_P), _ptr_f64(self.h_P0), _ptr_f64(self.h_via_r),
            n_segs, _ptr_i32(self.h_seg_ab), _ptr_f64(self.h_seg_hw),
            _ptr_i32(h_adj_indptr), len(adj_indices), _ptr_i32(h_adj_indices),
            _ptr_i32(h_at_indptr), len(at_pad), _ptr_i32(h_at_pad), _ptr_f64(h_at_cap),
            _ptr_i32(h_az_indptr), len(az_zone), _ptr_i32(h_az_zone),
        )

    def eval_candidates(self, P, D, outline, cand_kab):
        """Evaluate an array of `(kind_id, a, b)` candidate edges on the RTX 3060 GPU."""
        self.sync_state(P, D, outline)
        n_cand = len(cand_kab)
        if n_cand == 0:
            return np.zeros(0, dtype=np.float64)
        h_cand = np.ascontiguousarray(cand_kab, dtype=np.int32).reshape(n_cand, 3)
        h_out = np.empty(n_cand, dtype=np.float64)
        self.lib.pcbmorph_cuda_eval_candidates(
            _ptr_f64(self.h_P), _ptr_f64(self.h_pad_shift), _ptr_f64(self.h_outline),
            n_cand, _ptr_i32(h_cand), _ptr_f64(h_out)
        )
        return h_out

    def upload_edges_and_colors(self, edges, node_edges, part_edges, shape_edges,
                                fixed, part_of, adj, guard_mm):
        """Upload active edge table + CSR node_edges and build conflict-free GPU node coloring."""
        n_edges = len(edges)
        self.h_edge_kab = np.array(
            [(KIND_TO_ID[g.kind], g.a, g.b) for g in edges],
            dtype=np.int32
        ).reshape(n_edges, 3) if n_edges else np.zeros((0, 3), dtype=np.int32)
        self.h_edge_d0 = np.array([g.d0 for g in edges], dtype=np.float64)
        self.h_edge_req = np.array([g.rule + guard_mm for g in edges], dtype=np.float64)

        ne_indptr = [0]
        ne_indices = []
        for nid in range(self.n_nodes):
            el = node_edges.get(nid, ())
            ne_indices.extend(el)
            ne_indptr.append(len(ne_indices))
        h_ne_indptr = np.array(ne_indptr, dtype=np.int32)
        h_ne_indices = np.array(ne_indices, dtype=np.int32)

        # Build conflict graph for free nodes (nodes sharing a segment or a DRC edge)
        free_nodes = [nid for nid in range(self.n_nodes) if nid not in fixed and nid not in part_of]
        edge_to_free = {}
        for nid in free_nodes:
            for ei in node_edges.get(nid, ()):
                edge_to_free.setdefault(ei, []).append(nid)

        conflict = {nid: set() for nid in free_nodes}
        for nid in free_nodes:
            for nb in adj[nid]:
                if nb in conflict:
                    conflict[nid].add(nb)
        for nlist in edge_to_free.values():
            if len(nlist) > 1:
                for u in nlist:
                    for v in nlist:
                        if u != v:
                            conflict[u].add(v)

        # Greedy graph coloring
        node_color = {}
        color_groups = []
        for nid in free_nodes:
            used = {node_color[nb] for nb in conflict[nid] if nb in node_color}
            c = 0
            while c in used:
                c += 1
            node_color[nid] = c
            while len(color_groups) <= c:
                color_groups.append([])
            color_groups[c].append(nid)

        flat_colors = []
        color_offsets = [0]
        for grp in color_groups:
            flat_colors.extend(grp)
            color_offsets.append(len(flat_colors))
        h_color_nodes = np.array(flat_colors, dtype=np.int32)
        self.h_color_offsets = np.array(color_offsets, dtype=np.int32)

        self.lib.pcbmorph_cuda_upload_edges(
            n_edges, _ptr_i32(self.h_edge_kab), _ptr_f64(self.h_edge_d0), _ptr_f64(self.h_edge_req),
            _ptr_i32(h_ne_indptr), len(ne_indices), _ptr_i32(h_ne_indices),
            len(flat_colors), _ptr_i32(h_color_nodes),
        )

        # Pre-pack subset index arrays for fast C++ part/node/shape trial checks
        self.part_edge_subsets = {
            r: np.array(el, dtype=np.int32) for r, el in part_edges.items()
        }
        self.node_edge_subsets = {
            nid: np.array(el, dtype=np.int32) for nid, el in node_edges.items()
        }
        self.shape_edge_subset = np.array(shape_edges, dtype=np.int32)

    def check_edges_subset(self, P, D, outline, subset_arr):
        """Return index `ei` of first violated edge in `subset_arr`, or `-1` if all pass."""
        if subset_arr is None or subset_arr.size == 0:
            return -1
        self.sync_state(P, D, outline)
        return int(self.lib.pcbmorph_host_check_edges(
            _ptr_f64(self.h_P),
            _ptr_i32(self.h_seg_ab),
            _ptr_f64(self.h_seg_hw),
            _ptr_f64(self.h_via_r),
            _ptr_f64(self.h_pad_xy0),
            _ptr_f64(self.h_pad_dr),
            _ptr_f64(self.h_pad_shift),
            _ptr_f64(self.h_pad_verts0),
            _ptr_i32(self.h_pad_offsets),
            _ptr_f64(self.h_st_tracks),
            _ptr_f64(self.h_st_vias),
            _ptr_f64(self.h_zone_verts),
            _ptr_i32(self.h_zone_offsets),
            _ptr_f64(self.h_outline),
            len(self.h_outline),
            _ptr_i32(self.h_edge_kab),
            _ptr_f64(self.h_edge_d0),
            _ptr_f64(self.h_edge_req),
            _ptr_i32(subset_arr),
            len(subset_arr),
        ))

    def copper_sweeps(self, P, D, outline, press, field_stack, N_NET, LEN_CAP, net_len_fn,
                      n_sweeps, sub_mm, step, k_sp=0.35, w_pr=0.45, w_field=0.65, dz_mm=0.015):
        """Run `n_sweeps` graph-colored parallel `copper_pass` iterations on the RTX 3060 GPU."""
        self.sync_state(P, D, outline)

        h_press = np.zeros((self.n_nodes, 2), dtype=np.float64)
        for nid, (px, py) in press.items():
            if 0 <= nid < self.n_nodes:
                h_press[nid, 0] = px
                h_press[nid, 1] = py

        has_field = 1 if field_stack is not None else 0
        if has_field:
            h_node_u = np.array(field_stack.node_u[:self.n_nodes], dtype=np.float64).reshape(self.n_nodes, 2)
        else:
            h_node_u = np.zeros((self.n_nodes, 2), dtype=np.float64)

        net_rem = {net: max(0.0, cap - net_len_fn(net)) for net, cap in LEN_CAP.items()}
        h_node_len_slack = np.array(
            [net_rem.get(N_NET[nid], 99.0) * 0.35 for nid in range(self.n_nodes)],
            dtype=np.float64
        )

        n_colors = max(0, len(self.h_color_offsets) - 1)
        self.lib.pcbmorph_cuda_copper_sweeps(
            _ptr_f64(self.h_P),
            _ptr_f64(self.h_pad_shift),
            _ptr_f64(self.h_outline),
            _ptr_f64(h_press),
            _ptr_f64(h_node_u),
            _ptr_f64(h_node_len_slack),
            n_colors,
            _ptr_i32(self.h_color_offsets),
            int(n_sweeps),
            float(sub_mm),
            float(step),
            float(k_sp),
            float(w_pr),
            float(w_field),
            float(dz_mm),
            int(has_field),
        )

        # Write updated GPU positions back into Python list P
        pts_list = self.h_P.tolist()
        for nid in range(self.n_nodes):
            P[nid][0] = pts_list[nid][0]
            P[nid][1] = pts_list[nid][1]
