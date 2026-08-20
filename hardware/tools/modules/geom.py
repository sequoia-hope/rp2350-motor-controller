"""Small 2D geometry kit: polygon distances with bbox prefilter (pure python)."""
import math
def seg_seg(p1, p2, q1, q2):
    def pt_seg(p, a, b):
        ax, ay = a; bx, by = b; px, py = p
        dx, dy = bx-ax, by-ay; L2 = dx*dx+dy*dy
        if L2 < 1e-18: return math.hypot(px-ax, py-ay)
        t = max(0.0, min(1.0, ((px-ax)*dx+(py-ay)*dy)/L2))
        return math.hypot(px-ax-t*dx, py-ay-t*dy)
    def orient(a, b, c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    d1, d2 = orient(q1, q2, p1), orient(q1, q2, p2); d3, d4 = orient(p1, p2, q1), orient(p1, p2, q2)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 != 0 and d2 != 0 and d3 != 0 and d4 != 0: return 0.0
    return min(pt_seg(p1, q1, q2), pt_seg(p2, q1, q2), pt_seg(q1, p1, p2), pt_seg(q2, p1, p2))
def pip(x, y, pts):
    inside = False; j = len(pts)-1
    for i in range(len(pts)):
        xi, yi = pts[i]; xj, yj = pts[j]
        if (yi > y) != (yj > y) and x < (xj-xi)*(y-yi)/(yj-yi+1e-300)+xi: inside = not inside
        j = i
    return inside
def bbox(pts):
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; return (min(xs), min(ys), max(xs), max(ys))
def bbox_dist(a, b):
    dx = max(a[0]-b[2], b[0]-a[2], 0.0); dy = max(a[1]-b[3], b[1]-a[3], 0.0); return math.hypot(dx, dy)
def poly_dist(A, B, bA=None, bB=None):
    """min distance between polygon boundaries/interiors (0 if overlapping)."""
    bA = bA or bbox(A); bB = bB or bbox(B)
    lb = bbox_dist(bA, bB)
    if pip(A[0][0], A[0][1], B) or pip(B[0][0], B[0][1], A):
        return -overlap_depth(bA, bB)
    best = 1e9; n, m = len(A), len(B)
    for i in range(n):
        a1, a2 = A[i], A[(i+1) % n]
        for j in range(m):
            d = seg_seg(a1, a2, B[j], B[(j+1) % m])
            if d < best:
                best = d
                if best <= lb + 1e-12: return best
    if best < 1e-9:  # edges cross: overlapping polygons
        return -overlap_depth(bA, bB)
    return best
def overlap_depth(a, b):
    """penetration proxy: min axis overlap of the two bboxes (>= 0)."""
    return max(0.0, min(min(a[2], b[2]) - max(a[0], b[0]), min(a[3], b[3]) - max(a[1], b[1])))
def translate(P, dx, dy): return [(p[0]+dx, p[1]+dy) for p in P]
def rect_poly(b): return [(b[0], b[1]), (b[2], b[1]), (b[2], b[3]), (b[0], b[3])]
def seg_seg_closest(p1, p2, q1, q2):
    """(dist, point_on_p, point_on_q) for two segments."""
    def pt_seg(p, a, b):
        ax, ay = a; bx, by = b; px, py = p
        dx, dy = bx-ax, by-ay; L2 = dx*dx+dy*dy
        if L2 < 1e-18: return math.hypot(px-ax, py-ay), a
        t = max(0.0, min(1.0, ((px-ax)*dx+(py-ay)*dy)/L2))
        c = (ax+t*dx, ay+t*dy); return math.hypot(px-c[0], py-c[1]), c
    best = None
    for p, a, b, swap in ((p1, q1, q2, False), (p2, q1, q2, False), (q1, p1, p2, True), (q2, p1, p2, True)):
        d, c = pt_seg(p, a, b)
        if best is None or d < best[0]:
            best = (d, c, p) if swap else (d, p, c)
    return best
def poly_closest(A, B):
    """(dist, pa, pb); dist<0 (overlap proxy) with centroid-based points when overlapping."""
    d = poly_dist(A, B)
    if d <= 0:
        ca = (sum(p[0] for p in A)/len(A), sum(p[1] for p in A)/len(A)); cb = (sum(p[0] for p in B)/len(B), sum(p[1] for p in B)/len(B))
        return d, ca, cb
    best = None; n, m = len(A), len(B)
    for i in range(n):
        for j in range(m):
            r = seg_seg_closest(A[i], A[(i+1) % n], B[j], B[(j+1) % m])
            if best is None or r[0] < best[0]: best = r
    return best
