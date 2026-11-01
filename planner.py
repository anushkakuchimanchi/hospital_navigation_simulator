import heapq
import numpy as np
from scipy.interpolate import splprep, splev
import config as C
from mapgrid import (grid, planning_grid, safety_grid, world_to_grid,
                     grid_to_world, in_bounds, nearest_free_cell)

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(occ, start, goal):
    if not in_bounds(start, occ.shape) or not in_bounds(goal, occ.shape):
        return None
    if occ[start] != 0 or occ[goal] != 0:
        return None
    heap = [(heuristic(start, goal), 0, start)]
    came = {}
    g = {start: 0}
    closed = set()
    while heap:
        _, _, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        closed.add(cur)
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            n = (cur[0] + dr, cur[1] + dc)
            if not in_bounds(n, occ.shape) or occ[n] != 0:
                continue
            t = g[cur] + 1
            if t < g.get(n, float("inf")):
                came[n] = cur
                g[n] = t
                heapq.heappush(heap, (t + heuristic(n, goal), t, n))
    return None

def grid_path_to_world(path):
    return np.array([grid_to_world(r, c) for r, c in path], dtype=float)

def segment_is_safe(a, b, sg):
    a = np.asarray(a)
    b = np.asarray(b)
    d = float(np.linalg.norm(b - a))
    n = max(2, int(d / (C.CELL_SIZE / 5)) + 1)
    for t in np.linspace(0, 1, n):
        p = a + t * (b - a)
        cell = world_to_grid(float(p[0]), float(p[1]))
        if not in_bounds(cell, sg.shape) or sg[cell] != 0:
            return False
    return True

def path_is_safe(pts, sg):
    return all(segment_is_safe(pts[i], pts[i + 1], sg) for i in range(len(pts) - 1))

def smooth_path(path, sg):
    pts = grid_path_to_world(path)
    if len(pts) < 4:
        return pts
    keep = np.ones(len(pts), dtype=bool)
    keep[1:] = np.linalg.norm(np.diff(pts, axis=0), axis=1) > 1e-8
    pts = pts[keep]
    if len(pts) < 4:
        return pts
    try:
        tck, _ = splprep([pts[:, 0], pts[:, 1]], s=C.SPLINE_SMOOTHING, k=min(3, len(pts) - 1))
        L = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
        cnt = max(len(pts) * 3, int(L / C.SPLINE_SAMPLE_SPACING) + 2)
        x, y = splev(np.linspace(0, 1, cnt), tck)
        sm = np.column_stack((x, y))
        sm[0], sm[-1] = pts[0], pts[-1]
        if path_is_safe(sm, sg):
            return sm
        print("[!] Spline too close to obstacle; using original A* path.")
        return pts
    except Exception as e:
        print(f"[!] Spline failed ({e}); using original A* path.")
        return pts

def plan_route(p, cell):
    s0 = world_to_grid(p[0], p[1])
    for m in (True, False):
        o = planning_grid if m else grid
        s = nearest_free_cell(s0, o)
        g = nearest_free_cell(cell, o)
        if s is None or g is None:
            continue
        path = astar(o, s, g)
        if path is not None:
            gx, gy = grid_to_world(*g)
            print(f"    A* start {s} | requested dest {cell} -> final dest {g} world ({gx:.2f},{gy:.2f})")
            if not m:
                print("    [!] No path with safety margin; used margin 0.")
            return grid_path_to_world(path), smooth_path(path, safety_grid), g
    return None, None, None

def resolve_delivery_points():
    out = {}
    for n, c in C.DELIVERY_POINTS.items():
        f = nearest_free_cell(c, planning_grid, 15) or nearest_free_cell(c, grid, 15)
        if f is None:
            raise RuntimeError(f"No free cell near {n} {c}")
        if f != c:
            print(f"[!] {n}: {c} was blocked, using {f}")
        out[n] = f
    return out