from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import heapq
import math
import time
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation
from scipy.interpolate import splprep, splev
HOST = "localhost"
PORT = 23000
ORIGIN_X = -2.5
ORIGIN_Y = -2.5
CELL_SIZE = 0.25
SAFETY_MARGIN = 1
START_XY = (-1.4, -1.8)
BASE_SPEED = 1.5
MAX_WHEEL_SPEED = 2.0
WHEEL_BASE = 0.331
LOOKAHEAD_DISTANCE = 0.45
MIN_LOOKAHEAD = 0.25
MAX_LOOKAHEAD = 0.70
FINAL_RADIUS = 0.15
NEAR_RADIUS = 0.6
TURN_ON = 0.9
TURN_OFF = 0.25
TURN_SPEED = 1.5
SIM_STEP_SLEEP = 0.02
MAX_RUN_SECONDS = 40
SPLINE_SMOOTHING = 0.08
SPLINE_SAMPLE_SPACING = 0.06
DELIVERY_PAUSE = 1.0
BEDS = [
    (-2.10, 1.15, -1.40, 2.45),
    (-1.25, 1.15, -0.55, 2.45),
    (-0.35, 1.15, 0.30, 2.45),
    (0.70, 1.15, 1.40, 2.45),
    (0.80, -1.70, 2.05, -1.00),
    (-2.50, -2.30, -1.65, -0.90),
]
DELIVERY_POINTS = {
    "Ward": (13, 4),
    "ICU": (14, 17),
    "Pharmacy": (3, 5),
    "Surgery": (5, 13),
}
TASKS = [
    {"name": "T1 medicines to Ward", "dest": "Ward", "priority": 1},
    {"name": "T2 blood bags to ICU", "dest": "ICU", "priority": 3},
    {"name": "T3 tools to Surgery", "dest": "Surgery", "priority": 2},
    {"name": "T4 drugs to Pharmacy", "dest": "Pharmacy", "priority": 2},
]
grid = np.load("grid.npy")
grid = (grid != 0).astype(np.uint8)
for b in BEDS:
    c0 = max(0, int((b[0] - ORIGIN_X) / CELL_SIZE))
    c1 = int((b[2] - ORIGIN_X) / CELL_SIZE)
    r0 = max(0, int((b[1] - ORIGIN_Y) / CELL_SIZE))
    r1 = int((b[3] - ORIGIN_Y) / CELL_SIZE)
    grid[r0:r1 + 1, c0:c1 + 1] = 1
planning_grid = binary_dilation(grid.astype(bool), iterations=SAFETY_MARGIN).astype(np.uint8)
safety_grid = binary_dilation(grid.astype(bool), iterations=1).astype(np.uint8)

def world_to_grid(x, y):
    return int((y - ORIGIN_Y) / CELL_SIZE), int((x - ORIGIN_X) / CELL_SIZE)

def grid_to_world(r, c):
    return (float(ORIGIN_X + c * CELL_SIZE + CELL_SIZE / 2),
            float(ORIGIN_Y + r * CELL_SIZE + CELL_SIZE / 2))

def in_bounds(c, shape):
    return 0 <= c[0] < shape[0] and 0 <= c[1] < shape[1]

def nearest_free_cell(cell, fg, max_radius=10):
    if in_bounds(cell, fg.shape) and fg[cell] == 0:
        return cell
    r0, c0 = cell
    for rad in range(1, max_radius + 1):
        cand = []
        for r in range(max(0, r0 - rad), min(fg.shape[0], r0 + rad + 1)):
            for c in range(max(0, c0 - rad), min(fg.shape[1], c0 + rad + 1)):
                if max(abs(r - r0), abs(c - c0)) == rad and fg[r, c] == 0:
                    cand.append((r, c))
        if cand:
            return min(cand, key=lambda p: (p[0] - r0) ** 2 + (p[1] - c0) ** 2)
    return None

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
    n = max(2, int(d / (CELL_SIZE / 5)) + 1)
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
        tck, _ = splprep([pts[:, 0], pts[:, 1]], s=SPLINE_SMOOTHING, k=min(3, len(pts) - 1))
        L = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
        cnt = max(len(pts) * 3, int(L / SPLINE_SAMPLE_SPACING) + 2)
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

def get_heading(sim, robot):
    m = sim.getObjectMatrix(robot, -1)
    return math.atan2(m[4], m[0])

def stop_robot(sim, lm, rm):
    sim.setJointTargetVelocity(lm, 0)
    sim.setJointTargetVelocity(rm, 0)

def find_lookahead_point(pts, pos, si, la):
    near = si
    nd = float("inf")
    for i in range(si, len(pts)):
        d = float(np.linalg.norm(pts[i] - pos))
        if d < nd:
            near, nd = i, d
        if i > near + 20:
            break
    ti = near
    for i in range(near, len(pts)):
        ti = i
        if np.linalg.norm(pts[i] - pos) >= la:
            break
    return pts[ti], near

def follow_path(sim, robot, lm, rm, pts, trace=None):
    t0 = time.time()
    idx = 0
    goal = pts[-1]
    reached = False
    best = float("inf")
    turning = False
    while time.time() - t0 < MAX_RUN_SECONDS:
        pos = np.array(sim.getObjectPosition(robot, -1)[:2], dtype=float)
        if trace is not None:
            trace.append(pos.copy())
        h = get_heading(sim, robot)
        gd = float(np.linalg.norm(goal - pos))
        best = min(best, gd)
        if gd <= FINAL_RADIUS:
            reached = True
            break
        if best < NEAR_RADIUS and gd > best + 0.05:
            break
        tgt, idx = find_lookahead_point(
            pts, pos, idx, min(MAX_LOOKAHEAD, max(MIN_LOOKAHEAD, LOOKAHEAD_DISTANCE)))
        dx, dy = tgt[0] - pos[0], tgt[1] - pos[1]
        a = math.atan2(dy, dx) - h
        a = math.atan2(math.sin(a), math.cos(a))
        if gd > 0.4:
            if turning and abs(a) < TURN_OFF:
                turning = False
            elif not turning and abs(a) > TURN_ON:
                turning = True
        else:
            turning = False
        if turning:
            w = TURN_SPEED if a > 0 else -TURN_SPEED
            sim.setJointTargetVelocity(lm, -w)
            sim.setJointTargetVelocity(rm, w)
            time.sleep(SIM_STEP_SLEEP)
            continue
        ly = -math.sin(h) * dx + math.cos(h) * dy
        dsq = max(dx * dx + dy * dy, 1e-8)
        k = 2.0 * ly / dsq
        sp = BASE_SPEED / (1.0 + 0.7 * abs(k))
        if gd < 0.6:
            sp *= max(0.25, gd / 0.6)
        w = sp * k
        l = float(np.clip(sp - w * WHEEL_BASE / 2, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))
        r = float(np.clip(sp + w * WHEEL_BASE / 2, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))
        sim.setJointTargetVelocity(lm, l)
        sim.setJointTargetVelocity(rm, r)
        time.sleep(SIM_STEP_SLEEP)
    stop_robot(sim, lm, rm)
    time.sleep(0.15)
    pos = np.array(sim.getObjectPosition(robot, -1)[:2], dtype=float)
    d = float(np.linalg.norm(goal - pos))
    if reached:
        print(f"    goal reached, distance={d:.3f} m")
    else:
        print(f"    [!] Stopped short, {d:.2f} m from destination.")
    return reached, d

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
    for n, c in DELIVERY_POINTS.items():
        f = nearest_free_cell(c, planning_grid, 15) or nearest_free_cell(c, grid, 15)
        if f is None:
            raise RuntimeError(f"No free cell near {n} {c}")
        if f != c:
            print(f"[!] {n}: {c} was blocked, using {f}")
        out[n] = f
    return out

def preview(points):
    s = nearest_free_cell(world_to_grid(*START_XY), planning_grid)
    print("Destination check from robot start:")
    for n, c in points.items():
        ok = astar(planning_grid, s, c) is not None
        x, y = grid_to_world(*c)
        print(f"  {n}: cell {c} world ({x:.2f},{y:.2f}) reachable: {'YES' if ok else 'NO'}")
    ext = [ORIGIN_X, ORIGIN_X + grid.shape[1] * CELL_SIZE, ORIGIN_Y, ORIGIN_Y + grid.shape[0] * CELL_SIZE]
    plt.figure(figsize=(7, 7))
    plt.imshow(planning_grid, cmap="Reds", origin="lower", extent=ext, alpha=0.4, interpolation="nearest")
    plt.imshow(grid, cmap="gray_r", origin="lower", extent=ext, alpha=0.7, interpolation="nearest")
    for n, c in points.items():
        x, y = grid_to_world(*c)
        plt.scatter(x, y, marker="*", s=180, c="gold", edgecolors="k", zorder=5)
        plt.annotate(n, (x, y), textcoords="offset points", xytext=(5, 5), fontsize=9)
    plt.scatter(*START_XY, marker="o", c="blue", zorder=5)
    plt.title("Destinations (red = safety margin)")
    plt.savefig("results/dest_preview.png", dpi=120)
    plt.close()
    print("Saved results/dest_preview.png")

points = resolve_delivery_points()
preview(points)
queue = []
for i, t in enumerate(TASKS):
    heapq.heappush(queue, (-t["priority"], i, t))
print("Tasks waiting:")
for _, _, t in sorted(queue, key=lambda q: (q[0], q[1])):
    print(f"  priority {t['priority']}  {t['name']} -> {t['dest']}")
client = RemoteAPIClient(HOST, PORT)
sim = client.require("sim")
robot = sim.getObject("/PioneerP3DX")
lm = sim.getObject("/PioneerP3DX/leftMotor")
rm = sim.getObject("/PioneerP3DX/rightMotor")
started_here = False
legs = []
trace = []
results = []
try:
    try:
        running = sim.getSimulationState() != sim.simulation_stopped
    except Exception:
        running = False
    if not running:
        sim.startSimulation()
        started_here = True
        time.sleep(0.4)
    order = 0
    while queue:
        neg_p, _, task = heapq.heappop(queue)
        order += 1
        cell = points[task["dest"]]
        pos_xy = sim.getObjectPosition(robot, -1)[:2]
        print(f"\n[{order}] {task['name']} (priority {-neg_p}) -> {task['dest']}")
        raw, smooth, final = plan_route(pos_xy, cell)
        if smooth is None:
            print("    [!] No path found. Skipping this task.")
            results.append((order, task["name"], task["dest"], -neg_p, 0.0, False, 0.0))
            continue
        print(f"    path: {len(raw)} cells, {len(smooth)} smooth points")
        t0 = time.time()
        reached, d = follow_path(sim, robot, lm, rm, smooth, trace=trace)
        el = time.time() - t0
        ok = reached or d <= NEAR_RADIUS
        if reached:
            print(f"    DELIVERED: {task['name']}")
        elif ok:
            print(f"    DELIVERED (nearby, {d:.2f} m off): {task['name']}")
        else:
            print(f"    FAILED (too far, {d:.2f} m): {task['name']}")
        if ok:
            time.sleep(DELIVERY_PAUSE)
        results.append((order, task["name"], task["dest"], -neg_p, el, ok, d))
        legs.append({"name": task["name"], "smooth": smooth})
finally:
    stop_robot(sim, lm, rm)
    if started_here:
        try:
            sim.stopSimulation()
        except Exception:
            pass
print("\n================ DELIVERY SUMMARY ================")
print(f"{'#':<3}{'Task':<26}{'To':<10}{'Prio':<6}{'Time(s)':<9}{'Off(m)':<8}{'Delivered'}")
for o, n, d, p, t, ok, off in results:
    print(f"{o:<3}{n:<26}{d:<10}{p:<6}{t:<9.1f}{off:<8.2f}{'YES' if ok else 'NO'}")
if legs:
    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap="gray_r", origin="lower",
               extent=[ORIGIN_X, ORIGIN_X + grid.shape[1] * CELL_SIZE,
                       ORIGIN_Y, ORIGIN_Y + grid.shape[0] * CELL_SIZE],
               interpolation="nearest")
    cols = plt.cm.tab10.colors
    for i, lg in enumerate(legs):
        plt.plot(lg["smooth"][:, 0], lg["smooth"][:, 1], "-", color=cols[i % len(cols)],
                 linewidth=2.5, label=f"{i+1}. {lg['name']}")
    if trace:
        a = np.array(trace)
        plt.plot(a[:, 0], a[:, 1], "k:", linewidth=1.2, label="Actual path")
    for n, (r, c) in points.items():
        x, y = grid_to_world(r, c)
        plt.scatter(x, y, marker="*", s=160, c="gold", edgecolors="k", zorder=5)
        plt.annotate(n, (x, y), textcoords="offset points", xytext=(5, 5), fontsize=9)
    plt.legend(fontsize=8)
    plt.title("Priority queue deliveries")
    plt.xlabel("World X (m)")
    plt.ylabel("World Y (m)")
    plt.axis("equal")
    plt.tight_layout()
    plt.savefig("results/queue_run.png", dpi=160)
    print("Saved results/queue_run.png")
    plt.show()
print("Queue run finished.")