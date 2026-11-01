from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import heapq
import math
import time
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation
from scipy.interpolate import splprep, splev

# ---------------- CONFIG ----------------
HOST = "localhost"
PORT = 23000
ORIGIN_X = -2.5
ORIGIN_Y = -2.5
CELL_SIZE = 0.25
SAFETY_MARGIN = 1
GOAL = (16, 4)  # (row, col)

BASE_SPEED = 1.5
MAX_WHEEL_SPEED = 2.0
WHEEL_BASE = 0.331
LOOKAHEAD_DISTANCE = 0.45
MIN_LOOKAHEAD = 0.25
MAX_LOOKAHEAD = 0.70
FINAL_RADIUS = 0.10
SIM_STEP_SLEEP = 0.02
MAX_RUN_SECONDS = 180

SPLINE_SMOOTHING = 0.08
SPLINE_SAMPLE_SPACING = 0.06

# ---------------- GRID ----------------
grid = np.load("grid.npy")
if grid.ndim != 2:
    raise ValueError(f"grid.npy must be 2D; got shape {grid.shape}")
grid = (grid != 0).astype(np.uint8)
planning_grid = binary_dilation(grid.astype(bool), iterations=SAFETY_MARGIN).astype(np.uint8)


# ---------------- COORDINATES ----------------
def world_to_grid(x, y):
    col = int((x - ORIGIN_X) / CELL_SIZE)
    row = int((y - ORIGIN_Y) / CELL_SIZE)
    return row, col


def grid_to_world(row, col):
    return (
        float(ORIGIN_X + col * CELL_SIZE + CELL_SIZE / 2),
        float(ORIGIN_Y + row * CELL_SIZE + CELL_SIZE / 2),
    )


def in_bounds(cell, shape):
    return 0 <= cell[0] < shape[0] and 0 <= cell[1] < shape[1]


def nearest_free_cell(cell, free_grid, max_radius=10):
    if in_bounds(cell, free_grid.shape) and free_grid[cell] == 0:
        return cell
    r0, c0 = cell
    for radius in range(1, max_radius + 1):
        candidates = []
        for r in range(max(0, r0-radius), min(free_grid.shape[0], r0+radius+1)):
            for c in range(max(0, c0-radius), min(free_grid.shape[1], c0+radius+1)):
                if max(abs(r-r0), abs(c-c0)) == radius and free_grid[r, c] == 0:
                    candidates.append((r, c))
        if candidates:
            return min(candidates, key=lambda p: (p[0]-r0)**2 + (p[1]-c0)**2)
    return None


# ---------------- A* ----------------
def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])


def astar(occupancy, start, goal):
    if not in_bounds(start, occupancy.shape) or not in_bounds(goal, occupancy.shape):
        return None
    if occupancy[start] != 0 or occupancy[goal] != 0:
        return None

    open_heap = [(heuristic(start, goal), 0, start)]
    came_from = {}
    g_score = {start: 0}
    closed = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        closed.add(current)
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nxt = (current[0]+dr, current[1]+dc)
            if not in_bounds(nxt, occupancy.shape) or occupancy[nxt] != 0:
                continue
            tentative = g_score[current] + 1
            if tentative < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative
                heapq.heappush(open_heap, (tentative + heuristic(nxt, goal), tentative, nxt))
    return None


# ---------------- SPLINE + SAFETY ----------------
def grid_path_to_world(path):
    return np.array([grid_to_world(r, c) for r, c in path], dtype=float)


def world_to_grid_cell(point):
    return world_to_grid(float(point[0]), float(point[1]))


def segment_is_safe(a, b, safe_grid):
    distance = float(np.linalg.norm(np.asarray(b) - np.asarray(a)))
    n = max(2, int(distance / (CELL_SIZE / 5)) + 1)
    for t in np.linspace(0, 1, n):
        p = np.asarray(a) + t * (np.asarray(b) - np.asarray(a))
        cell = world_to_grid_cell(p)
        if not in_bounds(cell, safe_grid.shape) or safe_grid[cell] != 0:
            return False
    return True


def path_is_safe(points, safe_grid):
    return all(segment_is_safe(points[i], points[i+1], safe_grid)
               for i in range(len(points)-1))


def smooth_path(path, safe_grid):
    points = grid_path_to_world(path)
    if len(points) < 4:
        return points

    keep = np.ones(len(points), dtype=bool)
    keep[1:] = np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-8
    points = points[keep]
    if len(points) < 4:
        return points

    try:
        tck, _ = splprep([points[:, 0], points[:, 1]],
                         s=SPLINE_SMOOTHING, k=min(3, len(points)-1))
        length = float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
        count = max(len(points)*3, int(length/SPLINE_SAMPLE_SPACING)+2)
        u = np.linspace(0, 1, count)
        x, y = splev(u, tck)
        smooth = np.column_stack((x, y))
        smooth[0], smooth[-1] = points[0], points[-1]
        if path_is_safe(smooth, safe_grid):
            return smooth
        print("[!] Smoothed spline is too close to an obstacle; using original A* path.")
        return points
    except Exception as exc:
        print(f"[!] Spline smoothing failed ({exc}); using original A* path.")
        return points


# ---------------- PURE PURSUIT ----------------
def get_heading(sim, robot):
    matrix = sim.getObjectMatrix(robot, -1)
    return math.atan2(matrix[4], matrix[0])


def stop_robot(sim, left_motor, right_motor):
    sim.setJointTargetVelocity(left_motor, 0)
    sim.setJointTargetVelocity(right_motor, 0)


def find_lookahead_point(points, position, start_index, lookahead):
    nearest = start_index
    nearest_dist = float("inf")
    for i in range(start_index, len(points)):
        d = float(np.linalg.norm(points[i] - position))
        if d < nearest_dist:
            nearest, nearest_dist = i, d
        if i > nearest + 20:
            break

    target_idx = nearest
    for i in range(nearest, len(points)):
        target_idx = i
        if np.linalg.norm(points[i] - position) >= lookahead:
            break
    return points[target_idx], nearest, target_idx


def follow_path(sim, robot, left_motor, right_motor, points):
    started = time.time()
    index = 0
    goal = points[-1]

    while time.time() - started < MAX_RUN_SECONDS:
        pos3 = sim.getObjectPosition(robot, -1)
        pos = np.array(pos3[:2], dtype=float)
        heading = get_heading(sim, robot)
        goal_distance = float(np.linalg.norm(goal - pos))
        if goal_distance <= FINAL_RADIUS:
            print(f"[OK] Goal reached; distance={goal_distance:.3f} m")
            break

        target, index, _ = find_lookahead_point(
            points, pos, index,
            min(MAX_LOOKAHEAD, max(MIN_LOOKAHEAD, LOOKAHEAD_DISTANCE))
        )
        dx, dy = target[0]-pos[0], target[1]-pos[1]
        local_y = -math.sin(heading)*dx + math.cos(heading)*dy
        distance_sq = max(dx*dx + dy*dy, 1e-8)
        curvature = 2.0*local_y/distance_sq

        speed = BASE_SPEED/(1.0 + 0.7*abs(curvature))
        if goal_distance < 0.6:
            speed *= max(0.25, goal_distance/0.6)

        angular = speed*curvature
        left = speed - angular*WHEEL_BASE/2
        right = speed + angular*WHEEL_BASE/2
        left = float(np.clip(left, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))
        right = float(np.clip(right, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))

        sim.setJointTargetVelocity(left_motor, left)
        sim.setJointTargetVelocity(right_motor, right)
        time.sleep(SIM_STEP_SLEEP)
    else:
        print("[!] Navigation timed out.")

    stop_robot(sim, left_motor, right_motor)
    time.sleep(0.15)


# ---------------- MAIN ----------------
client = RemoteAPIClient("localhost", PORT)
sim = client.require("sim")

robot = sim.getObject("/PioneerP3DX")
left_motor = sim.getObject("/PioneerP3DX/leftMotor")
right_motor = sim.getObject("/PioneerP3DX/rightMotor")

print("Robot found: PioneerP3DX")
print(f"Left motor: {left_motor}")
print(f"Right motor: {right_motor}")

started_here = False
planned_raw = None
planned_smooth = None
actual_positions = []

try:
    try:
        running = sim.getSimulationState() != sim.simulation_stopped
    except Exception:
        running = False

    if not running:
        sim.startSimulation()
        started_here = True
        time.sleep(0.4)

    start_pos = sim.getObjectPosition(robot, -1)
    start = world_to_grid(start_pos[0], start_pos[1])
    goal = GOAL
    print(f"Planning from {start} to {goal}")

    if not in_bounds(start, planning_grid.shape):
        raise RuntimeError(
            f"Start {start} outside grid shape {planning_grid.shape}. "
            "Check origin and cell size."
        )
    if not in_bounds(goal, planning_grid.shape):
        raise RuntimeError(
            f"Goal {goal} outside grid shape {planning_grid.shape}. "
            "Adjust GOAL to a valid grid cell."
        )

    start_free = nearest_free_cell(start, planning_grid)
    goal_free = nearest_free_cell(goal, planning_grid)
    if start_free is None or goal_free is None:
        raise RuntimeError("Could not find a free start or goal cell.")
    if start_free != start:
        print(f"[!] Start cell blocked; planning from nearby free cell {start_free}")
    if goal_free != goal:
        print(f"[!] Goal cell blocked; planning to nearby free cell {goal_free}")

    path = astar(planning_grid, start_free, goal_free)

    if path is None and SAFETY_MARGIN > 0:
        print(f"[!] No path with SAFETY_MARGIN={SAFETY_MARGIN}; retrying with 0.")
        planning_grid = grid.copy()
        start_free = nearest_free_cell(start, planning_grid)
        goal_free = nearest_free_cell(goal, planning_grid)
        if start_free is not None and goal_free is not None:
            path = astar(planning_grid, start_free, goal_free)

    if path is None:
        raise RuntimeError(
            "No A* path found even with SAFETY_MARGIN=0. Check grid.npy, "
            "goal coordinates, and map connectivity."
        )

    print(f"Path found: {len(path)} cells")
    planned_raw = grid_path_to_world(path)

    # Validate smoothing against a one-cell inflated obstacle map.
    safety_grid = binary_dilation(grid.astype(bool), iterations=1).astype(np.uint8)
    planned_smooth = smooth_path(path, safety_grid)
    print(f"Following path with {len(planned_smooth)} points using Pure Pursuit.")

    actual_positions.append(np.array(start_pos[:2], dtype=float))
    follow_path(sim, robot, left_motor, right_motor, planned_smooth)
    actual_positions.append(np.array(sim.getObjectPosition(robot, -1)[:2], dtype=float))

finally:
    stop_robot(sim, left_motor, right_motor)
    if started_here:
        try:
            sim.stopSimulation()
        except Exception:
            pass

# ---------------- PLOT ----------------
if planned_raw is not None:
    plt.figure(figsize=(8, 8))
    plt.imshow(
        grid, cmap="gray_r", origin="lower",
        extent=[
            ORIGIN_X, ORIGIN_X + grid.shape[1]*CELL_SIZE,
            ORIGIN_Y, ORIGIN_Y + grid.shape[0]*CELL_SIZE
        ],
        interpolation="nearest"
    )
    plt.plot(planned_raw[:, 0], planned_raw[:, 1], "g--",
             label="A* path", linewidth=1.5)
    if planned_smooth is not None:
        plt.plot(planned_smooth[:, 0], planned_smooth[:, 1], "b-",
                 label="Spline / Pure Pursuit path", linewidth=2)
    if actual_positions:
        actual = np.array(actual_positions)
        plt.plot(actual[:, 0], actual[:, 1], "r-x",
                 label="Actual positions", markersize=6)
    plt.scatter(planned_raw[0, 0], planned_raw[0, 1], marker="o", label="Start")
    plt.scatter(planned_raw[-1, 0], planned_raw[-1, 1], marker="*", s=100, label="Goal")
    plt.legend()
    plt.title("Hospital Navigation: A* + Safe Spline + Pure Pursuit")
    plt.xlabel("World X (m)")
    plt.ylabel("World Y (m)")
    plt.axis("equal")
    plt.tight_layout()
    plt.savefig("path_comparison.png", dpi=160)
    print("Saved path_comparison.png")
    plt.show()

print("Navigation script finished.")
