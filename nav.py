from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import heapq
import math
import time
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation

# ---------- CONFIG ----------
ORIGIN_X = -2.5
ORIGIN_Y = -2.5
CELL_SIZE = 0.25
SAFETY_MARGIN = 1  # cells of clearance from walls for planning (drop to 1 if no path found)

# ---------- LOAD GRID ----------
grid = np.load('grid.npy')                                   # true wall layout (for collision/plot)
planning_grid = binary_dilation(grid, iterations=SAFETY_MARGIN).astype(int)  # inflated, for A* only

# ---------- A* ----------
def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(grid, start, goal):
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if not (0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]):
                continue
            if grid[neighbor[0], neighbor[1]] == 1:
                continue
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))
                came_from[neighbor] = current
    return None

# ---------- COORD CONVERSION ----------
def world_to_grid(x, y):
    col = int((x - ORIGIN_X) / CELL_SIZE)
    row = int((y - ORIGIN_Y) / CELL_SIZE)
    return row, col

def grid_to_world(row, col):
    x = ORIGIN_X + col * CELL_SIZE + CELL_SIZE/2
    y = ORIGIN_Y + row * CELL_SIZE + CELL_SIZE/2
    return x, y

# ---------- HEADING ----------
def get_heading(sim, robot):
    M = sim.getObjectMatrix(robot, -1)
    fx = M[0]
    fy = M[4]
    return math.atan2(fy, fx)

# ---------- MOVEMENT ----------
def move_to_point(sim, robot, left_motor, right_motor, target_x, target_y, speed=2.0):
    timeout = time.time() + 5
    while time.time() < timeout:
        pos = sim.getObjectPosition(robot, -1)
        current_heading = get_heading(sim, robot)
        dx = target_x - pos[0]
        dy = target_y - pos[1]
        target_heading = math.atan2(dy, dx)
        heading_error = math.atan2(math.sin(target_heading - current_heading),
                                    math.cos(target_heading - current_heading))
        if abs(heading_error) < 0.15:
            break
        turn_dir = 1 if heading_error > 0 else -1
        sim.setJointTargetVelocity(left_motor, -turn_dir * 1.0)
        sim.setJointTargetVelocity(right_motor, turn_dir * 1.0)
        time.sleep(0.02)
    else:
        print("  [!] Rotation phase timed out")

    sim.setJointTargetVelocity(left_motor, 0)
    sim.setJointTargetVelocity(right_motor, 0)
    time.sleep(0.1)

    timeout = time.time() + 8
    while time.time() < timeout:
        pos = sim.getObjectPosition(robot, -1)
        dx = target_x - pos[0]
        dy = target_y - pos[1]
        dist = math.sqrt(dx**2 + dy**2)
        if dist < 0.1:
            break
        sim.setJointTargetVelocity(left_motor, speed)
        sim.setJointTargetVelocity(right_motor, speed)
        time.sleep(0.02)
    else:
        print("  [!] Drive phase timed out — likely stuck against a wall")

    sim.setJointTargetVelocity(left_motor, 0)
    sim.setJointTargetVelocity(right_motor, 0)
    time.sleep(0.1)

# ---------- MAIN ----------
client = RemoteAPIClient()
sim = client.require('sim')

robot = sim.getObject('/PioneerP3DX')
left_motor = sim.getObject('/PioneerP3DX/leftMotor')
right_motor = sim.getObject('/PioneerP3DX/rightMotor')

sim.startSimulation()
time.sleep(0.3)

start_pos = sim.getObjectPosition(robot, -1)
start = world_to_grid(start_pos[0], start_pos[1])
goal = (16, 10)

print(f"Start grid cell: {start}, Goal: {goal}")

if planning_grid[start[0], start[1]] == 1:
    print("[!] WARNING: start cell blocked even after inflation — lower SAFETY_MARGIN")
if planning_grid[goal[0], goal[1]] == 1:
    print("[!] WARNING: goal cell blocked even after inflation — lower SAFETY_MARGIN")

path = astar(planning_grid, start, goal)

if path is None:
    print(f"No path found with SAFETY_MARGIN={SAFETY_MARGIN} — try lowering it to 1.")
    sim.stopSimulation()
    exit()

print(f"Path found: {len(path)} steps")

actual_positions = [start_pos[:2]]

for i, (row, col) in enumerate(path):
    wx, wy = grid_to_world(row, col)
    pos_before = sim.getObjectPosition(robot, -1)
    print(f"Waypoint {i+1}/{len(path)}: target grid=({row},{col}) world=({wx:.2f},{wy:.2f}) | robot at ({pos_before[0]:.2f},{pos_before[1]:.2f})")
    move_to_point(sim, robot, left_motor, right_motor, wx, wy)
    actual_positions.append(sim.getObjectPosition(robot, -1)[:2])

sim.stopSimulation()
print("Navigation complete!")

# ---------- PLOT ----------
planned = [grid_to_world(r, c) for r, c in path]

plt.figure(figsize=(6,6))
plt.imshow(grid, cmap='gray_r', origin='lower',
           extent=[ORIGIN_X, ORIGIN_X + grid.shape[1]*CELL_SIZE,
                   ORIGIN_Y, ORIGIN_Y + grid.shape[0]*CELL_SIZE])
plt.plot([p[0] for p in planned], [p[1] for p in planned], 'g-o', label='Planned', markersize=3)
plt.plot([p[0] for p in actual_positions], [p[1] for p in actual_positions], 'r-x', label='Actual', markersize=5)
plt.legend()
plt.title('Planned vs Actual Path')
plt.savefig('path_comparison.png')
print("Saved path_comparison.png")