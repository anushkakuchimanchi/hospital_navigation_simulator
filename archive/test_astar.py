import numpy as np
import heapq
import matplotlib.pyplot as plt

grid = np.load('grid.npy')

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

# pick a start and goal that are both on free (0) cells — check your grid_preview.png for open spots
ORIGIN_X = -2.5
ORIGIN_Y = -2.5
CELL_SIZE = 0.25
x=-1.4
y=-1.4
col = int((x - ORIGIN_X) / CELL_SIZE)
row = int((y - ORIGIN_Y) / CELL_SIZE)
goal = (18, 18)
start=(row,col)
path = astar(grid, start, goal)

if path is None:
    print("No path found — check start/goal aren't inside a wall")
else:
    print(f"Path found! {len(path)} steps")
    print(path)

    # visualize
    display_grid = grid.copy().astype(float)
    for r, c in path:
        display_grid[r, c] = 0.5
    plt.imshow(display_grid, cmap='gray_r', origin='lower')
    plt.title('A* Path')
    plt.savefig('astar_preview.png')
    print("Saved astar_preview.png")