import numpy as np
from scipy.ndimage import binary_dilation
import config as C
grid = (np.load("grid.npy") != 0).astype(np.uint8)
for b in C.BEDS:
    c0 = max(0, int((b[0] - C.ORIGIN_X) / C.CELL_SIZE))
    c1 = int((b[2] - C.ORIGIN_X) / C.CELL_SIZE)
    r0 = max(0, int((b[1] - C.ORIGIN_Y) / C.CELL_SIZE))
    r1 = int((b[3] - C.ORIGIN_Y) / C.CELL_SIZE)
    grid[r0:r1 + 1, c0:c1 + 1] = 1
planning_grid = binary_dilation(grid.astype(bool), iterations=C.SAFETY_MARGIN).astype(np.uint8)
safety_grid = binary_dilation(grid.astype(bool), iterations=1).astype(np.uint8)

def world_to_grid(x, y):
    return int((y - C.ORIGIN_Y) / C.CELL_SIZE), int((x - C.ORIGIN_X) / C.CELL_SIZE)

def grid_to_world(r, c):
    return (float(C.ORIGIN_X + c * C.CELL_SIZE + C.CELL_SIZE / 2),
            float(C.ORIGIN_Y + r * C.CELL_SIZE + C.CELL_SIZE / 2))

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