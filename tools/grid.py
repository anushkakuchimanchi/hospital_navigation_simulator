import numpy as np
import re

# your floor spans roughly -2.5 to 2.5 in both X and Y
ORIGIN_X = -2.5
ORIGIN_Y = -2.5
CELL_SIZE = 0.25
GRID_SIZE = 20  # 5m / 0.25m

grid = np.zeros((GRID_SIZE, GRID_SIZE))

# name -> length in meters
def get_length(name):
    if '200cm' in name: return 2.0
    if '100cm' in name: return 1.0
    if '50cm' in name: return 0.5
    if '25cm' in name: return 0.25
    if 'End' in name: return 0.25
    if 'L' in name: return 0.5  # corner piece, treat as small block
    return 0.25

# paste your wall data here: (name, x, y, rot_z)
walls = [
    ("80cmHighWall200cm", 2.43, 1.30, -0.0),
    ("80cmHighWallL", 2.40, 2.40, -90.0),
    ("80cmHighWall200cm", 2.43, -0.80, -0.0),
    ("80cmHighWall50cm", 2.43, -2.05, -0.0),
    ("80cmHighWallL", 2.40, -2.40, -180.0),
    ("80cmHighWall25cm", 0.15, -0.60, -0.0),
    ("80cmHighWall100cm", 0.15, -1.18, -0.0),
    ("80cmHighWall100cm", 1.85, -0.55, -90.0),
    ("80cmHighWall50cm", 0.15, -1.93, -0.0),
    ("80cmHighWall25cm", 0.15, -2.30, -0.0),
    ("80cmHighWallEnd", 0.15, -2.45, -0.0),
    ("80cmHighWall25cm", 0.78, 2.37, -0.0),
    ("80cmHighWall100cm", 0.78, 1.75, -0.0),
    ("80cmHighWall50cm", 0.78, 1.00, -0.0),
    ("80cmHighWall25cm", 0.78, 0.62, -0.0),
    ("80cmHighWall25cm", -2.42, -0.58, -0.0),
    ("80cmHighWall100cm", -2.42, -1.20, -0.0),
    ("80cmHighWall50cm", -2.42, -1.95, -0.0),
    ("80cmHighWall25cm", -2.42, -2.33, -0.0),
    ("80cmHighWallEnd", -2.42, -2.48, -0.0),
    ("80cmHighWall25cm", -2.42, 2.37, -0.0),
    ("80cmHighWall100cm", -2.42, 1.75, -0.0),
    ("80cmHighWall50cm", -2.42, 1.00, -0.0),
    ("80cmHighWall25cm", -2.42, 0.62, -0.0),
    ("80cmHighWallEnd", -2.42, 0.47, -0.0),
    ("80cmHighWall25cm", 1.15, 0.57, 90.0),
    ("80cmHighWall25cm", 0.95, 0.57, 90.0),
    ("80cmHighWall50cm", -0.17, -0.55, 90.0),
    ("80cmHighWall50cm", -2.10, -0.53, 90.0),
    ("80cmHighWall100cm", -1.85, 0.52, 90.0),
    ("80cmHighWall50cm", 0.45, 0.57, -90.0),
    ("80cmHighWall25cm", 0.08, 0.57, -90.0),
    ("80cmHighWall25cm", 2.43, 0.27, -0.0),
]

def world_to_grid(x, y):
    col = int((x - ORIGIN_X) / CELL_SIZE)
    row = int((y - ORIGIN_Y) / CELL_SIZE)
    return row, col

for name, x, y, rot in walls:
    length = get_length(name)
    horizontal = abs(rot) in [90, -90, 270]# extends along X
    steps = max(int(length / CELL_SIZE), 1)

    for i in range(-steps//2, steps//2 + 1):
        if horizontal:
            wx = x + i * CELL_SIZE
            wy = y
        else:
            wx = x
            wy = y + i * CELL_SIZE

        row, col = world_to_grid(wx, wy)
        if 0 <= row < GRID_SIZE and 0 <= col < GRID_SIZE:
            grid[row, col] = 1

np.save('grid.npy', grid)
print("Grid built and saved to grid.npy")
print(grid.astype(int))

import matplotlib.pyplot as plt

grid = np.load('grid.npy')
plt.imshow(grid, cmap='gray_r', origin='lower')
plt.title('Wall Grid')
plt.grid(True, alpha=0.3)
plt.savefig('grid_preview.png')
print("Saved grid_preview.png — open it to check the layout")