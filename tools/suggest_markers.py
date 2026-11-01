import math
import config as C
from mapgrid import grid, grid_to_world
from planner import resolve_delivery_points
P = math.pi / 2
O = {(1, 0): [P, 0, 0], (-1, 0): [-P, 0, 0], (0, 1): [0, -P, 0], (0, -1): [0, P, 0]}
ID = {"Ward": 1, "ICU": 2, "Pharmacy": 3, "Surgery": 4}
for n, (r, c) in resolve_delivery_points().items():
    x0, y0 = grid_to_world(r, c)
    b = None
    for (dr, dc), o in O.items():
        for k in range(1, 6):
            rr, cc = r + k * dr, c + k * dc
            if not (0 <= rr < grid.shape[0] and 0 <= cc < grid.shape[1]):
                break
            if grid[rr, cc]:
                d = (k - 0.5) * C.CELL_SIZE - 0.03
                if 0.3 <= d <= 1.0 and (b is None or abs(d - 0.7) < b[0]):
                    b = (abs(d - 0.7), d, x0 + d * dc, y0 + d * dr, o)
                break
    if b is None:
        print(f"# {n}: no wall within 1 m, use a thin post")
    else:
        print(f'    "{n}": ({ID[n]}, {b[2]:.3f}, {b[3]:.3f}, {b[4]}),   # {b[1]:.2f} m from stop')