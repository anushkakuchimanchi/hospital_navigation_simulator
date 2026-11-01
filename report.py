import os
import numpy as np
import matplotlib.pyplot as plt
import config as C
from mapgrid import grid, planning_grid, world_to_grid, grid_to_world, nearest_free_cell
from planner import astar
os.makedirs("results", exist_ok=True)

def extent():
    return [C.ORIGIN_X, C.ORIGIN_X + grid.shape[1] * C.CELL_SIZE,
            C.ORIGIN_Y, C.ORIGIN_Y + grid.shape[0] * C.CELL_SIZE]

def preview(points):
    s = nearest_free_cell(world_to_grid(*C.START_XY), planning_grid)
    print("Destination check from robot start:")
    for n, c in points.items():
        ok = astar(planning_grid, s, c) is not None
        x, y = grid_to_world(*c)
        print(f"  {n}: cell {c} world ({x:.2f},{y:.2f}) reachable: {'YES' if ok else 'NO'}")
    plt.figure(figsize=(7, 7))
    plt.imshow(planning_grid, cmap="Reds", origin="lower", extent=extent(), alpha=0.4, interpolation="nearest")
    plt.imshow(grid, cmap="gray_r", origin="lower", extent=extent(), alpha=0.7, interpolation="nearest")
    for n, c in points.items():
        x, y = grid_to_world(*c)
        plt.scatter(x, y, marker="*", s=180, c="gold", edgecolors="k", zorder=5)
        plt.annotate(n, (x, y), textcoords="offset points", xytext=(5, 5), fontsize=9)
    plt.scatter(*C.START_XY, marker="o", c="blue", zorder=5)
    plt.title("Destinations (red = safety margin)")
    plt.savefig("results/dest_preview.png", dpi=120)
    plt.close()
    print("Saved results/dest_preview.png")

def summary(results):
    print("\n================ DELIVERY SUMMARY ================")
    print(f"{'#':<3}{'Task':<34}{'To':<10}{'Prio':<6}{'Time(s)':<9}{'Off(m)':<8}{'Arrived':<9}{'Marker'}")
    for o, n, d, p, t, ok, off, st in results:
        print(f"{o:<3}{n:<34}{d:<10}{p:<6}{t:<9.1f}{off:<8.2f}{'YES' if ok else 'NO':<9}{st}")

def plot(legs, trace, points):
    if not legs:
        return
    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap="gray_r", origin="lower", extent=extent(), interpolation="nearest")
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