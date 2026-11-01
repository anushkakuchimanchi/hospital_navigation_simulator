import heapq
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import config as C
from task_input import ask_tasks
from planner import plan_route, resolve_delivery_points
from controller import follow_path, stop_robot
from report import preview, summary, plot
from verify import confirm

def main():
    tasks = ask_tasks(C.TASKS, C.DELIVERY_POINTS)
    points = resolve_delivery_points()
    preview(points)
    queue = []
    for i, t in enumerate(tasks):
        heapq.heappush(queue, (-t["priority"], -t.get("conf", 1.0), i, t))
    print("\nTasks waiting (in delivery order):")
    for _, _, _, t in sorted(queue, key=lambda q: (q[0], q[1], q[2])):
        print(f"  priority {t['priority']}  {t['name']} -> {t['dest']}")
    client = RemoteAPIClient(C.HOST, C.PORT)
    sim = client.require("sim")
    robot = sim.getObject("/PioneerP3DX")
    lm = sim.getObject("/PioneerP3DX/leftMotor")
    rm = sim.getObject("/PioneerP3DX/rightMotor")
    cam = sim.getObject("/PioneerP3DX/cam")
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
            neg_p, _, _, task = heapq.heappop(queue)
            order += 1
            pos_xy = sim.getObjectPosition(robot, -1)[:2]
            print(f"\n[{order}] {task['name']} (priority {-neg_p}) -> {task['dest']}")
            raw, smooth, final = plan_route(pos_xy, points[task["dest"]])
            if smooth is None:
                print("    [!] No path found. Skipping this task.")
                results.append((order, task["name"], task["dest"], -neg_p, 0.0, False, 0.0, "NO PATH"))
                continue
            print(f"    path: {len(raw)} cells, {len(smooth)} smooth points")
            t0 = time.time()
            reached, d = follow_path(sim, robot, lm, rm, smooth, trace=trace)
            el = time.time() - t0
            ok = reached or d <= C.NEAR_RADIUS
            if reached:
                print(f"    DELIVERED: {task['name']}")
            elif ok:
                print(f"    DELIVERED (nearby, {d:.2f} m off): {task['name']}")
            else:
                print(f"    FAILED (too far, {d:.2f} m): {task['name']}")
            st = "SKIPPED"
            if ok:
                time.sleep(C.DELIVERY_PAUSE)
                st = confirm(sim, robot, cam, lm, rm, task["dest"], str(order))
                print(f"    marker check: {st}")
            results.append((order, task["name"], task["dest"], -neg_p, el, ok, d, st))
            legs.append({"name": task["name"], "smooth": smooth})
        print(f"\nQueue empty, returning to dock ({C.DOCK})")
        pos_xy = sim.getObjectPosition(robot, -1)[:2]
        raw, smooth, final = plan_route(pos_xy, points[C.DOCK])
        if smooth is not None:
            follow_path(sim, robot, lm, rm, smooth, trace=trace)
            legs.append({"name": "Return to dock", "smooth": smooth})
    finally:
        stop_robot(sim, lm, rm)
        if started_here:
            try:
                sim.stopSimulation()
            except Exception:
                pass
    summary(results)
    plot(legs, trace, points)
    print("Queue run finished.")

if __name__ == "__main__":
    main()