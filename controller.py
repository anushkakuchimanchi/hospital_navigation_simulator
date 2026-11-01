import math
import time
import numpy as np
import config as C

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
    la = min(C.MAX_LOOKAHEAD, max(C.MIN_LOOKAHEAD, C.LOOKAHEAD_DISTANCE))
    while time.time() - t0 < C.MAX_RUN_SECONDS:
        pos = np.array(sim.getObjectPosition(robot, -1)[:2], dtype=float)
        if trace is not None:
            trace.append(pos.copy())
        h = get_heading(sim, robot)
        gd = float(np.linalg.norm(goal - pos))
        best = min(best, gd)
        if gd <= C.FINAL_RADIUS:
            reached = True
            break
        if best < C.NEAR_RADIUS and gd > best + 0.05:
            break
        tgt, idx = find_lookahead_point(pts, pos, idx, la)
        dx, dy = tgt[0] - pos[0], tgt[1] - pos[1]
        a = math.atan2(dy, dx) - h
        a = math.atan2(math.sin(a), math.cos(a))
        if gd > 0.4:
            if turning and abs(a) < C.TURN_OFF:
                turning = False
            elif not turning and abs(a) > C.TURN_ON:
                turning = True
        else:
            turning = False
        if turning:
            w = C.TURN_SPEED if a > 0 else -C.TURN_SPEED
            sim.setJointTargetVelocity(lm, -w)
            sim.setJointTargetVelocity(rm, w)
            time.sleep(C.SIM_STEP_SLEEP)
            continue
        ly = -math.sin(h) * dx + math.cos(h) * dy
        dsq = max(dx * dx + dy * dy, 1e-8)
        k = 2.0 * ly / dsq
        sp = C.BASE_SPEED / (1.0 + 0.7 * abs(k))
        if gd < 0.6:
            sp *= max(0.25, gd / 0.6)
        w = sp * k
        l = float(np.clip(sp - w * C.WHEEL_BASE / 2, -C.MAX_WHEEL_SPEED, C.MAX_WHEEL_SPEED))
        r = float(np.clip(sp + w * C.WHEEL_BASE / 2, -C.MAX_WHEEL_SPEED, C.MAX_WHEEL_SPEED))
        sim.setJointTargetVelocity(lm, l)
        sim.setJointTargetVelocity(rm, r)
        time.sleep(C.SIM_STEP_SLEEP)
    stop_robot(sim, lm, rm)
    time.sleep(0.15)
    pos = np.array(sim.getObjectPosition(robot, -1)[:2], dtype=float)
    d = float(np.linalg.norm(goal - pos))
    if reached:
        print(f"    goal reached, distance={d:.3f} m")
    else:
        print(f"    [!] Stopped short, {d:.2f} m from destination.")
    return reached, d