import math
import time
import numpy as np
import cv2
import config as C
from controller import get_heading, stop_robot
D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
DET = cv2.aruco.ArucoDetector(D, cv2.aruco.DetectorParameters())

def grab(sim, cam):
    b, r = sim.getVisionSensorImg(cam)
    im = np.frombuffer(b, np.uint8).reshape(r[1], r[0], 3)
    return cv2.cvtColor(cv2.flip(im, 0), cv2.COLOR_RGB2BGR)

def look(im):
    c, i, _ = DET.detectMarkers(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY))
    if i is None:
        return [], c, i
    return i.flatten().tolist(), c, i

def turn_to(sim, robot, lm, rm, ang, limit=6.0):
    t0 = time.time()
    while time.time() - t0 < limit:
        e = math.atan2(math.sin(ang - get_heading(sim, robot)), math.cos(ang - get_heading(sim, robot)))
        if abs(e) < 0.08:
            break
        w = C.SCAN_TURN_SPEED
        if e < 0:
            w = -w
        sim.setJointTargetVelocity(lm, -w)
        sim.setJointTargetVelocity(rm, w)
        time.sleep(C.SIM_STEP_SLEEP)
    stop_robot(sim, lm, rm)
    time.sleep(0.15)

def watch(sim, cam, want, sec, other):
    t0 = time.time()
    run = 0
    last = None
    while time.time() - t0 < sec:
        im = grab(sim, cam)
        ids, c, i = look(im)
        if want in ids:
            run += 1
            last = (im, c, i)
            if run >= C.NEED_FRAMES:
                return True, last
        else:
            run = 0
            for k in ids:
                other.add(k)
        time.sleep(0.05)
    return False, last

def confirm(sim, robot, cam, lm, rm, dest, tag):
    want, mx, my = C.MARKERS[dest]
    p = sim.getObjectPosition(robot, -1)
    base = math.atan2(my - p[1], mx - p[0])
    other = set()
    for off in (0.0, 0.5, -0.5, 1.0, -1.0):
        turn_to(sim, robot, lm, rm, base + off)
        ok, last = watch(sim, cam, want, C.WATCH_SEC if off == 0.0 else 1.0, other)
        if ok:
            im, c, i = last
            cv2.aruco.drawDetectedMarkers(im, c, i)
            cv2.imwrite(f"results/confirm_{tag}.png", im)
            return "CONFIRMED"
    others = sorted(other - {want})
    p = sim.getObjectPosition(robot, -1)
    dist = math.hypot(mx - p[0], my - p[1])
    print(f"    robot ({p[0]:.2f},{p[1]:.2f}) marker ({mx:.2f},{my:.2f}) distance {dist:.2f} m")
    cv2.imwrite(f"results/notseen_{tag}.png", grab(sim, cam))
    if others:
        return f"WRONG {others}"
    return "NOT SEEN"