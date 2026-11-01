import os
import time
import numpy as np
import cv2
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import config as C
os.makedirs("results", exist_ok=True)
sim = RemoteAPIClient(C.HOST, C.PORT).require("sim")
h = sim.getObject("/PioneerP3DX/cam")
sim.startSimulation()
time.sleep(1.0)
try:
    b, res = sim.getVisionSensorImg(h)
    im = np.frombuffer(b, np.uint8).reshape(res[1], res[0], 3)
    im = cv2.flip(im, 0)
    im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
finally:
    sim.stopSimulation()
g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try:
    c, ids, _ = cv2.aruco.ArucoDetector(D, cv2.aruco.DetectorParameters()).detectMarkers(g)
except AttributeError:
    c, ids, _ = cv2.aruco.detectMarkers(g, D)
if ids is not None:
    cv2.aruco.drawDetectedMarkers(im, c, ids)
    print("found ids:", ids.flatten().tolist())
else:
    print("no marker found")
cv2.imwrite("results/cam_test.png", im)
print("saved results/cam_test.png")