import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import config as C
P = math.pi / 2
sim = RemoteAPIClient(C.HOST, C.PORT).require("sim")
r = sim.getObject("/PioneerP3DX")
h = sim.getObject("/PioneerP3DX/cam")
rz = sim.getObjectPosition(r, -1)[2]
sim.setObjectPosition(h, r, [0.3, 0.0, 0.25 - rz])
sim.setObjectOrientation(h, r, [P, P, 0])
print("camera fixed")