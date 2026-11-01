import os
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import config as C
P = math.pi / 2
MARKS = {
    "Ward": (1, -1.220, 0.625, [0, 1.5707963267948966, 0]),   # 0.59 m from stop
    "ICU": (2, 2.220, 1.125, [0, -1.5707963267948966, 0]),   # 0.34 m from stop
    "Pharmacy": (3, -1.470, -1.625, [0, 1.5707963267948966, 0]),   # 0.34 m from stop
    "Surgery": (4, 1.220, -0.625, [0, -1.5707963267948966, 0]),   # 0.59 m from stop
}
Z = 0.25
S = 0.18
sim = RemoteAPIClient(C.HOST, C.PORT).require("sim")
for n, (i, x, y, o) in MARKS.items():
    f = os.path.abspath(f"markers/{n}_{i}.png").replace("\\", "/")
    h, t, r = sim.createTexture(f, 0, [S, S])
    sim.setObjectAlias(h, f"marker_{n}_{i}")
    sim.setObjectPosition(h, -1, [x, y, Z])
    sim.setObjectOrientation(h, -1, o)
    try:
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
    except Exception as e:
        print("param failed", e)
    try:
        sim.setObjectSpecialProperty(h, 0)
    except Exception:
        pass
    print("placed", n, i, f)
print("Now press Ctrl+S in CoppeliaSim to save the scene.")