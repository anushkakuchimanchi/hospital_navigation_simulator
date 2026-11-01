from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import config as C
sim = RemoteAPIClient(C.HOST, C.PORT).require("sim")
ID = {"Ward": 1, "ICU": 2, "Pharmacy": 3, "Surgery": 4}
out = []
for n, i in ID.items():
    try:
        h = sim.getObject(f"/marker_{n}_{i}")
        p = sim.getObjectPosition(h, -1)
        out.append(f'"{n}": ({i}, {p[0]:.3f}, {p[1]:.3f})')
        print(n, "z =", round(p[2], 2))
    except Exception as e:
        print(n, "NOT FOUND in scene:", e)
print("MARKERS = {" + ", ".join(out) + "}")