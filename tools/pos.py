from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math

client = RemoteAPIClient()
sim = client.require('sim')

all_objects = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
for obj in all_objects:
    name = sim.getObjectAlias(obj)
    if 'wall' in name.lower():
        pos = sim.getObjectPosition(obj, -1)
        ori = sim.getObjectOrientation(obj, -1)
        # convert rotation around Z from radians to degrees for readability
        z_deg = math.degrees(ori[2])
        print(f"{name} | pos=({pos[0]:.2f}, {pos[1]:.2f}) | rot_z={z_deg:.1f}")