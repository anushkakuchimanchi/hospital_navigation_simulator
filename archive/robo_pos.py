from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require('sim')

robot = sim.getObject('/PioneerP3DX')
pos = sim.getObjectPosition(robot, -1)
print(f"Robot world position: x={pos[0]:.3f}, y={pos[1]:.3f}")