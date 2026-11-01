
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient('localhost', 23001)
sim = client.require('sim')

print("Connected to CoppeliaSim!")

for name in ["/PioneerP3DX", "/leftMotor", "/rightMotor"]:
    try:
        handle = sim.getObject(name)
        print(f"FOUND: {name}, handle = {handle}")
    except Exception as e:
        print(f"NOT FOUND: {name} -> {e}")
