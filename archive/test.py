
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print("Connecting to CoppeliaSim on port 23000...", flush=True)

client = RemoteAPIClient(host='localhost', port=23000)

print("Client created", flush=True)

sim = client.require('sim')

print("Connected!", flush=True)
print("Simulation time:", sim.getSimulationTime())
