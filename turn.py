from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time, math

client = RemoteAPIClient()
sim = client.require('sim')

robot = sim.getObject('/PioneerP3DX')
left_motor = sim.getObject('/PioneerP3DX/leftMotor')
right_motor = sim.getObject('/PioneerP3DX/rightMotor')

sim.startSimulation()

pos_start = sim.getObjectPosition(robot, -1)
print(f"Start pos: {pos_start[0]:.2f}, {pos_start[1]:.2f}")

# spin in place — left backward, right forward
sim.setJointTargetVelocity(left_motor, -1.0)
sim.setJointTargetVelocity(right_motor, 1.0)

for i in range(20):
    time.sleep(0.2)
    pos = sim.getObjectPosition(robot, -1)
    print(f"t={i*0.2:.1f}s pos=({pos[0]:.2f}, {pos[1]:.2f})")

sim.setJointTargetVelocity(left_motor, 0)
sim.setJointTargetVelocity(right_motor, 0)
sim.stopSimulation()