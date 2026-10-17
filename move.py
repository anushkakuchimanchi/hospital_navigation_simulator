from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

client = RemoteAPIClient()
sim = client.require('sim')

# adjust these names to match YOUR scene hierarchy exactly
robot_handle = sim.getObject('/PioneerP3DX')
left_motor = sim.getObject('/PioneerP3DX/leftMotor')
right_motor = sim.getObject('/PioneerP3DX/rightMotor')

sim.startSimulation()

sim.setJointTargetVelocity(left_motor, 2.0)
sim.setJointTargetVelocity(right_motor, 2.0)

time.sleep(3)

sim.setJointTargetVelocity(left_motor, 0)
sim.setJointTargetVelocity(right_motor, 0)

sim.stopSimulation()

print("Done — check if the robot moved forward.")