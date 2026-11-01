from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

client = RemoteAPIClient()
sim = client.require('sim')

robot = sim.getObject('/PioneerP3DX')
left_motor = sim.getObject('/PioneerP3DX/leftMotor')
right_motor = sim.getObject('/PioneerP3DX/rightMotor')

sim.startSimulation()

pos_before = sim.getObjectPosition(robot, -1)
ori_before = sim.getObjectOrientation(robot, -1)

sim.setJointTargetVelocity(left_motor, 2.0)
sim.setJointTargetVelocity(right_motor, 2.0)
time.sleep(2)
sim.setJointTargetVelocity(left_motor, 0)
sim.setJointTargetVelocity(right_motor, 0)

pos_after = sim.getObjectPosition(robot, -1)

sim.stopSimulation()

print(f"Heading (yaw) was: {ori_before[2]:.3f} rad")
print(f"Moved from ({pos_before[0]:.2f}, {pos_before[1]:.2f}) to ({pos_after[0]:.2f}, {pos_after[1]:.2f})")

import math
dx = pos_after[0] - pos_before[0]
dy = pos_after[1] - pos_before[1]
actual_direction = math.atan2(dy, dx)
print(f"Actual direction of travel: {actual_direction:.3f} rad")
print(f"Difference (offset): {actual_direction - ori_before[2]:.3f} rad")