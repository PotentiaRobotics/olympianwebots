import math
from controller import Robot, Supervisor, Display
import time
supervisor = Supervisor()
global TIMESTEP
TIMESTEP = int(supervisor.getBasicTimeStep())
startTime = time.time()
robotNode = supervisor.getRoot() 
children = robotNode.getField("children")
robot = children.getMFNode(5)
right_torso_roll = supervisor.getDevice("Rev13")
left_torso_roll = supervisor.getDevice("Rev21")
right_ankle_roll = supervisor.getDevice("Rev16")
left_ankle_roll = supervisor.getDevice("Rev24")
for i in range(0, 1000):
    right_torso_roll.setPosition(-1 * i/1000)
    left_torso_roll.setPosition(-1 * i/1000)
    right_ankle_roll.setPosition(-1 * i/1000)
    left_ankle_roll.setPosition(-1 * i/1000)
    print(robot.getCenterOfMass())
    supervisor.step(TIMESTEP)