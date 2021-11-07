from controller import Robot, Supervisor, Display

import time

from shapely.geometry.polygon import Polygon
from shapely.geometry.point import Point

# import tinyik #https://github.com/lanius/tinyik

import numpy as np


import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import ikpy
import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
from ikpy.urdf.utils import get_urdf_tree

from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error

import sympy as sym


#robot - wb_supervisor_node
#supervisor - wb_robot
#so basically supervisor is the robot and robot is the supervisor
#hooray for poor nomenclature
import math
#supervisor.getUrdf() - get urdf from proto (wow)
supervisor = Supervisor()

global TIMESTEP
TIMESTEP = int(supervisor.getBasicTimeStep())
startTime = time.time()

robotNode = supervisor.getRoot() 
children = robotNode.getField("children")
robot = children.getMFNode(5)

right_shoulder_roll = supervisor.getDevice("Rev4")
left_shoulder_roll = supervisor.getDevice("Rev8")

right_arm_uzi = supervisor.getDevice("Rev3")
left_arm_uzi = supervisor.getDevice("Rev7")

torso_pitch = supervisor.getDevice("Rev1")
torso_roll = supervisor.getDevice("Rev2")
# torso_yaw = supervisor.getDevice("Rev")

right_torso_pitch = supervisor.getDevice("Rev12")
left_torso_pitch = supervisor.getDevice("Rev20")

right_torso_roll = supervisor.getDevice("Rev13")
left_torso_roll = supervisor.getDevice("Rev21")

right_knee_pitch = supervisor.getDevice("Rev15")
left_knee_pitch = supervisor.getDevice("Rev23")


right_ankle_pitch = supervisor.getDevice("Rev19")
left_ankle_pitch = supervisor.getDevice("Rev25")

right_ankle_roll = supervisor.getDevice("Rev16")

left_ankle_roll = supervisor.getDevice("Rev24")

def shiftTorsoLeft():
    value = 0
    print(robot.getCenterOfMass())
    while(robot.getCenterOfMass()[0] < 0.15):
        print(robot.getCenterOfMass())
        value += 0.005
        right_torso_roll.setPosition(value)
        left_torso_roll.setPosition(value)

        right_ankle_roll.setPosition(value)
        left_ankle_roll.setPosition(value)

        supervisor.step(TIMESTEP)
    
    return value

def stepRightFoot():
    value = 0
    # print(robot.getCenterOfMass())
    while(value < 0.1):
        # print(robot.getCenterOfMass())
        value += 0.005
        right_torso_pitch.setPosition(value)
        right_knee_pitch.setPosition(value)
        right_ankle_pitch.setPosition(value * -1)

        left_torso_pitch.setPosition(value)
        left_ankle_pitch.setPosition(value * -1)
        left_knee_pitch.setPosition(value * -1)
        # right_ankle_pitch.setPosition(value * -1)

        supervisor.step(TIMESTEP)

def shiftTorsoRight(value):
    
    print(robot.getCenterOfMass())
    while(robot.getCenterOfMass()[0] > 0.05):
        print(robot.getCenterOfMass())
        value -= 0.005
        right_torso_roll.setPosition(value)
        left_torso_roll.setPosition(value)

        right_ankle_roll.setPosition(value)
        left_ankle_roll.setPosition(value)

        supervisor.step(TIMESTEP)

def stepLeftFoot():
    value = 0.1
    # print(robot.getCenterOfMass())
    while(value > -0.1):
        # print(robot.getCenterOfMass())
        value -= 0.001
        right_torso_pitch.setPosition(value)
        right_knee_pitch.setPosition(value * -1)
        right_ankle_pitch.setPosition(value * -1)

        left_torso_pitch.setPosition(value)
        left_ankle_pitch.setPosition(value * -1)
        left_knee_pitch.setPosition(value )
        # right_ankle_pitch.setPosition(value * -1)

        supervisor.step(TIMESTEP)

torso_pitch.setPosition(0.3)

value = shiftTorsoLeft()
stepRightFoot()
shiftTorsoRight(value)
# stepLeftFoot()