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


def squatDown():
    i = 0
    while(i < 0.7):
        i += 0.005

        left_torso_pitch.setPosition(i*-1)
        right_torso_pitch.setPosition(i )

        left_knee_pitch.setPosition(i * -2)
        right_knee_pitch.setPosition(i * 2)

        left_ankle_pitch.setPosition(i * -1)
        right_ankle_pitch.setPosition(i)

       
        supervisor.step(TIMESTEP)


def squat():
    i = 0
    while(i < 0.7):
        i += 0.005

        left_torso_pitch.setPosition(i*-1)
        right_torso_pitch.setPosition(i )

        left_knee_pitch.setPosition(i * -2)
        right_knee_pitch.setPosition(i * 2)

        left_ankle_pitch.setPosition(i * -1)
        right_ankle_pitch.setPosition(i)

       
        supervisor.step(TIMESTEP)

    while(i > 0):
        i -= 0.005

        left_torso_pitch.setPosition(i*-1)
        right_torso_pitch.setPosition(i )

        left_knee_pitch.setPosition(i * -2)
        right_knee_pitch.setPosition(i * 2)

        left_ankle_pitch.setPosition(i * -1)
        right_ankle_pitch.setPosition(i)

       
        supervisor.step(TIMESTEP)


testingTorque = 20 #nm
left_torso_pitch.setAvailableTorque(testingTorque)
right_torso_pitch.setAvailableTorque(testingTorque)

left_knee_pitch.setAvailableTorque(testingTorque)
right_knee_pitch.setAvailableTorque(testingTorque)

left_ankle_pitch.setAvailableTorque(testingTorque)
right_ankle_pitch.setAvailableTorque(testingTorque)
        
for i in range(0,5):
    squat()

# stepLeftFoot()