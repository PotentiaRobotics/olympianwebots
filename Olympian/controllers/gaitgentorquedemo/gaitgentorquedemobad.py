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




ankleRollValue = 0.05
ankleRollValue2 = 0.06
def walk(value):

        left_ankle_roll.setPosition(0)
        right_ankle_roll.setPosition(-1 * ankleRollValue)

        # right_ankle_roll.setPosition(-1 * value/2)

        # torso_yaw.setPosition(value/4)
        # torso_roll.setPosition(0.1 + value/50)

        left_torso_pitch.setPosition(-1 * value)
        left_knee_pitch.setPosition(value * -1)

        right_arm_uzi.setPosition(-1 * value)
        left_arm_uzi.setPosition(value)

def walk2(value1, value2):

        left_ankle_roll.setPosition(ankleRollValue2)
        right_ankle_roll.setPosition(0)

        # left_ankle_roll.setPosition(value2/2)
        # right_ankle_roll.setPosition(-1 * value1/2)        

        # torso_yaw.setPosition(value/4)

        left_torso_pitch.setPosition(-1 * value1)
        left_knee_pitch.setPosition(value1 * -1)

        right_torso_pitch.setPosition( value2)
        right_knee_pitch.setPosition(value2)
        
        # torso_roll.setPosition(0.1 + value/50)
        #         # left_ankle_pitch.setPosition(value)

        right_arm_uzi.setPosition(-1 * value)
        left_arm_uzi.setPosition(value)

def walk3(value1, value2):

        

        left_ankle_roll.setPosition(0)
        right_ankle_roll.setPosition(-1 * ankleRollValue2)

        # left_ankle_roll.setPosition(value2/2)
        # right_ankle_roll.setPosition(-1 *value1/2)

        # torso_yaw.setPosition(value/4)

        left_torso_pitch.setPosition(-1 * value1)
        left_knee_pitch.setPosition(value1 * -1)

        right_torso_pitch.setPosition(value2)
        right_knee_pitch.setPosition(value2)

        right_arm_uzi.setPosition(-1 * value)
        left_arm_uzi.setPosition(value)
        # torso_roll.setPosition(0.1 + value/50)
        #         # left_ankle_pitch.setPosition(value)
        

first1 = True
first2 = True
first3 = True

value = 0
value2 = 0

# increment = 0.001
increment = 0.001

leftFoot = True

doneWithFirstStep = False

doneWithSecondStep = False

doneWithThirdStep = False

# coreValue = 0.25


fallingForward = True

wantActuation = True

numOfCrosses = 0
numOfCrosses2 = 0

coreValue = 0.25

tertiaryValue = 0.1

timeCompleted = 0

while supervisor.step(TIMESTEP) != -1:



        # if(time.time() - startTime >= 1 and first1 == True):


        #         right_shoulder_roll.setPosition(1.39)
        #         left_shoulder_roll.setPosition(-1.39)
   

        
        if(time.time() - startTime >= 2 and wantActuation == True):
                #torso_pitch.setPosition(0.25)
                torso_pitch.setPosition(0.3)
    
                if(doneWithFirstStep == False):
                        
                    value += increment
                    walk(value)
                    if(value >= coreValue ):

                        timeCompleted = time.time()

                        doneWithFirstStep = True
                            

                
                elif(doneWithFirstStep == True and doneWithSecondStep == False):

                    if (time.time() - timeCompleted >= 1):

                        value -= increment
                        value2 += increment

                        walk2(value, value2)

                        if(value2 >= coreValue):
                            
                            timeCompleted = time.time()
                            doneWithSecondStep = True
                            doneWithThirdStep = False

                            

                elif(doneWithFirstStep == True and doneWithThirdStep == False):

                    if(time.time() - timeCompleted >= 1):

                        value += increment
                        value2 -= increment

                        walk3(value, value2)

                        if(value >= coreValue):

                            timeCompleted = time.time()
                            doneWithThirdStep = True
                            doneWithSecondStep = False

                

                    






                # if(doneWithFirstStep == True and time.time() - startTime >= 5 and time.time() - startTime < 10):

                #         #break
                        
                #         # if(numOfCrosses2 == 3):
                #         #         leftFoot = True
                
                #         if(value >= coreValue):
                #                 leftFoot = False
                #         # elif(numOfCrosses == 3 and numOfCrosses2 != 3):
                #         #         leftFoot = False
                #         #         print("switching")


                #         elif(value <= 0):

                #                 value = 0.01
                #                 value2 = 0.25

                #                 # leftFoot = True
                #                 # break
                #         #         print("switching")
                #         if(leftFoot == True):
                        
                #                 value += increment
                #                 value2 -= increment
                #                 walk3(value, value2)

                #         elif(leftFoot == False):
                #                 # print("leftfoot false")

                #                 value -= increment
                #                 value2 += increment
                #                 walk2(value, value2)

                # elif(doneWithFirstStep == True and time.time() - startTime >= 10 and time.time() - startTime < 15):

                #         #break
                        
                #         # if(numOfCrosses2 == 3):
                #         #         leftFoot = True
                
                #         if(value >= coreValue):
                #                 # leftFoot = True
                #                 break
                #         # elif(numOfCrosses == 3 and numOfCrosses2 != 3):
                #         #         leftFoot = False
                #         #         print("switching")


                #         elif(value <= 0):

                #                 leftFoot = True
                #                 # break
                #         #         print("switching")
                #         if(leftFoot == True):
                        
                #                 value += increment
                #                 value2 -= increment
                #                 walk3(value, value2)

                #         elif(leftFoot == False):
                #                 # print("leftfoot false")

                #                 value -= increment
                #                 value2 += increment
                #                 walk2(value, value2)

                
                
                
                
