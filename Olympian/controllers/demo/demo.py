import math
from controller import Robot, Supervisor, Display
import time

import ikpy
import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
from ikpy.urdf.utils import get_urdf_tree


import matplotlib.pyplot as plt


supervisor = Supervisor()

global TIMESTEP
TIMESTEP = int(supervisor.getBasicTimeStep())
startTime = time.time()

robotNode = supervisor.getRoot() 
children = robotNode.getField("children")
robot = children.getMFNode(5)

lThighPitch = supervisor.getDevice("Rev20")
lKneePitch = supervisor.getDevice("Rev23")

lArmRoll = supervisor.getDevice("Rev8")

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

print(lArmRoll.getAvailableTorque())
# lArmRoll.setAvailableTorque(10)
# lArmRoll.setTorque(-3)
print(lArmRoll.getAvailableTorque())
# lArmRoll.setPosition(-0.5*math.pi)

left_torso_pitch.setAvailableTorque(10)
right_torso_pitch.setAvailableTorque(10)

right_torso_pitch.setPosition(0.5)

# print(lThighPitch.getAvailableTorque())
# print(lKneePitch.getAvailableTorque())

# lThighPitch.setAvailableTorque(50)
# lKneePitch.setAvailableTorque(50)
# print(lThighPitch.getAvailableTorque())
# print(lKneePitch.getAvailableTorque())
# lThighPitch.setPosition(-0.2)
# lKneePitch.setPosition(-0.2)

# f = open("olympian.urdf", "w")
# f.write(supervisor.getUrdf())
# f.close()
# get_urdf_tree("olympian.urdf", root_element="base_link", out_image_path="./out/olympian")

# lLegChain = ikpy.chain.Chain.from_urdf_file("olympian.urdf" ,
# # active_links_mask=[False,True,False,True,True,False,False,False],
# base_elements=["base_link", "Rev20"]
# )


# fig, ax = plot_utils.init_3d_figure()
# lLegChain.plot(lLegChain.inverse_kinematics_frame(lLegChain.forward_kinematics([0] * 7)), ax)

# print(lLegChain)

# plt.show()

# device0 = supervisor.getDeviceByIndex(0)
# device0.setPosition(0.3)

# device6 = supervisor.getDeviceByIndex(6)
# device6.setPosition(0.3)

# device12 = supervisor.getDeviceByIndex(14)
# device12.setPosition(-0.3)

#TODO GET IKPY IN HERE AND LOOK AT UML DIAGRAM AND FIGURE OUT DEVICE NAMES