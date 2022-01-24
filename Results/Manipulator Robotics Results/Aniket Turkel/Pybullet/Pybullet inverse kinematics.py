#IK line

import time
import pybullet
import numpy as np
import sympy as sp
import mpmath as m
from sympy import Symbol
import pybullet_data
import matplotlib.pyplot as plt

physics_client = pybullet.connect(pybullet.GUI) #For creating GUI
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

#planeID = pybullet.loadURDF("Downloads/base.urdf") #For plane
Robot = pybullet.loadURDF("/home/aniket/Downloads/pybullet-force-control-main/urdf/ur5.urdf", useFixedBase = 1)

numjoints = pybullet.getNumJoints(Robot)

pybullet.setGravity(0, 0, -9.8)
pybullet.setRealTimeSimulation(0)

#Orientation = pybullet.getQuaternionFromEuler([np.pi, 0., 0.]) #we are also specifying the orientation

pybullet.addUserDebugLine([0.3,0.3,0.6], [0.6,0.6,0.9], [0,1,0],55)

x,y,z = 0.3,0.3,0.6 #Line parameters initial points
del_k = 0.001

count = 0

numJoints = pybullet.getNumJoints(Robot)

revolute_joints = []
upper_limits = []
lower_limits = []
for i in range(numJoints):

    info = pybullet.getJointInfo(Robot, i)

    jointName = info[1]
    jointType = info[2]
    if (jointType == pybullet.JOINT_REVOLUTE):
        revolute_joints.append(i)

    lower_limits.append(-1* np.pi) 
    upper_limits.append( np.pi)

a1 , a2 , a3 = [] , [] , [] #For debugging

while (count<300) :
    
    count = count + 1

    x = x + del_k
    y = y + del_k
    z = z + del_k

    a1.append(x)
    a2.append(y)
    a3.append(z)

    #print(x,y,z)

    targetPositionsJoints = pybullet.calculateInverseKinematics(Robot, 7, [x,y,z])
    print(targetPositionsJoints)
        
    pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = targetPositionsJoints)
    
    pybullet.stepSimulation()
    time.sleep(1/25)

# plt.plot(a1)
# plt.plot(a2)
# plt.plot(a3)
# plt.show()

#pybullet.disconnect()



