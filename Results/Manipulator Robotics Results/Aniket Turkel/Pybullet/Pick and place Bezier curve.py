#Pick and place Bezier curve

import time
import pybullet
import numpy as np
import sympy as sp
import mpmath as m
from sympy import Symbol
import pybullet_data
import matplotlib.pyplot as plt

box_spawn_pos = [0.32,-0.568,0.85]

# Obtaining path function

u = Symbol('u')

p0 = np.array([0.3,-0.5,1.2]) #Point 1
#p0 = np.array([box_spawn_pos[0]+0.02,box_spawn_pos[1]+0.068,box_spawn_pos[2]+0.35]) #Initial point of the path
p1 = np.array([0,-0.5,1.5]) #Point 2, Control point
p2 = np.array([-0.3,-0.5,1.2]) #point 3
#p2 = np.array([-0.3,-0.3,1.2]) #Final position

exprx = (1-u)**2 * p0[0] + 2*u*(1-u)* p1[0] + u**2 * p2[0]
expry = (1-u)**2 * p0[1] + 2*u*(1-u)* p1[1] + u**2 * p2[1]
exprz = (1-u)**2 * p0[2] + 2*u*(1-u)* p1[2] + u**2 * p2[2]

# -x-x-x-

sp.pprint(exprx)
sp.pprint(expry)
sp.pprint(exprz)

# -x-x-x-

physics_client = pybullet.connect(pybullet.GUI) #For creating GUI
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

plane = pybullet.loadURDF("plane.urdf") #For plane
#Robot = pybullet.loadURDF("/home/aniket/Downloads/pybullet-force-control-main/urdf/ur5.urdf", useFixedBase = 1)
#gripper = pybullet.loadURDF("/home/aniket/Documents/pybullet-ur5-equipped-with-robotiq-140/urdf/robotiq_140.urdf", useFixedBase = 0)
table = pybullet.loadURDF("/home/aniket/Documents/pybullet-ur5-equipped-with-robotiq-140/urdf/objects/table.urdf",useFixedBase = 1,basePosition = [0,0.5,0])
block = pybullet.loadURDF("/home/aniket/Documents/pybullet-ur5-equipped-with-robotiq-140/urdf/objects/block.urdf", basePosition = box_spawn_pos,useFixedBase = 0)
#stand = pybullet.loadURDF("/home/aniket/Downloads/Manipulator robotics/urdf-20220109T122042Z-001/urdf/objects/ur5_stand.urdf", useFixedBase = 1)
#Robot = pybullet.loadURDF(r"pybullet-ur5-equipped-with-robotiq-140/urdf/ur5_robotiq_140.urdf", useFixedBase = 1)
Robot = pybullet.loadURDF("/home/aniket/Documents/pybullet-ur5-equipped-with-robotiq-140/urdf/ur5_robotiq_140_backup.urdf", useFixedBase = 1)

numjoints = pybullet.getNumJoints(Robot)
print(numjoints)

pybullet.setGravity(0, 0, -98)
pybullet.setRealTimeSimulation(0)

numJoints = pybullet.getNumJoints(Robot)

orientation = pybullet.getQuaternionFromEuler([m.pi()/2, m.pi(), m.pi()/2])
print(orientation)

revolute_joints = [] #Addition to classify different joint types
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

count = 0

a1,a2,a3 = [],[],[]

time.sleep(5)

#Hovering over box

targetPositionsJoints = pybullet.calculateInverseKinematics(Robot, 8, p0, targetOrientation = orientation)
pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = targetPositionsJoints)
for _ in range(100):
    pybullet.stepSimulation()
    time.sleep(1/100)

#print(targetPositionsJoints)

#Open gripper
#joint_positions = pybullet.getJointStates(Robot, range(revolute_joints))
pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = [targetPositionsJoints[0],targetPositionsJoints[1],targetPositionsJoints[2],targetPositionsJoints[3],targetPositionsJoints[4],targetPositionsJoints[5],-1*m.pi(),0,-1*m.pi(),0,targetPositionsJoints[10],targetPositionsJoints[11]]) #Gripper angles to open
for _ in range(100):
    pybullet.stepSimulation()
    time.sleep(1/100)

#Lower the gripper in line
del_k = 0.01
x,y = 0.3,-0.5
count = 0
for i in range(1, 101):
    z = p0[2] - 0.14*del_k*i
    targetPositionsJoints = pybullet.calculateInverseKinematics(Robot, 7, [x,y,z])
    print(targetPositionsJoints)
    pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = targetPositionsJoints)

    pybullet.stepSimulation()
    time.sleep(1/100)


#Grab
#targetPositionsJoints[6],targetPositionsJoints[7],targetPositionsJoints[8],targetPositionsJoints[9]= 0.5,0.5,0.5,0.5 #Gripper angles to close
#joint_positions = pybullet.getJointStates(Robot, range(revolute_joints))
pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = [targetPositionsJoints[0],targetPositionsJoints[1],targetPositionsJoints[2],targetPositionsJoints[3],targetPositionsJoints[4],targetPositionsJoints[5],0.5,-2.5,0.5,-2.5,targetPositionsJoints[10],targetPositionsJoints[11]]) #For moving the joints
for _ in range(100):
    pybullet.stepSimulation()
    time.sleep(1/100)

#Lift in line form
del_k = 0.01
x,y = p0[0],p0[1]
for i in range(0, 100):
    z = p0[2]-0.1 + 0.1*del_k*i
    targetPositionsJoints = pybullet.calculateInverseKinematics(Robot, 7, [x,y,z])
    pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = targetPositionsJoints)
    
    pybullet.stepSimulation()
    time.sleep(1/100)

#The curve path

tempx,tempy,tempz = p0[0],p0[1],p0[2]
while (count<=100) :

    x = exprx.subs(u,count/100).evalf()
    y = expry.subs(u,count/100).evalf()
    z = exprz.subs(u,count/100).evalf()

    #pybullet.addUserDebugLine([tempx,tempy,tempz], [x,y,z], [0,1,0], 5)
    
    count = count + 1

    tempx,tempy,tempz = x,y,z

    a1.append(x)
    a2.append(y)
    a3.append(z)

    targetPositionsJoints = pybullet.calculateInverseKinematics(Robot, 7, [x,y,z])
        
    pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = targetPositionsJoints)
    
    pybullet.stepSimulation()
    time.sleep(1/100)

# # Orienting the end effector 
# targetPositionsJoints = pybullet.calculateInverseKinematics(Robot, 7, [x,y,z], targetOrientation = orientation)
# pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = targetPositionsJoints)

# pybullet.stepSimulation()
# time.sleep(1/15)

#Lower in line form
del_k = 0.01
x,y = p2[0],p2[1]
for i in range(0, 100):
    z = p2[2] - 0.1*del_k*i
    targetPositionsJoints = pybullet.calculateInverseKinematics(Robot, 7, [x,y,z])
    pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = targetPositionsJoints)
    
    pybullet.stepSimulation()
    time.sleep(1/100)

#Leave box/open gripper
#targetPositionsJoints[6],targetPositionsJoints[7],targetPositionsJoints[8],targetPositionsJoints[9]= -1*m.pi(),m.pi(),-1*m.pi(),m.pi() #Gripper angles to open
pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = [targetPositionsJoints[0],targetPositionsJoints[1],targetPositionsJoints[2],targetPositionsJoints[3],targetPositionsJoints[4],targetPositionsJoints[5],-1*m.pi(),m.pi(),-1*m.pi(),m.pi(),targetPositionsJoints[10],targetPositionsJoints[11]]) #For moving the joints

pybullet.stepSimulation()
time.sleep(1/100)

#Rise in line form
del_k = 0.01
x,y = p2[0],p2[1]
for i in range(0, 100):
    z = p2[2] + 0.5*del_k*i
    targetPositionsJoints = pybullet.calculateInverseKinematics(Robot, 7, [x,y,z])
    pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = targetPositionsJoints)
    
    pybullet.stepSimulation()
    time.sleep(1/100)

# plt.plot(a1)
# plt.plot(a2)
# plt.plot(a3)

# plt.show()

#pybullet.disconnect()