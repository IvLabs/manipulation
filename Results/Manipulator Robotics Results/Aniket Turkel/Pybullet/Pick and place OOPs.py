#Pick and place using OOPs

import time
import pybullet
import numpy as np
import sympy as sp
import mpmath as m
from sympy import Symbol
import pybullet_data
import matplotlib.pyplot as plt

box_spawn_pos = [0.32,-0.54,0.85]

#p0 = np.array([0.3,-0.5,1.2]) #Point 1
p0 = np.array([box_spawn_pos[0]-0.02,box_spawn_pos[1]+0.04,box_spawn_pos[2]+0.35]) #Initial point of the path
p1 = np.array([0,-0.5,1.5]) #Point 2, Control point
p2 = np.array([-0.3,-0.6,1.2]) #point 3

class manipulator:
    def __init__(self, spawn_pos, initial_pos, control_point, final_pos):
        self.initial_pos = initial_pos
        self.control_point = control_point
        self.final_pos = final_pos

def bezier_function(p0,p1,p2):
    u = Symbol('u')

    p0 = np.array([0.3,-0.5,1.2]) #Point 1
    #p0 = np.array([box_spawn_pos[0]+0.02,box_spawn_pos[1]+0.068,box_spawn_pos[2]+0.35]) #Initial point of the path
    p1 = np.array([0,-0.5,1.5]) #Point 2, Control point
    p2 = np.array([-0.3,-0.6,1.2]) #point 3
    #p2 = np.array([-0.3,-0.3,1.2]) #Final position

    exprx = (1-u)**2 * p0[0] + 2*u*(1-u)* p1[0] + u**2 * p2[0]
    expry = (1-u)**2 * p0[1] + 2*u*(1-u)* p1[1] + u**2 * p2[1]
    exprz = (1-u)**2 * p0[2] + 2*u*(1-u)* p1[2] + u**2 * p2[2]

    return(exprx,expry,exprz)

#manipulator.bezier_path_func = bezier_function()

def initial_setup(p0):
    #To avoid collision with obstacles

    lower_lim = [-1*m.pi(),-2*m.pi(),-1*m.pi(),-1*m.pi(),-1*m.pi(),-1*m.pi(),-1*m.pi(),-1*m.pi(),-1*m.pi(),-1*m.pi(),-1*m.pi(),-1*m.pi()]
    upper_lim = [m.pi(),0,m.pi(),m.pi(),m.pi(),m.pi(),m.pi(),m.pi(),m.pi(),m.pi(),m.pi(),m.pi()]
    joint_range = revolute_joints
    rest_pos = [0,-1*m.pi(),0,0,0,0,0,0,0,0,0,0]

    # Setting initial position so that it does not collide with the table

    targetPositionsJoints = [0,-1*m.pi()/2,m.pi()/2,0,0,0,0,0,0,0,0,0]
    pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = targetPositionsJoints)
    for _ in range(100):
        pybullet.stepSimulation()
        time.sleep(1/100)

    #Hovering over box

    targetPositionsJoints = pybullet.calculateInverseKinematics(Robot, 8, p0, targetOrientation = orientation, lowerLimits = lower_lim, upperLimits = upper_lim, jointRanges = joint_range, restPoses =rest_pos)
    pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = targetPositionsJoints)
    for _ in range(100):
        pybullet.stepSimulation()
        time.sleep(1/100)

    return(targetPositionsJoints)    

def open_gripper(targetPositionsJoints):
    #Open gripper
    pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = [targetPositionsJoints[0],targetPositionsJoints[1],targetPositionsJoints[2],targetPositionsJoints[3],targetPositionsJoints[4],targetPositionsJoints[5],-1*m.pi(),0,-1*m.pi(),0,targetPositionsJoints[10],targetPositionsJoints[11]]) #Gripper angles to open
    for _ in range(100):
        pybullet.stepSimulation()
        time.sleep(1/100)

def grab(targetPositionsJoints):
    #Grab
    pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = [targetPositionsJoints[0],targetPositionsJoints[1],targetPositionsJoints[2],targetPositionsJoints[3],targetPositionsJoints[4],targetPositionsJoints[5],0.5,-2.5,0.5,-2.5,targetPositionsJoints[10],targetPositionsJoints[11]]) #For moving the joints
    for _ in range(100):
        pybullet.stepSimulation()
        time.sleep(1/100)

def curve_path(p0,exprx,expry,exprz):
    #The curve path

    count = 0

    u = Symbol('u')

    tempx,tempy,tempz = p0[0],p0[1],p0[2]
    while (count<=100) :

        x = exprx.subs(u,count/100).evalf()
        y = expry.subs(u,count/100).evalf()
        z = exprz.subs(u,count/100).evalf()
    
        count = count + 1

        tempx,tempy,tempz = x,y,z

        targetPositionsJoints = pybullet.calculateInverseKinematics(Robot, 7, [x,y,z])
        
        pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = targetPositionsJoints)
    
        pybullet.stepSimulation()
        time.sleep(1/100)

def line_path_descend(p):
    #Lower the gripper in line
    del_k = 0.01
    x,y = p[0],p[1]
    for i in range(1, 100):
        z = p[2] - 0.1*del_k*i
        targetPositionsJoints = pybullet.calculateInverseKinematics(Robot, 7, [x,y,z])
        print(targetPositionsJoints)
        pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = targetPositionsJoints)

        pybullet.stepSimulation()
        time.sleep(1/100)

        return(targetPositionsJoints)

def line_path_ascend(p):
    del_k = 0.01
    x,y = p[0],p[1]
    for i in range(0, 100):
        z = p[2] + 0.1*del_k*i
        targetPositionsJoints = pybullet.calculateInverseKinematics(Robot, 7, [x,y,z])
        pybullet.setJointMotorControlArray(Robot, revolute_joints, pybullet.POSITION_CONTROL, targetPositions = targetPositionsJoints)
    
        pybullet.stepSimulation()
        time.sleep(1/100)

#-x-x-x-        

physics_client = pybullet.connect(pybullet.GUI) #For creating GUI
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

plane = pybullet.loadURDF("plane.urdf") #For plane
table = pybullet.loadURDF("/home/aniket/Documents/pybullet-ur5-equipped-with-robotiq-140/urdf/objects/table.urdf",useFixedBase = 1,basePosition = [0,0.5,0])
block = pybullet.loadURDF("/home/aniket/Documents/pybullet-ur5-equipped-with-robotiq-140/urdf/objects/block.urdf", basePosition = box_spawn_pos,useFixedBase = 0)
Robot = pybullet.loadURDF("/home/aniket/Documents/pybullet-ur5-equipped-with-robotiq-140/urdf/ur5_robotiq_140_backup.urdf", useFixedBase = 1)

pybullet.setGravity(0, 0, -98)
pybullet.setRealTimeSimulation(0)

orientation = pybullet.getQuaternionFromEuler([m.pi()/2, m.pi(), m.pi()/2]) #Orientation downward pointing
numJoints = pybullet.getNumJoints(Robot)

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

#-x-x-x-

robot = manipulator(box_spawn_pos,p0,p1,p2)
current_pos = initial_setup(robot.initial_pos)
open_gripper(current_pos)
current_pos = line_path_descend(robot.initial_pos)
current_pos = pybullet.calculateInverseKinematics(Robot, 7, [0.3,-0.5,1.06])
grab(current_pos)
line_path_ascend(robot.initial_pos)
exprx,expry,exprz = bezier_function(robot.initial_pos,robot.control_point,robot.final_pos)
curve_path(robot.initial_pos,exprx,expry,exprz)
current_pos = line_path_descend(robot.final_pos)
open_gripper(current_pos)
line_path_ascend(robot.final_pos)
