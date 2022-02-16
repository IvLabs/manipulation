import numpy as np
import pybullet as pb
import pybullet_data
import matplotlib.pyplot as plt
from time import sleep
from math import sin,cos
import time
import faulthandler

class Manipulator():
    def __init__(self,basePosition = [0,0,0],baseOrientation = [0,0,0,1]):
        # Start Pybullet
        pb.connect(pb.GUI, options = "--cl_device=2" )
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0,0,-9.81)
        location = r"/home/nachiket/Desktop/work/ivlabs/n/Controls/pybullet/kuka_experimental/kuka_ur5/force-control/files/urdf/ur5.urdf"
        #load URDF
        self.armID = pb.loadURDF(location,basePosition,baseOrientation,useFixedBase = True)

        #define joints
        self.controlJoints = []
        self.totalJoints = pb.getNumJoints(self.armID)
        print("Total Number of joints : ", self.totalJoints)

        ## define the joints that we can control
        # set the end effector link
        self.endEffectorIndex = -1
        for i in range(self.totalJoints):
            jointInfo = pb.getJointInfo(self.armID,i)

            if jointInfo[2]==0:
                #append the joints we can control
                self.controlJoints.append(i)

            if jointInfo[1] == b'ee_fixed_joint':
                #set the endeffector joint
                self.endEffectorIndex = i
        print(self.endEffectorIndex,"is the end effector index")
        ## control zero is the zero vec required for variopus calculations.
        ## the length for the zero vector is the length of the control joints
        self.controlZero = [0] * len(self.controlJoints)


    def setJointAngles(self,reqjointAngles):
        pb.setJointMotorControlArray(self.armID,self.controlJoints,pb.POSITION_CONTROL,targetPositions = reqjointAngles,targetVelocities = self.controlZero,
                                     positionGains = 6*[0.5], velocityGains = 6*[0.2])
        for i in range(200):
            pb.stepSimulation()

    def getJointAngles(self):
        jointState = pb.getJointStates(self.armID,self.controlJoints)
        jointAngles = [ i[0] for i in jointState]
        jointVelocities = [ i[1] for i in jointState]
        jointReactionForces = [ i[2] for i in jointState]

        return np.array(jointAngles),np.array(jointVelocities),np.array(jointReactionForces)

    def forwardKinematics(self):
        endEffectorLink = pb.getLinkState(self.armID,self.endEffectorIndex,computeForwardKinematics = True)
        position,orientation = endEffectorLink[0],pb.getEulerFromQuaternion(endEffectorLink[1])
        endEffector = position + orientation
        return np.array(endEffector) 


    def inverseKinematics(self,endEffector):
        position,orientation = endEffector[0:3],endEffector[3:6]
        quatOrientation = pb.getQuaternionFromEuler(orientation)
        jointAngles = pb.calculateInverseKinematics(self.armID,self.endEffectorIndex,targetPosition = position,targetOrientation = quatOrientation)
        return jointAngles

    def inverseKinematics2( self , endEffector):
        jointAngles = pb.calculateInverseKinematics(self.armID,self.endEffectorIndex , targetPosition = endEffector)
        return np.array(jointAngles)

    def calculateGeometricJacobian(self,jointAngles):

        endEffector = pb.getLinkState(self.armID,self.endEffectorIndex)
        linJac,angJac = pb.calculateJacobian(self.armID,self.endEffectorIndex,endEffector[2],jointAngles,self.controlZero,self.controlZero)
        jacobian = np.vstack((linJac,angJac))

        return jacobian


    def calculateGeometricJacobian2(self,pos , jointAngles , jointVelocities):
        # print(self.armID,self.endEffectorIndex,pos,jointAngles,jointVelocities,self.controlZero)
        linJac,angJac = pb.calculateJacobian(self.armID,self.endEffectorIndex,pos,jointAngles,jointVelocities,self.controlZero)
        return np.array( angJac )
        # return  np.vstack((linJac,angJac))

    def calculateAnalyticJacobian(self,jointAngles):
        endEffector = self.forwardKinematics()
        roll,pitch,yaw = endEffector[3:6]

        AMatrix3 = np.array([
            [1,0,sin(pitch)],
            [0,cos(roll),cos(pitch)*sin(roll)],
            [0,sin(roll),cos(pitch)*cos(roll)]
        ])

        AMatrix3Inv = np.linalg.inv(AMatrix3)

        AMatrix6 = np.block([
            [np.eye(3),        np.zeros((3,3))],
            [np.zeros((3,3)),  AMatrix3Inv]
        ])


        jacobian = self.calculateGeometricJacobian(jointAngles)

        analyticJacobian = np.dot(AMatrix6,jacobian)

        return analyticJacobian


    def calculateDynamicMatrices(self,jointAngles,jointVelocities):
        
        massMatrix = np.array(pb.calculateMassMatrix(self.armID,jointAngles))
        gravityVector = np.array(pb.calculateInverseDynamics(self.armID,jointAngles,self.controlZero,self.controlZero))
        coriolisVector = np.array(pb.calculateInverseDynamics(self.armID,jointAngles,jointVelocities,self.controlZero)) - gravityVector
        
        return massMatrix,gravityVector,coriolisVector



    def addGUISliders(self,startPos):
        self.sliders = []
        self.sliders.append(pb.addUserDebugParameter("X",-1.5,1.5,startPos[0]))
        self.sliders.append(pb.addUserDebugParameter("Y",-1.5,1.5,startPos[1]))
        self.sliders.append(pb.addUserDebugParameter("Z",-1.5,1.5,startPos[2]))
        self.sliders.append(pb.addUserDebugParameter("Roll",-1.5,1.5,startPos[3]))
        self.sliders.append(pb.addUserDebugParameter("Pitch",-1.5,1.5,startPos[4]))
        self.sliders.append(pb.addUserDebugParameter("Yaw",-1.5,1.5,startPos[5]))

    def readGUIValues(self):
        x = pb.readUserDebugParameter(self.sliders[0])
        y = pb.readUserDebugParameter(self.sliders[1])
        z = pb.readUserDebugParameter(self.sliders[2])
        roll = pb.readUserDebugParameter(self.sliders[3])
        pitch = pb.readUserDebugParameter(self.sliders[4])
        yaw = pb.readUserDebugParameter(self.sliders[5])
        return np.array([x,y,z,roll,pitch,yaw])

