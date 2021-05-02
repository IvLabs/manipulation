import pybullet as p
import time
import pybullet_data
from math import pi
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation 
from celluloid import Camera
from matplotlib.animation import PillowWriter
import ffmpeg

class manipulator():
    def __init__( self , loc ,path ):
        self.path = loc + path 
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")# setting the plane
        tableStartPos = [0.0, -0.9, 0.75]
        table = p.loadURDF(loc+"kuka_experimental/pybullet-ur5-equipped-with-robotiq-140/urdf/objects/table.urdf", tableStartPos,useFixedBase = True)
        ur5standStartPos = [-0.7, -0.36, 0.0]
        stand = p.loadURDF(loc+"kuka_experimental/pybullet-ur5-equipped-with-robotiq-140/urdf/objects/ur5_stand.urdf", ur5standStartPos,useFixedBase = True)
        self.bot = p.loadURDF(self.path ,[0,0,0],useFixedBase = 1)
        self.numJoints = p.getNumJoints(self.bot)
        self.revolute_joints = []
        self.upper_limits = []
        self.lower_limits = []
        for i in range(self.numJoints):

            info = p.getJointInfo(self.bot, i)

            jointName = info[1]
            jointType = info[2]
            if (jointType == p.JOINT_REVOLUTE):
                self.revolute_joints.append(i)

            self.lower_limits.append(-pi) 
            self.upper_limits.append( pi)

       
        self.gripper_angle = 0
        self.debug_list = [ [] , [] , []]

    def set_object(self , obj_pos ,loc):
        obj = obj_pos
        boxId = p.loadURDF(loc+"kuka_experimental/pybullet-ur5-equipped-with-robotiq-140/urdf/objects/block.urdf", obj )

    def debug(self):
        print(p.getBasePositionAndOrientation(self.bot ))
        self.angle_id = []
        for i in range(self.numJoints):
            self.angle_id.append(p.addUserDebugParameter("joint_"+str(i),-1*pi*2,pi*2,0))

    def inverse(self,ef_position = (3,0,0) ,r = 0 , pc= 0 , y = 0):
        orientation = ( r*pi/2 , pc*pi/2 , y*pi/2 )
        orientation = p.getQuaternionFromEuler(orientation)
        joint_pose = p.calculateInverseKinematics(self.bot ,7, ef_position , orientation ,
                                                 lowerLimits = self.lower_limits ,upperLimits = self.upper_limits)
        return joint_pose # 

    def inverse2(self,ef_position = (3,0,0) ,*args ):
        
        joint_pose = p.calculateInverseKinematics(self.bot , 7, ef_position 
                                               ,lowerLimits = self.lower_limits ,upperLimits = self.upper_limits)
        return joint_pose

    def forward(self , joint_angles ):
        if  (np.array_equal(joint_angles,None) )  :
            joint_angles = [ 0 for i in self.revolute_joints]

        return   p.getLinkState(self.bot , 7)[:2][0]

    def comapare(self , pos1 ,pos2):
        flag1 = pos1[0] == pos2[0]
        flag2 = pos1[1] == pos2[1]
        flag3 = pos1[2] == pos2[2]
        return flag1 and flag2 and flag3

    def controlGripper(self ,pose):
        parentid = 12
        child = [9 ,10 ,11,13,14,15,16]
        controlMode = p.POSITION_CONTROL
        p.setJointMotorControl2(self.bot, parentid, controlMode, targetPosition=pose)
        time.sleep(1/50.)
        p.stepSimulation()
        for jointid in child:
            childPose = pose * 1
            p.setJointMotorControl2(self.bot, jointid, controlMode, targetPosition=childPose)
            time.sleep(1/50.)
            p.stepSimulation()

    def move(self , joint_angles = None):

        if (np.array_equal(joint_angles,None) ):
            joint_angles = self.up_pos

        joint_angles = np.array(joint_angles , dtype=float)

        kp , kd , ki  = 0.1 , 0.0 , 0.0001

        error =  np.array([0 for _ in range(len(self.revolute_joints))], dtype=float) 
        e_sum =  np.array([0 for _ in range(len(self.revolute_joints))], dtype=float) 
        e_old =  np.array([0 for _ in range(len(self.revolute_joints))], dtype=float) 
      
        current_angle = p.getJointStates(self.bot , self.revolute_joints)
        current_angle = np.array([ i[0] for i in current_angle] , dtype=float)
        tik_count  =  True
        stop_countdown = 50
        while stop_countdown > 0 :
            if np.array_equal(current_angle , joint_angles ):
                tik_count  =  True
            if tik_count :
                stop_countdown -= 1
            
            error = joint_angles - current_angle 
            
            e_sum += error
            de = error - e_old
            e_old = error
            current_angle += (kp*error + kd*de + ki*e_sum) 
            p.setJointMotorControlArray(self.bot,self.revolute_joints,p.POSITION_CONTROL,targetPositions = list(current_angle) )
            self.controlGripper(self.gripper_angle)
            time.sleep(1/100.)
            p.stepSimulation()

            pos = self.forward(current_angle)
            self.debug_list[0].append(pos[0])
            self.debug_list[1].append(pos[1])
            self.debug_list[2].append(pos[2])
 


    def plot(self):

        for i in range(len(self.debug_list[0])):
            print(self.debug_list[0][i],self.debug_list[1][i],self.debug_list[2][i])

        plt.title("x vs t")
        plt.plot( self.debug_list[0] , 'ro')  
        plt.show()
        plt.title("y vs t")
        plt.plot( self.debug_list[1] , 'ro')
        plt.show()
        plt.title("z vs t")
        plt.plot( self.debug_list[2] , 'ro')
        plt.show()
        return

    def run_simulation(self  , ini_pos ,obj_pos , fin_pos):
        t1 = time.time()
        obj_pos = [obj_pos[0] , obj_pos[1], obj_pos[2]+0.3 ]
        fin_pos = [fin_pos[0] , fin_pos[1], fin_pos[2]+0.4 ]
        ini_joint_angle = self.inverse(ini_pos ,0,0,0)
        obj_joint_angle = self.inverse(obj_pos ,0,1.0,0 )
        fin_joint_angle = self.inverse(fin_pos ,0,1.0,0)
        self.move(ini_joint_angle)
        i = 0 
        while (i<1):
            
            #moving to obj pos slightly up
            self.move(obj_joint_angle)
            #moving on object
            obj_pos = [obj_pos[0] , obj_pos[1], obj_pos[2]-0.10 ]
            obj_joint_angle = self.inverse(obj_pos ,0,1.0,0 )
            self.move(obj_joint_angle)
            # gripping the object
            self.gripper_angle = 0.44
            self.controlGripper(self.gripper_angle)
            time.sleep(1)
            obj_pos = [obj_pos[0] , obj_pos[1], obj_pos[2]+0.20 ]
            obj_joint_angle = self.inverse(obj_pos ,0,1.0,0 )
            self.move(obj_joint_angle)
  
            # moving to target
            self.move(fin_joint_angle)
            fin_pos = [fin_pos[0] , fin_pos[1], fin_pos[2]-0.10 ]
            fin_joint_angle = self.inverse(fin_pos ,0,1.0,0 )
            self.move(fin_joint_angle)
            # releasing the object 
            self.gripper_angle = 0
            self.controlGripper(0)
            self.controlGripper(self.gripper_angle)
            time.sleep(1)

            fin_pos = [fin_pos[0] , fin_pos[1], fin_pos[2]+0.20 ]
            fin_joint_angle = self.inverse(fin_pos ,0,1.0,0 )
            self.move(fin_joint_angle)

            self.move(ini_joint_angle)
            t2 = time.time()
            print(t2-t1)
            self.plot()



if __name__ == '__main__' :
    loc = r"/home/nachiket/Desktop/assignments/AssignmentsIVLABS/Controls/pybullet/"
    ation = r"kuka_experimental/pybullet-ur5-equipped-with-robotiq-140/urdf/ur5_robotiq_140.urdf"

    arm = manipulator(loc,ation)

    ini_pos = [ 1,0,0]

    obj_pos = [0.2, -0.85, 0.78]

    fin_pos = [-0.2 ,-0.85,0.78]

    arm.set_object(obj_pos , loc)

    arm.run_simulation(  ini_pos ,obj_pos , fin_pos)
