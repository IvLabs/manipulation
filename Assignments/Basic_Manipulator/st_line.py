import pybullet as p
import time
import pybullet_data
from math import pi
physicsClient = p.connect(p.GUI)

#https://usermanual.wiki/Document/pybullet20quickstart20guide.479068914/html

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")# setting the plane
loc = r"/home/nachiket/Desktop/assignments/AssignmentsIVLABS/Controls/pybullet/kuka_experimental/"
ation = r"kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf"
bot = p.loadURDF(loc+ation,[0,0,0],useFixedBase = 1)
numJoints = p.getNumJoints(bot)
revolute_joints = []
for i in range(numJoints):
    info = p.getJointInfo(bot, i)

    jointName = info[1]
    jointType = info[2]
    if (jointType == p.JOINT_REVOLUTE):
        revolute_joints.append(i)



def inverse(ef_position = (3,0,0)):
    joint_pose = p.calculateInverseKinematics(bot , revolute_joints[-1], ef_position)
    return joint_pose
def forward(joint_angles = [0 for i in revolute_joints ]):
    p.setJointMotorControlArray(bot,revolute_joints,p.POSITION_CONTROL,targetPositions = joint_angles )
    joints_position = []
    for j in revolute_joints:
        joints_position.append( p.getLinkState(bot , j)[:2] )
    return  joints_position


m , n = 1000 , 0
final  = [0,0.8,0.8]
ini = [0.8,0,0.8]
current = [0.8,0,0.8]
last = [0.8,0,0.8]
p.setRealTimeSimulation(1)
i=0

trailDuration =15 
while i < 1000:
    i+=1
    m -= 1
    n += 1
    # time.sleep(1./40.)
    last = [i for i in current ]
    current[0] = (m*final[0]+n*ini[0])/(m+n)
    current[1]  = (m*final[1]+n*ini[1])/(m+n)
    current[2]  = (m*final[2]+n*ini[2])/(m+n)

    forward(inverse(current))
    p.addUserDebugLine(last, current, [0, 0, 0.3], 1, trailDuration)
    p.stepSimulation()
    time.sleep(1./240.)
 




