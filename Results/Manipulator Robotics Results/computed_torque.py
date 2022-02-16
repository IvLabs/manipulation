import numpy as np
import pybullet as pb
import pybullet_data
import matplotlib.pyplot as plt
from time import sleep
from math import sin,cos
import time
import faulthandler
from manipulator import Manipulator
from trajectory import TrajObj
import colorama
from colorama import Fore

### computed torque control or feedforward control
if __name__ == "__main__":
    arm = Manipulator()
    dt = 1/2.0
    ini = [0.817250000000927, 0.19144999999999995, 0.9945090000040019]
    final = [0.5 , 0.8 , 0.3]
    traj = TrajObj( ini , final, [0,0,0] , [0,0,0] , dt)

    kp , kd = 0.1 , 0.1

    faulthandler.enable()
    zeroVect = np.zeros( len( arm.controlJoints))
    pb.setJointMotorControlArray( arm.armID , arm.controlJoints ,pb.VELOCITY_CONTROL, forces=zeroVect)

    q_d_old = np.zeros(len(arm.controlJoints))
    i = 0
    while  i <= 1000:
        ## get current state
        jointAngles,jointVelocities,__ = arm.getJointAngles()
        ## ge desired state
        ef_pos , ef_vel , ef_acxl = traj.getNextpoint()

        q_ = arm.inverseKinematics2(ef_pos)

        jac = arm.calculateGeometricJacobian2( list(ef_pos) , list(jointAngles) , list(jointVelocities))
        inv_jac = np.dot(jac.T ,  np.linalg.inv( np.dot( jac , jac.T )  )) 

        q_d_ = np.dot( inv_jac , np.array(ef_vel ) )

        q_dd_ = ( q_d_ - q_d_old) 

        pos_error = q_ - jointAngles
        
        while (  np.any(abs( pos_error) > 0.05 ) ) :
            jointAngles,jointVelocities,__ = arm.getJointAngles()

            ## pos error, 
            pos_error = q_ - jointAngles
            ## velocity error, 
            vel_error = q_d_ - jointVelocities

            axcl = q_dd_  + kp* pos_error + kd*vel_error
            massMatrix, gravityVector , coriolisVector = arm.calculateDynamicMatrices( list( jointAngles), list(jointVelocities) )
            # torque = 1e-8 * ( np.dot(massMatrix, axcl) + coriolisVector ) + gravityVector
            torque = gravityVector
            # torque = 0 *torque
            # print(np.shape(torque) )
            pb.setJointMotorControlArray(arm.armID,arm.controlJoints,pb.TORQUE_CONTROL,forces = torque)
            pb.stepSimulation()
            time.sleep( 1/200.)

        i += dt
        print(i)
        # pb.addUserDebugLine(last, current_eepos, [0, 0, 0.5], 1, 20)
        # if (  np.all(abs( pos_error) < 0.05 )   ):
            # i += 1
        