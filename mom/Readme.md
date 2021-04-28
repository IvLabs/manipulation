# Table of contents
- [Date 22-10-2020](#date-22-10-2020)
- [Date 30-10-2020](#date-30-10-2020)
- [Date 07-11-2020](#date-07-11-2020)
- [Date 22-11-2020](#date-22-11-2020)
- [Date 17-12-2020](#date-17-12-2020)
- [Date 29-12-2020](#date-29-12-2020)
- [Date 03-01-2021](#date-03-01-2021)
- [Date 20-01-2021](#date-20-01-2021)
- [Date 01-02-2021](#date-01-02-2021)
- [Date 14-02-2021](#date-14-02-2021)
- [Date 09-03-2021](#date-09-03-2021)
- [Date 24-03-2021](#date-24-03-2021)
- [Date 28-04-2021](#date-28-04-2021)


## Date 22-10-2020
* Mentors gave information about Osama Khalid.
* Aim of this week: To watch lecture 1 & 2 in detail.
* Make notes.(hand written)
* https://see.stanford.edu/Course/CS223A
* Mentors gave introduction about project and issue tab in github.


## Date 30-10-2020
* Doubt 1 = why we perform  **Homogenization** ?
* Answer =  To make Further calculations  Simple
* Doubt 2 = How the inverse  Transformation Matrix Obtained ?
* Answer = By tracing  the reverse path , there may be any obstacles considering  that in transformation  components
* Continue  to next leactures



## Date 07-11-2020
* Translation matrix is unit less.
* Rotation matrix's nine parmeters.
* Parameters for representation of manipulator.
     - length of arm .
     - Joint angle.
     - Angles b/w axis ( not sure ).
* There is not any velocity transformation matrix.
* state space vector .
* There are 12 no. of ways to attend any oriantation using Eular angles as well as Fixed angles method.
* Rotation matrix about an axis.
* Parameter K ( which is vector ) in Equivalent angle axis representation.
* Eulers Parameter , why we need that parameters  ?
     - To avoid singularity problem .
* The last Example using cosine angles.
---
* TO DO .
    - post the values of cosine angles on discord.
    - find the sign convention for that angles.
    - avoid confusion b/w XYZ and ZYX sequence.
    - Slove Assignment.
    
 
## Date 22-11-2020
* Maintain proper DOC
* Have direct communication with mentors and teammates avoid any third party apps(like voltaire).
* After completion of khatib lectures we have to make PPT explaining things learned.
* Like rohan sir showed a pipeline we will also have to create similar pipeline for our project. 
* Doubt from previous meet (about end-effector angle) explained.
* Video shared further explaining [DH parameters.](https://www.youtube.com/watch?v=nuB_7BkYNMk)

## Date 17-12-2020
* Target: 
    * Complete Oussama khatib lecture series and then start inverse kinematics( Lecture with vision can be excluded)
    * Simulation of 3R joint arm using python.
* Questions:
    1. Should we use CPP or python for simulation ?
    Conclusion: Will use cpp for final hardware implementation and python will be used as of now for 3R.
    2. Difference between simulation and animation?
    Conclusion: Animation is basically a dynamic graph kind of thing whereas in simulation realtime differential equations are solved to get future values.
* As we are free now our learning pace should increase.

## Date 29-12-2020

    1.Complete 3R From Scratch upto 10 December.
    2. Complete the Dynamics parallel with 3R with Gravity.
    3. Learn Py-Bullet
    4. Make Solidwork  Model
    5. Take it on Py-Bullet.
    -----
*   TO-DO
    Complete 3R forward Kinematics in python within 3-4 days.

## Date 03-01-2021

* Implement Newton Tapshon Method to find teh sol of quadratic
* Finish oussama khatib leacture series
* Finish Inverse kinematics leacture

## Date 20-01-2021
* 1. Work
* 2. To find Family of motors which to be used for the hardware purpose.
* 3. Coding language must be same in all.
* 4. Planning and algorithms...
     *    learn algorithms and implement.
     *    CAD model of arm... learn gazebo
     *    Else work simultaneously in all.
     
## Date 01-02-2021
* Install Pybullet
* Start the pybullet tutorial on youtube ( there is only one )
* complete ROS wiki tutorial upto 15
* Complete the assignment of ROS
* Do this Stuff in one week , next meet in the next week

## Date 14-02-2021
* Implementing slider in pybullet
* Complete ROS assignment
* Do pybullet tutorial on vscode
* Follow ![Quick start guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit) for pybullet
* Use Kuka UR5 for simulation

## Date 09-03-2021
* ROS acts as a bridge between your code and Hardware
* Solidworks -- URDF , SDF , Xacro
     * urdf + STL file = Simulation
     * SDF - Simulation Friendly
     * Xacro - makes easier i.e seprate files for each class
* RRBOT 
* pybullet 
     * design path
     * Go from one point to another point -- a) Joint Space   b) Task Space
     * Task Space : we can do path planning in it
     * make your Code to Transofrmation Matrix, X_dest , X_current, planning of Transformation
     * Screw --- St. Line
     * both SImultaneously but performed differently
     * Normal Method for inverse

## Date 24-03-2021
* doubts solved for pid control
* doubt regarding tragectory following and control solved
* how to convert xacro to urdf ? use command `xacro ur5_robot_name.urdf.xacro > ur5_robot_name.urdf`

## Date 28-04-2021
* Generate Trajectory for the simulation
* Try to solve pick and place in 4 steps
     * Position by inverse Kinematics and apply PID control
     * Grasp the Object
     * Pick up the object
     * place to next desired post
