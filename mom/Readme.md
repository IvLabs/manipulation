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
- [Date 03-12-2021](#date-03-12-2021)
- [Date 11-12-2021](#date-11-12-2021)
- [Date 15-12-2021](#date-15-12-2021)
- [Date 17-12-2021](#date-17-12-2021)
- [Date 21-12-2021](#date-21-12-2021)
- [Date 26-12-2021](#date-26-12-2021)
- [Date 05-01-2022](#date-05-01-2022)
- [Date 09-01-2022](#date-09-01-2022)
- [Date 19-01-2022](#date-19-01-2022)
- [Date 26-01-2022](#date-26-01-2022)
- [Date 02-02-2022](#date-02-02-2022)
- [Date 04-02-2022](#date-04-02-2022)
- [Date 07-02-2022](#date-07-02-2022)
- [Date 15-02-2022](#date-15-02-2022)
- [Date 22-02-2022](#date-22-02-2022)

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

## MoM Manipulator robotics (Date 03-12-21 Onwards)

## Date 03-12-2021

* Starting from the basics- 
* 7 lectures-https://www.youtube.com/playlist?list=PL64324A3B147B5578
* Ask doubts as they come on the server
* Add notes and move MoM to Github repo

## Date 11-12-2021
* Complete the course till 8th lec and upload the notes
* Target- forward kinematics implementation of RRR arm using SymPy library and visualizing it on Matplotlib 

## Date 15-12-2021

* Task on Inverse kinematics by implementing Newton-Raphson's method

## Date 17-12-2021

* Install pybullet.
* Upload the results of Inverse Kinematics problem.

## Date 21-12-2021

* Complete Pybullet FK and IK implementations.
* PyBullet advantages
* How does pybullet implement IK, working of pybullet IK.
* See PyBullet QuickStart Guide

## Date 26-12-2021
* Make and keep Documentation of the project, useful for future work
* Find a way to implement PID controller in python:(check videos before)
* https://youtu.be/HFXdcImAwv4 
* https://youtu.be/COWaaCknkcY

## Date 05-01-2022
* Implement Pick and place using force and velocity control.
* Pid is a resort if that didn’t work out.
* Try using pybullet.velocitycontrol mode.
* Make a trajectory which may account for a third control point which can represent an obstacle in the real world.

## Date 09-01-2022
* Put all results on a Github repo with proper presentation
* Velocity Control-Use the resources on channel
* Finish the Lectures from Stanford intro to robotics series
* Next meet at 19th
* Implement dynamics if possible
* Moving to next Phase

## Date 19-01-2022

Dynamics Books :
* https://1lib.in/book/2802899/0c19ca
* https://1lib.in/book/782106/1f55f4
* These books doesn't explain the expressions from base, try derivative those expressions after reading from books then understand their use.
* Lagrangian video: https://www.youtube.com/watch?v=1U6y_68CjeY
## Date 26-01-2022
* Velocity control doubts
* Try increasing the gains

## Date 02-02-2022
* Implementation of VC- issues: check workspace, proper tuning
* To complete velocity control first


## Date 04-02-2022
* Start Implementation of Torque control
* Use PD controller for torque values.
* Control Mode: Torque control
* PD controller error term based based on distance.

## Date 07-02-2022

* Computed torque control
* Book ‘modern robotics’ over this topic
* Stiffness control, find scholarly papers or books related to stiffness control

### Date 15-02-2022

* Don’t use a while loop within trajectory loop, otherwise it will stop and cause jerk because of large difference in error term in the next iteration of loop,
* Even P-controller will work for simulations, PD is usually enough for most simulations,
* PID is needed to eliminate errors arising in hardware,
* Coriolis force component already compensates for the gravity hence you must not add gravity component if you add the coriolis component,
* Improve code readability/make it modular if possible (just a recommendation, helps several people to work together in a piece of code)

### Date 22-02-2022

* Explore Stiffness control and
* Explore Impedence control.
* Can explore Modern Robotics book on this
* Try to complete before MSE.



