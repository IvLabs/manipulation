# Manipulation 
Control a Robotic arm and perform tasks autonomously


 a manipulator is a device used to manipulate materials without direct physical contact by the operator. ... manipulators have the ability to reach in to tight spaces and remove workpieces.
 
 In this project we study the manipulator's 
 
 - Forward kinematics 
 - Inverse kinematics 
 - Dynamics

# Forward Kinematics

 Forward kinematics is frequently used to calculate the position of end effector when we know the degree value of each joint.To calculate forward kinematic, we can use simple trigonometry or denavit hartenberg parameter or screw theory.In this example we have used D-H parameters to calculate 3d forward kinematics for 3 DOF robotic arm.


 # Inverse Kinematics 
   Inverse kinematics is about calculating the angles of joints (i.e. angles of the servo motors on a robotic arm) that will cause the end effector of a robotic arm (e.g. robotics gripper ) to reach some given desired position (x, y, z) in 3D space. 
   
|Forward Kinematics   | Inverse Kinematics   |
|:-------------------:|:--------------------:|
|![Forward_](https://user-images.githubusercontent.com/70883690/118961286-ca3f5500-b981-11eb-9077-21a34d389091.gif)|![Inv](https://user-images.githubusercontent.com/70883690/118954284-3c606b80-b97b-11eb-867e-dd71b803e61b.gif)|
   

# Dynamics
  Robot dynamics studies the relation between robot motion and forces and moments acting on the robot. For a robotic manipulator, the dynamic model defines the relationship between joint position qi, angular velocity q˙i, and angular acceleration q¨i to torque τi necessary to achieve desired position, velocity, and acceleration.
  
 # Pick and Place
  Our aim was to pick a box and place to another position, given that intial and final box position. We have applied velocity control to manuver the arm from intial to final position
  
 ![picknplace](https://github.com/IvLabs/manipulation/blob/main/pick_place_Results/pick-place.gif)
|x vs t   | y vs t   | z vs t|
|:-------:|:--------:|:-----:|
|![](/pick_place_Results/x_vs_t.png)|![](/pick_place_Results/x_vs_t.png)|![](/pick_place_Results/x_vs_t.png)|

 
 
