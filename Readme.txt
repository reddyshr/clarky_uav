Readme

CODEBASE OVERVIEW

run_uav_sim.m
* This is the upper level simulation file used to run the simulation. The primary inputs, such as thrust value and flap angles, can be set in the initial conditions section of the file.


Translation3D.m 
* written by CJ Taylor. This function takes in three parameters, and x distance, y distance, and a z distance and constructs a homogeneous transformation. 


Rotation3D.m
* written by CJ Taylor. This function takes in an axis x, y, or z, and an angle and constructs the homogeneous transformation about the specified axis by the the specified angle. 


Scale3D.m 
* written by CJ Taylor. This function takes in three parameters, and x scale, y scale, and z scale, and produces the homogeneous transformation matrix.


ApplyTransform.m 
* written by CJ Taylor. This function takes in a homogeneous transformation and a patch object and applies the transformation to each vertex in the patch object. 


Stlread.m 
* written by Eric Johnson. This function takes in an stl file and creates a matlab patch object. 


Update_state.m 
* This function is where the ode45 call lives. It takes in a time span and a UAV object and runs ode45 on the current state of the UAV object. 


UAV.m
* This UAV class could be used to construct a UAV object which contains all the necessary geometric, inertial, and aerodynamic characteristics of our UAV, as well as some functions used for calculation, coefficient of lifts/drags, angle of attack, non conservative generalized forces, etc.


Compute_dynamics.m
* This function uses the properties and state of the UAV, along with the dynamical equations, to solve for the derivative of the state. This function is used in our ODE45 function
* syms_lagrange_eqM.m 
* This function is not part of the simulation but symbolically calculates the Lagrange and equations of motion for the system. 
compute_omega_from_R.m
* This function computes the angular velocity from the rotation matrix and is not used explicitly in the function.
