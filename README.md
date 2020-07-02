# FCND-3D-Quadrotor-Controller
In this project, a complete cascaded controller is going to be implemented in C++. Part of the control design relies on some math that one can find on the following paper [Feed-Forward Parameter Identification for Precise Periodic
Quadrocopter Motions](http://www.dynsyslab.org/wp-content/papercite-data/pdf/schoellig-acc12.pdf).

<img src="./images/pid0.png" width =700/>
The diagram above summarizes the different control blocks that constitute the cascaded P and PD controllers.

## Simulator and C++ Implementation
Most of the code in this project can be found in [FCND-Controls-CPP](https://github.com/bwassim/FCND-3D-Quadrotor-Controller/tree/master/FCND-Controls-CPP) folder. The main file for coding the controller is [src/QuadControl.cpp](https://github.com/bwassim/FCND-3D-Quadrotor-Controller/tree/master/FCND-Controls-CPP/src). All the configuration files for the controller and the vehicle are in the `config` directory. All the tunning parameters will be done in the file called `QuadControlParams.txt`. We will be using the simulator to fly different trajectories to test out the performance of the C++ controllers. These trajectories can be found in `traj` repo.
### Prerequisites
If you are using a mac, Xcode would be enough to run the project. Otherwise follow the prerequisites in the seed project's [README](https://github.com/udacity/FCND-Controls-CPP) fil

### The Simulator 
After running the project, in the simulator window itself, you can right click the window to select between a set of different scenarios that are designed to test the different parts of your controller.
The simulation (including visualization) is implemented in a single thread. This is so that you can safely breakpoint code at any point and debug, without affecting any part of the simulation.
Vehicles are created and graphs are reset whenever a scenario is loaded. When a scenario is reset (due to an end condition such as time or user pressing the `R` key), the config files are all re-read and state of the simulation/vehicles/graphs is reset -- however the number/name of vehicles and displayed graphs are left untouched.
e.
### Testing it out
When you run the simulator, you'll notice your quad is falling straight down. This is due to the fact that the thrusts are simply being set to:
`QuadControlParams.Mass * 9.81 / 4`. Therefore, if the mass doesn't match the actual mass of the quad, it'll fall down.
It can be verified that a mass = 0.24 [kg] will make the vehicle hover for short period of time. 
### Objectives
We would like to design the different controller blocks in order to control the drone in 3D. To do this we need to proceed with the following steps 
1. Implement a Body rate controller
2. Implement a roll/pitch controller 
3. Implement an altitude controller 
4. Implement a lateral position controller
5. Implement a yaw controller.


### Scenario 2: Body rate control
For the attitude control, scenario 2 will be used in the simulation. The body rate control will stabilize the rotational motion and bring the vehicle back to level altitude.
Initially we start by implementing the body rate controller. The latter generates the command Moment as follows 
- Calculate the error between the desired and current body rate
- generate the moment command vector as follows 

<img src="./images/cmdMoment.png" width="600"/>

Notice that we have used a P controller for the body rates p, q, r. [BodyRateControl](https://github.com/bwassim/FCND-3D-Quadrotor-Controller/blob/552d921b647f1052275d90093f553645f029aa1f/FCND-Controls-CPP/src/QuadControl.cpp#L111-L117) function, that takes as argument the desired body rates pqrCmd and the current or estimated body rates.
Now that we have the moment command values, it is possible to derive the thrust in each rotor by solving the following set of equations

<img src="./images/thrust_equations.png" width="257"/> 
<img src="./images/drone1_1.png" width="257"/> 

The result is coded in the [GenerateMotorCommands](https://github.com/bwassim/FCND-3D-Quadrotor-Controller/blob/552d921b647f1052275d90093f553645f029aa1f/FCND-Controls-CPP/src/QuadControl.cpp#L73-L82) function. We start by tunning the parameters kpPQR until we stabilize the rotation rate omega.x. The vehicle will consequently drift since we have not yet built the pitch roll controller. Before I proceed I would like to show you the final result in the animated figure below. 

<img src="./images/body_roll_pitch.gif" width="620" /> 

### Pitch Roll control 
We won't be worrying about the Yaw angle yet. Neither the fact that the drone is loosing altitude, since the altitude controller is going to be implemented soon. 

The roll-pitch controller is a P controller responsible for commanding the roll and pitch rates (pc and qc) in the body frame based on a desired global lateral acceleration, the current attitude of the quad, and desired thrust command

**Note** - subscript c means "commanded" and a means "actual"

where bx_a = R13 and by_a=R23. The given values can be converted into the angular velocities into the body frame by the next matrix multiplication. 

<img src ="./images/dot.png" width="150"/>

Remember that the lateral position controller calculates the reference accelerations in the x-y directions. Since we know the desired collective thrust denoted in the the C++ project by `collThrustCmd` (provided by the altitude controller) and the reference acceleration in the x-y direction, i.e., `accelCmd.x`, `accelCmd.y` respectively, we can derive the reference rotational angle in both directions. 

<img src="./images/collCmd.png" width="200"/>

We know that the last column of the rotational matrix represent the rotational angles that we need to stabilize around the reference bx_c, by_c and for that we use a P controller. 

<img src="./images/bxp.png" width="200"/>

These values are given in the intertial frame and needs to be converted to the body frame because the rotational acceleration from the gyros are measured in the body frame. The following transformation is needed

<img src="./images/pqR.png" width="300"/>

We have not talked about constraints but it is important that the bx_c and by_c belong to some interval with min, max tilt angles. 
The code for the pitch/roll controller is given here [FCND-Controls-CPP/src/QuadControl::RollPitchControl](https://github.com/bwassim/FCND-3D-Quadrotor-Controller/blob/8b07c5182f6e955f147d2fb44334823f05316c73/FCND-Controls-CPP/src/QuadControl.cpp#L148-L173)

### Scenerio 3: Position/Velocity and yaw angle control
In this scenario we will explore the design of three controller blocks
* The lateral position controller: `LateralPositionControl()`
* The altitude controller: `AltitudeControl()`
* The Yaw controller: `YawControl()`

The lateral controller will use a PD controller to command target values for elements of the drone's rotation matrix. The drone generates lateral acceleration by changing the body orientation which results in non-zero thrust in the desired direction. This will translate into the commanded rotation matrix elements bx_c and by_c. The control equations have the following form:

<img src="./images/lateral.png" width ="350"/>

For the y direction we use the same form as above.

