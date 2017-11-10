# CarND-MPC-Project
Controlling a simulated car using Model Predictive Control.

## Model
I have used a global kinematic model in this project. The car has the following variables in the state of the car:
1. px (x-position of the car in the car coordinates)
2. py (y-position of the car in the car coordinates)
3. psi angle made by the car in the car coordinate system
4. v velocity of the car
5. cte cross track error of the car wrt to desired path
6. epsi error in the psi value i.e (psi_desired - psi)

I have used the IPOPT library for optimization. Optimization involves three major parts, namely:
* Error function which needs to be minimized
* Variables whose values are to be found for minimum error
* Constraints on what values are permitted for the variables

### Variables
The car has a state at each of the N timesteps, which means there are N px values, N py values, N psi values and so on.
Along with these values other variables include actuator values of the car. The actuator values include \[delta, a\]. There are a total of N - 1 actuator values.
So in conclusion the total vars that are optimized are 6*N + 2*(N-1).
These vars are optimized to give the least errors

### Constraints
In order to get realistic actuator values from the optimizer, constraints are put on these variables to mimic a real car as much as possible. The constraints used in this project are those of a global kinematic model. These are the contraints used:
<img width="358" alt="eqns" src="https://user-images.githubusercontent.com/26694585/32673552-ac138f7c-c675-11e7-84a2-51105c0c7cc6.png">



### Error Function
The error function consists of functions that you would like to minimize. These are all the error functions I used along with their parameters:
```
		// The part of the cost based on the reference state.
		for (unsigned int t = 0; t < N; t++) {
			fg[0] += 3000*CppAD::pow(vars[cte_start + t], 2);
			fg[0] += 1600*CppAD::pow(vars[epsi_start + t], 2);
			fg[0] += 20*CppAD::pow(vars[v_start + t] - ref_v, 2);
		}

		// Minimize the use of actuators.
		for (unsigned int t = 0; t < N - 1; t++) {
			fg[0] += 5*CppAD::pow(vars[delta_start + t], 2);
			fg[0] += 5*CppAD::pow(vars[a_start + t], 2);
			fg[0] += 700 * CppAD::pow(vars[delta_start + t] * vars[v_start + t], 2);
		}

		// Minimize the value gap between sequential actuations.
		for (unsigned int t = 0; t < N - 2; t++) {
			fg[0] += 60*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
			fg[0] += 10*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
		}
```
I have added an extra error function that is *velocity\*delta*, this ensured that my turns were much more smoother as the car would slow down when it had to turn.

The error parameters were really tricky to modify
For example: If I am decreasing the errors on delta then the car tends to overshoot, however if I am constraining the delta value too much then my car tends to run parallel to the waypoints and takes a long time to come closer to the reference line.
So I had to do a lot of trial and error befor finally reaching the above said parameters.

## Timestep Length and Elapsed Duration (N & dt)
The values I have used are N = 10, and dt = 0.1. I tried to increase N values but that resulted in more wavery results due to longer time taken for calculation, and also with a higher value for N the car would stay away from the reference line even if there is a turn far away, so it would in a way pre-prepare itself for the upcoming turn.

## Polynomial Fitting and MPC Preprocessing
As mentioned above all the state values are taken in car coordinate systems. I transformed the obtained ptsx, and ptsy values of the reference line by computing a simple rotation and translation, similar to what I have done in the Particle Filter Project. This is how it was done:
```
		  for (unsigned int i = 0; i < ptsx.size(); i++)
		  {
			  double shift_x = ptsx[i] - px;
			  double shift_y = ptsy[i] - py;

			  ptsx[i] = (shift_x*cos(0 - psi) - shift_y*sin(0 - psi));
			  ptsy[i] = (shift_x*sin(0 - psi) + shift_y*cos(0 - psi));
		  }
```
Here px, py are the positions of the car in map coordinates.

## Model Predictive Control with Latency
The MPC handles latency by simulating a time difference between sending the time the actuation commands were sent and the time the car implements these commands.


## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
