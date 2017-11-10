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
The values I have used are N = 10, and dt = 0.1. I tried to increase N values but that resulted 
