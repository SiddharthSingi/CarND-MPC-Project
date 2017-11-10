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



### Error Function
If I am decreasing the errors on delta then the car tends to overshoot, however if I am constraining the delta value too much then my car tends to run parallel to the waypoints and takes a long time to come closer to the reference line.
