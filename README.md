# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Model

The state of the car is composed of, first of all, the position (x, y) and the orientation of the car, psi. Additionally, we need the velocity v. Also, we need to consider the error in the state, cte, which is the distance of the vehicle from the trajectory, and epsi, which is the difference of the vehicle orientation and trajectory orientation.

There are two actuators, the throttle and the steering of the vehicle, with which we can control the velocity and orientation.

The update equation of the state are the following:

	x(t+1) = x(t) + v(t)*cos(psi(t)*dt)

	y(t+1) = y(t) + v(t)*sin(psi(t)*dt)

	psi(t+1) = psi(t) + v(t)/Lf*deltat*dt

	v(t+1) = v(t) + a(t)*dt

	cte(t+1) = f(x(t)) - y(t) + (v(t)*sin(epsi(t))*dt)

	epsi(t+1) = psi(t) - psides(t) + (v(t)/Lf*deltat*dt)

where deltat is the steering and a is the throttle.

## Timestep length and elapsed duration

I've used a value of n equal to 15 and a timestep of 0.1, which gives a duration of 1.5 second. To see how these values affect the result, let's modify each one, and see how the simulation behaves.

First, if we increase the value of N to, for example, 50, we get a duration of 5 s, which is too large and the vehicle becomes instable, and quickly gets out of the road. This is due to two reasons, because the duration is large, but also, because the calculations take too much time, and the car cannot react in time. If we now reduce the timestep to mantain the same duration, we have too many points, and it becames unstable, because it takes too much time to solve the solution too. On the other hand, if we reduce the timestep, but we mantain N equal to 15, the duration is too short and the vehicle cannot react in time, it takes too much time to act according to the changes in the road. The effect of this is similar of what we find if we choose a timestep smaller than the latency, as we will see later.

## Polynomial Fitting and MPC Preprocessing

Before the MPC procedure, I transform the global coordinates to local coordinates to the vehicle by the following formulas:
	
	dx_i = ptsx_i - px;
    dy_I = ptsy_i - py;
	x_i = dx_i * cos(-psi) - dy_i * sin(-psi)
    y_i = dx_i * sin(-psi) + dy_i * cos(-psi)

where pts are the waypoints, (px, py, psi) is the state of the vehicle, and (x, y) are the waypoints in the local coordinate frame.

## Model Predictive Control with Latency

First of all, to be able to take into account the latency, we can use a timestep bigger than the latency, which works quite well.

Another possible solution is to use the equations of the evolution of the state, and calculate the position after a time equal to the latency. Because psi is equal to zero in the local reference frame, we use the steering angle, and the equations are:

	px = px + v * cos(psi)*latency
	py = py + v * sin(psi)*latency
	cte = cte + v*sin(epsi)*latency
	epsi = epsi - v*delta*latency/Lf (we need to multiply delta by -1)
	psi = psi - v*delta*latency/Lf
	v = v + a*latency

(the order of the quations is important, because we need to use psi and v before updating them)

With this equations we take into account the latency, because we predict the state of the vehicle in the instant we are going to solve.
