# CarND-Controls-MPC

The goal of this project is to implement Model Predictive Control to drive the car around the track.

[//]: # (Image References)
[img1]: ./mpc_project.png
[img2]: ./kinematic_model_update_equations.png

My project includes the following files:

* ```MPC.cpp``` model predictive control implementation
* ```main.cpp``` initialization, state setup and waypoints creation

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

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Implementation Details

### The Model


The model used was the kinematic model. This model has its limitations since does not account for tire forces, longitudinal forces, lateral forces, inertia, gravity, air resistance, drag, mass, etc.
Nevertheless, for its simplicity and also the inherent capabilities of MPC that allow to constantly reevaluate the horizon at each time step make it an excellent choice for the task.

The **state** variables:

| variable | description       |
|----------|-------------------|
| x        | x-position        |
| y        | y-position        |
| psi      | heading           |
| v        | velocity          |
| cte      | cross-track error |
| epsi     | heading error     |

The model **actuators**:

| variable | description       |
|----------|-------------------|
| delta       | steering angle between [-25, 25] degrees       |
| a        | acceleration, throttle and brake pedals        |

The **update equations**:

![Kinematic Model - Update Equations][img2]

### Timestep Length and Elapsed Duration

The choice for the time step length N and duration dt, where the following:

| variable | value       |
|----------|-------------------|
| N        | 15        |
| dt        | 0.05        |

For my model, these values ended up working best and didn't have much performance penalty.
I also tried with N=10 and dt=0.05, but occasionally encounter a zig-zag behavior around the cte.

### Polynomial Fitting and MPC Preprocessing

The code for preprocessed the waypoints is contained in lines 102 through 120 of the file `main.cpp`.

Since coordinates were transformed into vehicle perspective, cross-track error and heading error calculations were simplified

### Model Predictive Control with Latency

This part of the code is contained in lines 129 through 142 of the file `main.cpp`.

To deal with the latency issue I applied a latency delay of a 100 ms to the kinematic
update equations, and use resulting values to initialize the MPC state.
The goal is to initialize MPC with the future state of the car after the 100 ms of latency.

## Video

Here's a [video of the project](https://youtu.be/5hHgTyoRvX4)

![MPC Project][img1]