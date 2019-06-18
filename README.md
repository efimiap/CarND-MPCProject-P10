# CarND-MPCProject-P10
CarND Term 2 Model Predictive Control (MPC) Project  
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

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Rubrics

**1. Compilation**  
The code compiles without errors

**2. Implementation**  
* The model  
The model is built according to the Kinematic model, following the equations taught in class  
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt    
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt    
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt   
v[t] = v[t-1] + a[t-1] * dt   
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt   
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt   

x,y - vehicle's coordinates, psi - orientation angle, v - velocity, cte - cross track error, epsi - psi error   

* The Actuators   
The actuators output acceleration and steering angle   
acc: Throttle ( positive ) and Brake (negative) values between [-1,1]   
delta: Steering angle     

* Timestep Length (N) and Elapsed Duration (dt)   

On this step it was essential to find the N value that didn't slow down our model or caused erratic behavior. When increasing N, it allowed projection into the future but also slowed down the model. The optimal value was the one that predicted up to the model's horizon. When decreasing N, it didn't allow sufficient prediction into the future, causing unpredictable behavior to the model. My approach was to choose values that kept the simulator's horizon at the same distance to the waypoints.   
The final values I chose are: N = 10 and dt = 0.1   

* Polynomial Fitting and MPC Preprocessing   

On main.cpp, before I fit a 3rd degree polynomial, I convert the waypoints to the vehicle's coordinates from map coordinates, as instructed. A similar approach was implemented at PID. This facilitates the implementation as x,y are on (0,0) and the vehicle's orientation is at 0.   

* Model Predictive Control with Latency    

For the actuator latency of 100 ms, I update the state of the vehicle according to the latency before passing it to the solver function. As a result the state values include the model and the delay interval. I use these instead of the initial values. (main.cpp)   

**3. Simulation:**   
The vehicle completes successfully a lap around the track



