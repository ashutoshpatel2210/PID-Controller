# CarND-Controls-PID
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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## PID Controller 

PID Controller computes control commands(steering) using error information (CTE(Cross track Error)). It consists of 3 terms

1. Propotional Term: This component establishes linear relationship with control value. Larger the value of propotional term or CTE, more impact on control value. The propotional term tries to minimize error at current time step. Larger the error value at currrent time, Larger the control value. 
    
   TE = - Kp*CTE
   
2. Integral Term: This component captures impact of sum of all observed errors during previous time frame or time stamp. Greater imbalance observed between positive and negative values in previous timeframes, more impact on control value. The Integral coefficiant tries to minimize the sum of all errors observed in past.

   TE = -Ki * sum(CTE)
   
3. Differential Term: This component captures impact of difference of error between current time stamp and previous time stamp. Greater difference observed between previous and current , more impact on control value. The Differential coefficiant tries to minimize this error.

   TE = -Kd * d(CTE)/dt
   
## Implementation

### The PID procedure follows what was taught in the lessons.

Implemented PID controller in src/pid.cpp file. The source code initializes Kp,Kd and Ki coefficients and errors. The code has implementation to update and total error in update() and TotalError() APIs. The code also captures minimum, maximum and average error of CTE. The Hyperparameter tuning is done manually. All data points with different hyper parameters captured in ./data folder for 1 lap(1000 iterations). I selected parameters based on minimum average value over 1000 iterations. The final paramerters are  0.27.0.01 and 2.5 for Kp, Ki and Kd.

## Reflection

### Describe the effect each of the P, I, D components had in your implementation.

1. Propotional Component: This component establishes linear relationship with control value and tries to minimize CTE proptionaly with each time step. I tried to measure the imapact of propotional term using (1,0,0) triplet. This term tries to put car in center line at each step. But if used only, car oscilates back and forth towards center line and then overshoots to off track.
    
   TE = - Kp*CTE

2. Integral Component: This component impacts on sum of all CTEs and tries to minimize CTE integrally over the time. I tried to measure the imapact of integral term using (0,1,0) triplet. This term tries to eliminate bias. But if used only, car is going into circles and then going out of track.
    
   TE = -Ki * sum(CTE)

3. Differential Component: This component tries to minimize CTE between previous and current time step. I tried to measure the imapact of differntial term using (0,0,1) triplet. But if used only, car is going off the track soon.  
    
   TE = -Kd * d(CTE)/dt

### Describe how the final hyperparameters were chosen.

The Hyperparameters are selected based on manual tuning. I selected parameters (1,1,1) but it is not keeping car on center line. I started paramters (0.2, 0, 3.0) as mentioned in course.  To reduce average error, i ran experiment by measuring average error with +/-0.05(propotional), +1/100 to + 1/1000 (Integral), -0.5 to - 1.0 (differntial). The below are summary of results collected for hyperparameters.

| No | Kp     | Ki    | Kd   | Min Error    | Max Error    | Average Error|
| -- |:------:|:-----:|:----:|:------------:|:------------:|:------------:|    
| 1  |   0.2  | 0     | 3.0  |  -0.0296     |    2.7336    | 0.446543     | 
| 2  |   0.2  | 0     | 2.5  | -0.0074      |  3.0257      | 0.43414      | 
| 3  |   0.15 | 0     | 2.5  | -0.0216 	    |  3.1859      | 0.547701     |
| 4  |   0.2  | 0.001 | 2.5  | 0.0001       |  2.7524      | 0.350967     |
| 5  | 0.2    | 0.0001| 2.5  | -0.0097      | 3.1511       | 0.428597     |  
| 6  | 0.2    | 0.01  | 2.5  | -0.0075      | 2.5491       | 0.333472     |
| 7  | 0.2    | 0.01  | 2    | 0            | 2.2485       | 0.340835     | 
| 8  | 0.2    | 0.01  | 2.7  | -0.0025      | 2.186        | 0.344935     |
| 9 | 0.25     | 0.01  | 2.5  | -0.0093     | 2.1781       | 0.327996     |
| 10| 0.27    |  0.001 | 2.5  | -0.0002     | 2.6491       | 0.295924     |

## Simulation

### The vehicle must successfully drive a lap around the track.

Vehicle is not going off the track in above all experiments. Vehicle stays on track for final selected params [0.27, 0.001, 2.5]. 
