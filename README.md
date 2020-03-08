# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
# Writeup
## Implementation of the PID class
To start with I implemented the PID controller, which consists of the three functions
- Initialization
- Error update and
- Calculate the total error

In the main function the pid controller can than be used as follows for calculating the steering angle
```c++
// Calculate steering value
steer_pid.UpdateError(cte);
steer_value = steer_pid.TotalError();

// Keep steer value within boundaries
if(steer_value > 1.0) { steer_value = 1.0; }
if(steer_value < -1.0) { steer_value = -1.0; }
```
Respectively the speed pid can be implemented.

The core part of the PID controller implementation is the update of the error and the calculation of the total error
```c++
// 1. Update the error terms, where the variable error below is representing:
// - the offset in y direction from the middle of the lane for the steering angle PID-Controller
// - the delta between desired and actual speed for the speed PID-Controller
d_error = error - error_prev;   // calculate differential term
error_prev = error;
p_error = error;             // assign cte to proportional term
i_error += error;            // sum all ctes over time

// 2. Sum up the proportional term, the integral term and the differential term
// -(Kp * p_error + Ki * i_error + Kd * d_error);

```

The hyperparameters that are multiplied with the three different error terms can be explained as follows:

## Hyperparameters (Kp, Ki, Kd)

### Proportional scale of the error by Kp
A P-Controller (Proportional controller) is a controller where the error is scaled/multiplied by the negative of a constant Kp. This can already lead to good results until the point where the controller overshoots the target value and than starts oscillating from where on it gets worse and worse.

### Differential scale of the error by Kd
To overcome this problem a differential term can be added in addition to the proportional term, which leads to a multiplicatin of the error depending on the delta of the last two errors (gradient of the error). When the gradient is high, this term increases, when the gradient is low (already close to the target value) than this term decreases already, in contrast to the proportional term.

### Integral scale of the rror by Ki
To overcome any possible target bias, that might be inherently within the system, we can add an integral term, that sums up all errors from the past and therefore would react in case we are seemingly already on a target, but are actually off target by a certain bias.


The core part of getting any PID to run with reasonable results, is to tune the hyperparameters Kp, Ki and Kd. This can be done by different optimization methods such as twiddle, SGD or manual tuning.

I chose to manually tune the parameters to better understand how each change behaves in the simulator.
## Tuning hyperparameters

To tune the hyperparameters it is a good practice to start by setting all to zero and than gradually increase Kp, until the car starts oscillating. From my trial and error process this value is somewhere between 0.05 and 0.15 for Kp. For these values the car still oscillates, but does it quite steady.

From here on the differential term is increased to get rid of the small, steady oscillations. Depending a lot on what Kp I chose the values for Kd that worked are between 1.0 and 2.5.

For me the integral term was not really necessary, to achieve good results, nevertheless I added a very small value of 0.0001 to make sure that eventually any bias can be compensated.

For the speed controller I choose quite similar values, except the Kd, since I do not want to have sharp de/accelerations.

The final values for the steering PID, that I chose were:
- K = 0.1, D = 1.5, I = 0.0001

The final values for the speed PID that I chose were
- - K = 0.1, D = 0.0, I = 0.001

## Videos

I uploaded 4 videos with the following hyperparameters for the steering PID:
- Steering-PID: K = 0.5, D = 0.0, I = 0.0 | (Without Speed PID) --> Heavy oscillations of the car
- Steering-PID: K = 0.125, D = 0.0, I = 0.0 | (Without Speed PID) --> Small, steady oscillations
- Steering-PID: K = 0.1, D = 2.5, I = 0.0 | (Without Speed PID) --> Very aggresive turns because of high differential term
- Steering-PID: K = 0.1, D = 1.5, I = 0.0001 | Speed PID: K = 0.1, D = 0.0, I = 0.001 --> Final solution, which is quite smooth, except for the last couple of sharp corners, where it could be optimized

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
