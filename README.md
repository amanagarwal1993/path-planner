# Path Planning in Traffic
Self-Driving Car Engineer Nanodegree Program

### Goals
* In this project my goal was to safely navigate around a virtual highway with other traffic that is driving with a 50 MPH speed limit. 
* I was provided with the car's localization and sensor fusion data, and a sparse map list of waypoints around the highway. 
* The car tries to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. 
* The car avoids hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. 
* The car makes complete loops around the 6946m highway. Since the car is trying to go 50 MPH, it takes a little over 5 minutes to complete 1 loop. 
Also the car does not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## My Strategy
My first decision was to separate the project into three separate modules:
1. Signal processing and prediction
2. Behaviour planning
3. Trajectory Planning

The prediction module outputs easy-to-process information so that the behaviour planner can use it and decide how the car should behave. The behaviour planner itself cannot turn the car. Instead it gives out a one-word command and target speed to be followed and it is the job of the trajectory planner to implement that command. 

This way we separate the logic of the code so that it is much more flexible and maintainable. With this separation of concerns, we can easily make changes which would otherwise be extremely hard to do if the code was all tangled together like spaghetti.

#### Prediction
The prediction class takes as input the sensor fusion data and the car's current status (position, speed etc). With this, it tells you: 
1. Which cars are in which lane
2. What is the speed of the vehicle right in front of it in any of the lanes
3. Where each of the cars on the road will be after 2 seconds and 4 seconds.

Armed with this information, we know what's happening on the road and can plan the most optimal manoeuvre.

#### Behaviour Planning
This module has 2 objectives: reach the destination as quickly as possible, while staying safe and avoiding risky decisions. Therefore, it is optimized to only change lanes when it is advantageous to do so. It gives as output any one of these 3 behaviours which is to be implemented by the car:
1. Stay in lane at a target speed
2. Turn Left
3. Turn Right

Now here's the overall decision making process:
* If the car is currently executing a left- or right- turn and has not yet completed the manoeuvre, keep turning that way.
* Else, if the car is safely inside a lane, craft a new strategy...
* See if the car in front is fast enough or far enough that you can go at full speed (i.e near the speed limit). If yes, keep going in this lane at high speed!
* If the car in front is not fast enough, check if the lane (or lanes) right next to it is moving faster than current lane. If they're not faster, then just stay in lane and follow the car in front at its speed. It also slows down if the gap between the 2 cars is too small.
* If the other lanes are faster and current lane is too slow, then check if a **4-second window** exists which can allow the car to make a safe turn. As long as the window is not available, just stay in lane.
* As soon as there is a 4-second window, turn!
* Once in the next lane, repeat the above steps.

#### Trajectory Planning
This class is updated with the input of the current state of the vehicle, and the "plan" as commanded by the behaviour planner. It has a private `drive()` function which outputs the set of X,Y points forming the required trajectory. Then the main function simply sends the points back to the simulator and the whole cycle continues!
I used the Spline module to calculate smooth trajectories.

---
Now, how to run the project:
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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)
["x"] The car's x position in map coordinates
["y"] The car's y position in map coordinates
["s"] The car's s position in frenet coordinates
["d"] The car's d position in frenet coordinates
["yaw"] The car's yaw angle in the map
["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator
["previous_path_y"] The previous list of y points previously given to the simulator

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.


## License
**_Ask me for help, it's better than copying my code directly!_**
[GNU General Public License](http://choosealicense.com/licenses/gpl-3.0/#)
