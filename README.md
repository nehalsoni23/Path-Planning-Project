# Path Planning Project

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

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

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

---

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
	
### Reflection ###

* In order to make the car drive autonomously we need to generate waypoints which our car will be following. I am generating 50 points at a time as shown in project walkthrough video. Rather than generating it from 0 to 50 points every time, I am using previous points as a reference line. The previous points are the left points which our car hasn't covered in the last iteration. Also, in order to generate this points, spline library is being used. It is a really helpful resource for doing this project and creating smooth trajectories. http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single header file is really easy to use.

* Now, the simulator is sending the sensor fusion data of other cars on the road. 
```auto sensor_fusion = j[1]["sensor_fusion"];```
This data gives us the information consisting of s, d, x, y co-ordinates. From which we can calculate the speed of the car and the lane in which it is driving. If there is any car ahead of us in our lane and it is within permissible distance then a car will be decelerated at the rate of 5 m/s^2. The code for the same is written from line 341 to 368.

* To calculate the scenario where a lane change is desired, I have performed cost calculation of each possible lane. The code block to calculate the best lane to drive in written from 371 to 438. Following actions are performed to decide if it is safe to keep driving in a current lane with the desired velocity or to change the lane based on cost:

    *1. Based on which lane we are in, possible lanes are being calculated. In order to calculate the cost, there are certain conditions which contribute to the higher cost. If a possible lane is not current lane then high cost will be added.
    
    *2. For each such lane, it's average lane speed is being calculated. The code to find the average speed of the lane can be found from line 299 to 337. 
    
    *3. Now the next step is to calculate the best lane to drive. First, based on the average speed of the targeted lane from the 2nd point, cost calculation is performed. Lower the speed, higher the cost. Then, all the car_ids in the targeted lane are collected using `getCarsOfTargetLane` method. Then, using `getClosestDistanceFromCar` method, closest distance of another car to our car is calculated. Both the car ahead and/or behind of our car are considered to find the closest distance. If it is lesser than permissible distance then the high cost value will be incremented.
    
    *4. From all the possible lanes, the low cost lane will be picked and a car will be shifted to that lane. It is possible that even though there is a car very close to our car in the current lane, staying in current lane will be more plausible based on the other lane's speed and distance from other cars.
    
* The code lines from 434 to 438 contains the logic of speeding up when there is no vehicle in front of us at a distance of at least 30 meters and the path is clear to accelerate the vehicle in order to achieve the speed limit of 50 mph.

### Results ###

Following are the screenshots of a car driving in the simulator in different scenarios:

[image1]: ./data/decelerating_the_car.JPG "Decelerating the car"

[image2]: ./data/change_to_LL.JPG "Changing to left lane"

[image3]: ./data/change_to_ML.JPG "Changing to middle lane"

[image4]: ./data/change_to_RL.JPG "Changing to right lane"

[image5]: ./data/highest_miles.JPG "Highest miles without any accident"

<h3 align="center"> Decelerating when lane change is not possible and car ahead is close <h3>

![alt text][image1]


<h3 align="center"> Car changing from the middle lane to the left most lane <h3>

![alt text][image2]


<h3 align="center"> Car changing from the left lane to the middle lane <h3>

![alt text][image3]


<h3 align="center"> Car changing from the middle lane to the right most lane <h3>

![alt text][image4]


<h3 align="center"> Best distance without incident : 12.40 miles, the car was keep going on after that as well <h3>

![alt text][image5]
