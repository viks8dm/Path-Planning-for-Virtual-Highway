# Path-Planning for Virtual Highway
Self-Driving Car Engineer Nanodegree Program

---

[image1]: ./results/05MileMark_2.jpg "5-miles driving completion image"
[image2]: ./results/10MileMark.jpg "5-miles driving completion image"
[image3]: ./results/15MileMark.jpg "5-miles driving completion image"
[image4]: ./results/20MileMark.jpg "5-miles driving completion image"

--
![alt text][image2]
--

In this path-planning project of term-3 of the self-driving car nanodegree program by Udacity, goal was to design a path-planner, that generates smooth, safe paths for the car to follow along a 3-lane highway with traffic. Condition for a successful path planner is that it should be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

The project has been created using [path-planning framework](https://github.com/udacity/CarND-Path-Planning-Project) provided by Udacity.

The path-planner was successfully implemented to drive the car on a virtual highway with traffic using Udacity's [term-3 simulator](https://github.com/udacity/self-driving-car-sim/releases)

## Goals
The goals to achieve in this project were:

* The car is able to drive at least 4.32 miles without incident.
* The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.
* Max Acceleration and Jerk are not exceeded. The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
* Car does not have collisions. The car must not come into contact with any of the other cars on the road.
* The car stays in its lane, except for the time between changing lanes. The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.
* The car is able to change lanes. The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

## Result

My implementaton is focused on achieving the requirements listed above. The final implementation can successfully meet all requirements, as is evident from this [video](https://youtu.be/vhTjNlz-NDs)

On one of the occassions I left the car running beyond 5-miles and it went without incident for more than 20 miles. A screen capture at 15-miles and 20-miles is shown below.

--
![alt text][image3]
--
![alt text][image4]
--

#### limitations / incidents

Over multiple runs, I did not encounter any case where there was an incident before 5-mile mark. However, the incidents when they occurred, were limited to one particular type of cases:

* When a car is in the side lane within 1-2 meters of my-car (ahead or behind) and suddenly changes lane, this leads to collision. This is a typical scenario in highway driving when the other-car does not check blind spot and makes a lane-change. At this point, I am not sure how to tackle this issue.

## Discussion

The implementation in this repository achieves all goals, as listed above.

The algorithm follows general outline provided in the classroom discussion in the 'project-walkthrough' lecture. It starts in `/src/main.cpp`. I have added a `./src/CAR.cpp` and `./src/CAR.h` to define variables that are used repeatedly and for ease of separation of different aspects of my implementation, like parameter initialization, path-planning, cost-definition etc.

The overall path-planning section is divided into three parts (following similar nomenclature as in lectures):

### Data used from sensor-fusion

The localization data that is used from sensor-fusion parameterfor locating the main-car:

* ["x"] The car's x position in map coordinates
* ["y"] The car's y position in map coordinates
* ["s"] The car's s position in frenet coordinates
* ["d"] The car's d position in frenet coordinates
* ["yaw"] The car's yaw angle in the map
* ["speed"] The car's speed in MPH

For other car's, following information is used:

* `vx`: x-direction speed of other-car
* `vy`: y-direction speed of other-car
* `s`: other-car's s-position in frenet corrdinate
* `d`: other-car's d-position in frenet coordinate

### Prediction

In this section, map and sensor-fusion data is used to extract information about all other cars going in the same direction. Information like lane, s-coordinate, d-coordinate are extracted information related to:

* presence and relative-distance of other cars in my lane
* state of other cars in left or right lanes.

Using other-car speed and current position information, prediction is made on where the car might be one time step from now. This information and relative speed of the other car is used to assess lane-occupancy states of all cars one time step later which in turn is used to decide my-car's behavior, as in if my car should keep-lane or change-lane.

A cost parameter for left or right lane change is also computed based on position of other-cars in these lanes relative to my-car. This cost comes handy during behavior-planning if my-car's lane is occupied while both left and right lanes are free up to a pre-defined minimum front and rear gap value.

### Behavior-planning

Using computations and information gather in prediction phase, this section decides on what my-car should do, as in:

* keep-lane
	* slow-down
	* keep-following vehicle in front, if side lanes are occupied
	* increase speed or accelerate more
* change lane to left or right, if and where it is safe to do so

If decision is in favor of lane change, and my-car is in the center lane, then computed cost is used to decide which side to move. The cost is inversely proportional to front-gap and rear-gap in either lane. This means that if the next closet car is close to my-car then the cost will be higher compared to when it is far away from me.

My implementation for lane change is based on simple criteria which just checks for lack of cars in the adjacent lanes in a predefined space in front of and behind the car. It is possible to have very complex behavior planning algorithms that account for position of all other "visible" cars on the road and develop a plan based on how soon my-car will be farther ahead without volating any of the conditions listed in the "goals" above. However, I have not tried to implement any such algorithm.

### Trajectory generation

Once the behavior decision has been made, this section calculates the trajectory my-car should follow. For this purpose I am simply using the framework provided in the walk-through lecture where vehicle speed, recommended lane (from behavior-planning), current coordinates and previous path-points are used for trajectory generation.

Here, last two points of previous trajectory and three points at a far distance are to compute a spline. Then the spline points are transformed to my-car's coordinates, for simplicity. Use of last two points from previous trajectory helps smoothen the motion by maintainig continuity (or tangency).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


After execution of `./path_planning`, simulator should be opened and it should be started by selecting the path-planning project for visualization of car moving on virtual highway.