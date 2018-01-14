#ifndef CAR_H
#define CAR_H

#include <vector>
#include <math.h>
#include "spline.h"
#include <iostream>


using namespace std;

/***************************************************************/
constexpr double pi() { return M_PI; }

/***************************************************************/
// define constants
#define LARGE_NUMBER 1e6
#define TIME_STEP 0.02
#define MAX_POINTS 50 //50

#define MAX_SPEED 49.75 //mph
#define MAX_ACC 0.22 // 0.224 // mph/sec
#define MIN_FRONT_GAP 30 //meters
#define MIN_REAR_GAP 12.5 // meters

#define FRONT_COST_FACTOR 100
#define REAR_COST_FACTOR 25
#define EPS 1e-3

// 1 = mid-lane, 0=left lane, 1=right lane

/***************************************************************/
// data structures
struct nextXY {
    vector<double> x; // cartesian x-coordinate
    vector<double> y; // cartesian y-coordinate
};

/***************************************************************/
class CAR {
 public:
    // variables
    int idx;          // car ID
    int lane;       // car lane, 0--left-lane, 1--mid-lane, 2--right-lane
    double s;       // s-coordinate (frenet)
    float d;        // d-coordinate (frenet)
    double speed;   // vehicle speed (m/s)
    
    bool myLane;    // false (0) implies free-lane
    bool leftLane;  // false implies free-lane
    bool rightLane; // false implies free-lane
    
    double leftCost;   // cost to change to left lane
    double rightCost;   // cost to change to right lane
    double left_front_gap;
    double left_rear_gap;   //negative number
    double right_front_gap;
    double right_rear_gap;  // negative number
    
    // constructor & destructor
    CAR();
    virtual ~CAR();
    
    // get Euclidean distance
    double getEucl(double x, double y);
    //update my car's state
    void updateMyCarState(int i, double s, double d, double speed);
    //update other car's state per sensor data
    void updateOtherCarState(int i, double s, float d, double vx, double vy);
    
    //initialize car lane number to default value
    void initCarLane();
    // find which lane a car is driving in
    void getCarLane(float d);
    
    //initialize lane-occupancy state w.r.t. my car
    void initLaneState() ;
    // update lane occupancy state based on position of other cars
    void updateLaneState(int obs_lane, double obs_future_s, int myCar_lane, double myCar_s);
    
    // plan car behavior (modify speed or chage lane) per car & traffic details
    double behaviorPlanner(double other_car_speed, double ref_vel, double left_cost, double right_cost);
    
    //initialize path planning cost parameters
    void initCost();
    // compute left/right lane change cost
    void laneChangeCost(CAR someCar, double lane);

};
#endif /* CAR_H */
