#include "CAR.h"

//constexpr double pi() { return M_PI; }
double deg2rad_car(double x) { return x * pi() / 180; }

/***************************************************************/
// constructor
CAR::CAR() {}
// destructor
CAR::~CAR() {}

/***************************************************************/
/***************************************************************/
// update my car's state
void CAR::updateMyCarState(int id, double s, double d, double v) {
    this->idx = id;
    this->s = s;
    this->d = d;
    this->speed = v;
}
/***************************************************************/
/***************************************************************/
// update other car's state per sensor data
void CAR::updateOtherCarState(int id, double s, float d, double vx, double vy) {
    this->idx = id;
    this->s = s;
    this->d = d;
    this->speed = getEucl(vx, vy);
}
/***************************************************************/
/***************************************************************/
//compute Euclidean distance
double CAR::getEucl(double x, double y) {
    return sqrt(x*x + y*y);
}
/***************************************************************/
/***************************************************************/
//initialize car lane number to default value
void CAR::initCarLane() {
    lane = -1;
}
/***************************************************************/
/***************************************************************/
// find which lane a car is driving in
void CAR::getCarLane(float d) {
    initCarLane();;
    if(d>0 && d<4) { lane = 0; } // left lane
    else if(d>4 && d<8) { lane = 1; } // center lane
    else if(d>8 && d<12) {lane = 2; } // right lane
}

/***************************************************************/
/***************************************************************/
//initialize lane-occupancy state w.r.t. my car
void CAR::initLaneState() {
    myLane = false;
    leftLane = false;
    rightLane = false;
}

/***************************************************************/
/***************************************************************/
// update lane occupancy state based on position of other cars
void CAR::updateLaneState(int obs_lane, double obs_future_s, int myCar_lane, double myCar_s) {
    
    //check if side-lane is occupied (look front and back)
    bool sideLaneStatus = obs_future_s - myCar_s < MIN_FRONT_GAP && myCar_s - obs_future_s < MIN_REAR_GAP;
    
    // update lane occupancy status based on lane of other-car
    if (obs_lane == myCar_lane) { // car in my lane
        myLane = myLane || (obs_future_s > myCar_s && obs_future_s - myCar_s < MIN_FRONT_GAP);
    } else if (obs_lane == myCar_lane -1) { // car in lane left to me
        leftLane = leftLane || sideLaneStatus;
    } else if (obs_lane == myCar_lane + 1) { // car in lane right to me
        rightLane = rightLane || sideLaneStatus;
    }
}

/***************************************************************/
/***************************************************************/
// plan car behavior (modify speed or chage lane) per car & traffic details
double CAR::behaviorPlanner(double other_car_speed, double ref_vel, double left_cost, double right_cost) {
    double delta_speed = 0.0;
    
    if (myLane) { //car ahead in my lane
        if (lane==1 && !leftLane && !rightLane) {
            // both size lanes are free
            if (left_cost<right_cost) {
                this->lane--; // go to left lane
            } else {
                this->lane++;
            }
        } else if (!leftLane && lane > 0) { // no car in left lane
            this->lane--; // go to left lane, if left exists
            //cout << "GO LEFT <<<<<<<<< " << endl;
        } else if (!rightLane && lane !=2) { // no car in right lane
            this->lane++; // mover to right lane, if right exists
            //cout << "GO RIGHT >>>>>>>>>>> " << endl;
        } else { // reduce speed, cannot change lane
            //modified to reduce back and forth swinging motion
            delta_speed -= MAX_ACC * (fabs(speed - other_car_speed)/MAX_SPEED);
            //cout << "SLOWWWWWWWWWWW ..............." << endl;
        }
    }
    else { // no car in my lane, close to me
        /*
        if (speed>0 && speed<=0.5*MAX_SPEED) {
            delta_speed += MAX_ACC;
        } else {
            delta_speed += 0.75*MAX_ACC;
        }
        */
        if (ref_vel < MAX_SPEED) {
            delta_speed += MAX_ACC;
            //cout << "FASTTTTTTTTTTTTTTT ++++++++++++++" << endl;
        }
        
    }
    // return speed difference value
    return delta_speed;
}
/***************************************************************/
/***************************************************************/
//initialize path planning cost parameters
void CAR::initCost() {
    leftCost = LARGE_NUMBER - 1;
    rightCost = LARGE_NUMBER;
    left_front_gap = LARGE_NUMBER;
    left_rear_gap = -LARGE_NUMBER; //negative number
    right_front_gap = LARGE_NUMBER;
    right_rear_gap = -LARGE_NUMBER; // negative number
}
/***************************************************************/
/***************************************************************/
// compute left/right lane change cost
void CAR::laneChangeCost(CAR someCar, double lane) {
    double car_gap = someCar.s - s;
    int lane_diff = lane - someCar.lane;
    
    if (lane_diff == 1) { //other-car in left lane
        if (car_gap>0 && car_gap<left_front_gap) { // car ahead & closer
            this->left_front_gap = car_gap;
        } else if (car_gap<0 && car_gap>left_rear_gap) {//car behind and closer
            this->left_rear_gap = car_gap; // negative number
        }
    }
    else if (lane_diff == -1) { //other-car in right lane
        if (car_gap>0 && car_gap<right_front_gap) { // car ahead & closer
            this->right_front_gap = car_gap;
        } else if (car_gap<0 && car_gap>right_rear_gap) {//car behind and closer
            this->right_rear_gap = car_gap; // negative number
        }
    }
    
    this->leftCost = 1/(lane+EPS) + FRONT_COST_FACTOR/left_front_gap - REAR_COST_FACTOR/left_rear_gap;
    this->rightCost = 1/(2-lane+EPS) + FRONT_COST_FACTOR/right_front_gap - REAR_COST_FACTOR/right_rear_gap;
}
