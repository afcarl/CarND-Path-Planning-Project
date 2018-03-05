#include "vehicle.h"
#include <iostream>
#include <iostream>
#include <math.h>
#include "cost.h"

// mile per seconds to mile per hour
const double MS_TO_MPH = 2.23694;
// mile per hour to mile per second
const double MPH_TO_MS = 0.44704;

Vehicle::Vehicle(int lane, double target_speed){
    ref_speed = target_speed;
    ref_lane = lane;
}
  
void Vehicle::update_data(double ax, double ay, double as, double ad, double ayaw, double aspeed, int lane, double target_speed, double delta){
  //update raw data
  this->x = ax;
  this->y = ay;
  this->s = as;
  this->d = ad;
  this->yaw = ayaw;
  this->speed = aspeed;
  this->delta_time = delta;

  this->ref_speed = target_speed;
  this->ref_lane = lane;

  //clean data
  reset_data();
}

void Vehicle::reset_data(){
  //clean data

  //reset trajectory
  this->trajectory.lane_start = ref_lane;
  this->trajectory.lane_end = ref_lane;
  this->trajectory.target_speed = ref_speed;

  //reset update
  this->reference.ref_v = ref_speed;
  this->reference.lane = ref_lane;
  this->reference.target_v = 49.50;
  this->collider.collision = false;
  this->collider.distance = 10000;
  this->collider.closest_approach = 10000;
  this->collider.target_speed = 0;
}


void Vehicle::choose_next_state(vector<vector<double>> sensor){
  State current_state = state;
  vector<State> possible_states;

  //select reachable states
  possible_states.push_back(KL);
  if (state == KL) {
    if(ref_lane != 0){
      // check if a vehicle is ready to change lane to left.
      if(d<(2+4*(ref_lane)+2) && d>(2+4*(ref_lane)-2)){
        possible_states.push_back(PLCL);
      }
    }
    // check if a vehicle is ready to change lane to left.
    if(ref_lane != 2){
      if(d<(2+4*(ref_lane)+2) && d>(2+4*(ref_lane)-2)){
        possible_states.push_back(PLCR);
      }
    }
  } else if(state == PLCL){
    // prepare lane change left
    possible_states.push_back(LCL);
    possible_states.push_back(PLCL);
  } else if(state == PLCR){
    // prepare lane change right
    possible_states.push_back(LCR);
    possible_states.push_back(PLCR);
  }

  // compute cost of all possible states
  State min_state = KL;
  double min_cost = 10000000;
  for (int i = 0; i < possible_states.size(); i++) {
    State possible_state = possible_states[i];
    // update states to simulate the given possible state
    reset_data();
    realize_next_state(possible_state, sensor);
    CostFunction cost = CostFunction(this, sensor);
    double value = cost.compute();
    if (value < min_cost) {
      min_state = possible_state;
      min_cost = value;
    }
  }

  // update state with the state with the minimum cost
  state = min_state;
  reset_data();
  realize_next_state(state, sensor);

  // update reference velocity
  CostFunction cost = CostFunction(this, sensor);
  float v = cost.compute();

  std::cout << "Next state is " << state << " with cost " << min_cost << endl;

  // modify reference velocity
  if (!collider.collision && ref_speed < reference.target_v && ref_speed < 49.5) {
    reference.ref_v += 0.224;
  } else if (ref_speed > reference.target_v && ref_speed > 0) {
    reference.ref_v -= 0.224;
  }
}

void Vehicle::realize_next_state(State next_state, vector<vector<double>> sensor_fusion) {
  state = next_state;

  switch(state) {
    case KL: {
      trajectory.lane_start = ref_lane;
      trajectory.lane_end = ref_lane;
      reference.lane = ref_lane;
      break;
    }
    case PLCL: {
      trajectory.lane_start = ref_lane;
      trajectory.lane_end = ref_lane - 1;
      reference.lane = ref_lane;
      break;
    }
    case LCL: {
      trajectory.lane_start = ref_lane;
      trajectory.lane_end = ref_lane - 1;
      reference.lane = ref_lane - 1;
      break;
    }
    case PLCR: {
      trajectory.lane_start = ref_lane;
      trajectory.lane_end = ref_lane + 1;
      reference.lane = ref_lane;
      break;
    }
    case LCR: {
      trajectory.lane_start = ref_lane;
      trajectory.lane_end = ref_lane + 1;
      reference.lane = ref_lane + 1;
      break;
    }
    default:
      std::cout << "ERROR: invalid state" << std::endl;
  }

  double target_speed_front = 0;
  double target_distance_front = 10000;
  double target_speed_lane_front = 0;
  double target_distance_lane_front = 10000;
  double target_speed_lane_back = 0;
  double target_distance_lane_back = -10000;

  // compute collision on start and end lane
  for (int i = 0; i < sensor_fusion.size(); i++) {
    float car_d = sensor_fusion[i][6];

    // safety check for speed of car which is in the same lane
    if((car_d < (2+4*(trajectory.lane_start)+2) && car_d > (2+4*(trajectory.lane_start)-2))){
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += ((double) delta_time*check_speed);
      // check s values greater than mine and s gap
      double dist_to_collision = (check_car_s - s);
      if((check_car_s >= s) && (dist_to_collision < 30)){
        if(target_distance_front > dist_to_collision){
          target_speed_front = check_speed*MS_TO_MPH-2;
          target_distance_front = dist_to_collision;
        }
      }
    }

    // check if a target car is in the end lane.
    if (car_d < (2+4*(trajectory.lane_end)+2) && car_d > (2+4*(trajectory.lane_end)-2)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += ((double) delta_time*check_speed);
      //check s values greater than mine and s gap
      double dist_to_collision = (check_car_s - s);      
      if((trajectory.lane_end != trajectory.lane_start
          && (abs(dist_to_collision) < 30))
          || ((check_car_s >= s) && (dist_to_collision < 30))){

        if(collider.distance > abs(dist_to_collision)){
          collider.distance = abs(dist_to_collision);
          collider.collision = true;
          collider.closest_approach = abs(dist_to_collision);
          collider.target_speed = check_speed*MS_TO_MPH;

          if(abs(dist_to_collision) > 30){
            //change target speed
            if(check_car_s >= s){
              //car in front
              reference.target_v = check_speed*MS_TO_MPH-2;
              if(target_distance_lane_front > dist_to_collision){
                target_speed_lane_front = check_speed*MS_TO_MPH;
                target_distance_lane_front = dist_to_collision;
              }
            } else {
              //car in back
              reference.target_v = check_speed*MS_TO_MPH+2;
              if(target_distance_lane_back < dist_to_collision){
                target_speed_lane_back = check_speed*MS_TO_MPH;
                target_distance_lane_back = dist_to_collision;
              }
            }
          }
        }
      }
      else if(!collider.collision && collider.closest_approach > dist_to_collision){
        collider.closest_approach = dist_to_collision;
        collider.target_speed = check_speed*MS_TO_MPH;
      }
    }   
  }

  // check if speed is safe
  if (state == PLCL || state == PLCR) {
    // When there is another car behind the car and on the next lane,
    // the reference velocity will be that's speed.
    if (target_speed_lane_back != 0 && reference.target_v < target_speed_lane_back) {
      reference.target_v = target_speed_lane_back;
    }
    // When there is another car in front of the car and on the next lane,
    // the reference velocity will be that's speed.
    if (target_speed_lane_front != 0 && reference.target_v > target_speed_lane_front) {
      reference.target_v = target_speed_lane_front;
    }
  }  

  // When there is another car in front of the car, the car have to following the speed of one.
  if (target_speed_front != 0 && reference.target_v > target_speed_front) {
    reference.target_v = target_speed_front-2;
  }
}
