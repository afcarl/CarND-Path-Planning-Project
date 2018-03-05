#include <iostream>
#include <iostream>
#include <math.h>

#include "cost.h"
#include "vehicle.h"

const double MS_TO_MPH = 2.23694;

const double MPH_TO_MS = 0.44704;

CostFunction::CostFunction(Vehicle *v, vector<vector<double>> s){
  vehicle = v;
  sensor_fusion = s;
}

double CostFunction::compute(){
  
  //compute cost
  double cost = 0;
  cost += COMFORT * change_lane_cost();
  cost += EFFICIENCY * inefficiency_cost();
  cost += COLLISION * collision_cost();
  cost += buffer_cost();
  cost += EFFICIENCY * reference_velocity_cost();
  
  return cost;
}

double CostFunction::reference_velocity_cost() {
  double cost =0;
  if (!vehicle->collider.collision) {
    return 0;
  }
//  int end_lane = vehicle->trajectory.lane_end;
//  int start_lane = vehicle->trajectory.lane_start;
  double diff = (vehicle->collider.target_speed - vehicle->speed)/vehicle->collider.target_speed;
  cost = pow(diff, 2);
  return cost;
}

double CostFunction::change_lane_cost(){
  //Compute cost to change lane, penalizes lane Away fron the leftiest lane (fastest).
  int end_lane = vehicle->trajectory.lane_end;
  int start_lane = vehicle->trajectory.lane_start;
  double cost = 0;
  if (start_lane != end_lane) {
    cost += 1;
  } 
  
  return cost;
}

double CostFunction::inefficiency_cost(){
  //Always, the best efficiency is when the speed is closest to the limit
  double cost = 0;
  double diff = (49.5 - vehicle->reference.target_v)/49.5;
  cost = pow(diff,2);
  return cost;
}

double CostFunction::collision_cost(){  
  double cost = 0;
  if(vehicle->collider.collision){       
    double distance =  vehicle->collider.distance;
    //distance divided by the relative speed
    double time_to_collide = abs(vehicle->collider.distance)/(abs(vehicle->speed)*MPH_TO_MS);
    cost = exp(-pow(time_to_collide,2));
    //changing lane    
    if(vehicle->trajectory.lane_end != vehicle->trajectory.lane_start){
      if(time_to_collide > DESIRED_BUFFER){
        //safe to change lane
        cost /= 10;
      } 
    }
  }
  return cost;
}


double CostFunction::buffer_cost(){
  double cost = 0;

  if(vehicle->collider.closest_approach == 10000){
    return 0;
  }

  // check buffer
  double time_steps = abs(vehicle->collider.closest_approach)/(abs(vehicle->speed)*MPH_TO_MS);
  if(time_steps > DESIRED_BUFFER){
    return 0;
  }

  double multiplier = 1.0 - pow((time_steps / DESIRED_BUFFER),2);
  cost = multiplier * DANGER;
  if(vehicle->collider.closest_approach < 0){
    // car in the back
    cost /= 10;
  }       
  return cost;
}
