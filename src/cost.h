#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H
#include <iostream>
#include <vector>
#include <math.h>
#include "vehicle.h"

using namespace std;

class CostFunction {
public:
  const double SUFFICIANT_SPEED = pow(10, 5);
  const double COLLISION = pow(10, 6);
  const double DANGER = pow(10, 5);
  const double REACH_GOAL = pow(10, 5);
  const double COMFORT = pow(10, 4);
  const double EFFICIENCY = pow(10, 2);  //2

  const double DESIRED_BUFFER = 1;//1.5;
  const double PLANNING_HORIZON = 2;

  Vehicle *vehicle;
  vector<vector<double>> sensor_fusion;

  /**
   * Constructor
   */
  CostFunction(Vehicle *v, vector<vector<double>> s);

  /**
   * Compute cost
   */
  double compute();

  /*
   * Cost functions
   */
  double sufficiant_speed_cost();
  double change_lane_cost();
  double distance_goal_lane_cost();
  double inefficiency_cost();
  double collision_cost();
  double buffer_cost();
  double reference_velocity_cost();
  
};

#endif
