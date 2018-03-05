#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>


using namespace std;

enum State {
  KL,   // keep lane
  LCL,  // lane change left
  LCR,  // lane change right
  PLCL, // prepare lane change left
  PLCR  // prepare lane change right
};

class Vehicle {
 public:

  // car state
  State state = KL;

  struct reference {
    double ref_v = 0;
    double target_v = 49.50;
    int lane = 0;
  } reference;

  struct collider {
    bool collision = false;
    double distance = 0;
    double closest_approach = 1000;
    double target_speed = 0;
  } collider;

  struct trajectory {
    int lane_start = 0;
    int lane_end = 0;
    double target_speed=0;
  } trajectory;

  double ref_speed = 0;
  int ref_lane = 0;
  
  double x = 0;
  double y = 0;
  double s = 0;
  double d = 0;
  double yaw = 0;
  double speed = 0;

  double delta_time = 0;

  
  /**
  * Constructor
  */
  Vehicle(int lane, double target_speed);

  // reset data
  void reset_data();
  // update car
  void update_data(double ax, double ay, double as, double ad, double ayaw, double aspeed,
                   int lane, double target_speed, double delta);
  
  // get next state
  void get_next_state(vector<vector<double>> sensor);
  // realize next state
  void realize_next_state(State state, vector<vector<double>> sensor_fusion);
};

#endif
