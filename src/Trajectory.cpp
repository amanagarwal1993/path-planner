//
//  Trajectory.cpp
//  
//
//  Created by Aman Agarwal on 13/08/17.
//
//

#include "Trajectory.hpp"
using namespace std;

TP::TP() {
  
}

TP::~TP() {
  cout << "Deleted" << endl;
}

void TP::Update(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, double target_speed, vector<double> previous_x, vector<double> previous_y, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
  this->car_x = car_x;
  this->car_y = car_y;
  this->car_s = car_s;
  this->car_d = car_d;
  this->car_yaw = car_yaw;
  this->car_speed = car_speed;
  this->previous_x = previous_x;
  this->previous_y = previous_y;
  this->target_speed = target_speed;
  this->maps_s = maps_s;
  this->maps_x = maps_x;
  this->maps_y = maps_y;
}

vector< vector<double> > TP::keep_lane() {
  double min_dist = this->target_speed / 5;
  vector<double> nextx;
  vector<double> nexty;
  vector< vector<double> > points;
  
  for (int i=1; i<16; i++) {
    vector<double> point = getXY((this->car_s + i*min_dist), this->car_d, this->maps_s, this->maps_x, this->maps_y);
    nextx.push_back(point[0]);
    nexty.push_back(point[1]);
  }
  
  points.push_back(nextx);
  points.push_back(nexty);
  
  return points;
};

vector< vector<double> > TP::turn_left() {
  
};

vector< vector<double> > TP::turn_right() {
  
};

vector< vector<double> > TP::keep_turning() {
  
};


vector< vector<double> > TP::executePlan(string command) {
  vector< vector<double> > next(0);
  
  switch (command) {
    case "keep":
      next = keep_lane();
      break;
      
    case "turnLeft":
      next = turn_left();
      break;
      
    case "turnRight":
      next = turn_right();
      break;
      
    case "keepTurning":
      next = keep_turning();
      break;
      
    default:
      string error = "Not a valid behavior plan for trajectory."
      throw(error);
      break;
  }
  
  return next;
}