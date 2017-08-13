//
//  Trajectory.hpp
//  
//
//  Created by Aman Agarwal on 13/08/17.
//
//

#ifndef Trajectory_hpp
#define Trajectory_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include <uWS/uWS.h>
constexpr double pi() { return M_PI; }
#endif /* Trajectory_hpp */

using namespace std;

double metersPerSecond(double mph) {
  return (mph * 0.447);
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  int prev_wp = -1;
  
  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }
  
  int wp2 = (prev_wp+1)%maps_x.size();
  
  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);
  
  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
  
  double perp_heading = heading-pi()/2;
  
  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);
  
  return {x,y};
  
}


class TP {
private:
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  double target_speed;
  vector<double> maps_s;
  vector<double> maps_x;
  vector<double> maps_y;
  vector<double> previous_x;
  vector<double> previous_y;
  vector< vector<double> > keep_lane();
  vector< vector<double> > turn_left();
  vector< vector<double> > turn_right();
  vector< vector<double> > keep_turning();

public:
  TP();
  
  void Update(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, double target_speed, vector<double> previous_x, vector<double> previous_y, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);
  
  ~TP();
  
  vector< vector<double> > executePlan(string command);
};
