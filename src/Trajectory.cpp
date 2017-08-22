//
//  Trajectory.cpp
//  
//
//  Created by Aman Agarwal on 13/08/17.
//
//

#include "Trajectory.h"
#include "Eigen/Dense"
#include "spline.h"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


// TP = Trajectory Planner
TP::TP() {
  
}

TP::~TP() {
  cout << "Deleted" << endl;
}

// This just sets the private variables for TP, which reflect its current state. After which it doesn't really need many external inputs to work.
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


// This function outputs trajectory points which make the car stay in the same lane at constant speed. These points will go straight to the output to simulator
vector< vector<double> > TP::keep_lane() {
  
  double target_speed = this->target_speed;
  vector<double> nextx;
  vector<double> nexty;
  vector< vector<double> > points;
  double lane = this->target_lane;
  
  //cout << "\n\nCurrent position: " << this->car_x << " " << this->car_y << endl;
  
  int previous_size = previous_x.size();
  
  double lastx, lasty, secondlastx, secondlasty, last_theta;
  
  vector<double> ptsx;
  vector<double> ptsy;
  
  //cout << "Previous size: " << previous_size << endl;
  
  if (previous_size < 2) {
    lastx = this->car_x;
    lasty = this->car_y;
    last_theta = this->car_yaw;
    secondlastx = lastx - cos(last_theta);
    secondlasty = lasty - sin(last_theta);
    
    ptsx.push_back(secondlastx);
    ptsx.push_back(lastx);
    ptsy.push_back(secondlasty);
    ptsy.push_back(lasty);
    
    //vector<double> last_frenet = TP::getFrenet(this->car_x, this->car_y, this->car_yaw, this->maps_x, this->maps_y);
    //vector<double> last_frenet = {this->car_s, this->car_d};
    
    // Gotta space out the waypoints, so that the points aren't too strictly followed. Smoother driving.
    vector<double> waypoint1 = TP::getXY(this->car_s+40.0, lane, this->maps_s, this->maps_x, this->maps_y);
    vector<double> waypoint2 = TP::getXY(this->car_s+50.0, lane, this->maps_s, this->maps_x, this->maps_y);
    vector<double> waypoint3 = TP::getXY(this->car_s+80.0, lane, this->maps_s, this->maps_x, this->maps_y);
    vector<double> waypoint4 = TP::getXY(this->car_s+110.0, lane, this->maps_s, this->maps_x, this->maps_y);
    vector<double> waypoint5 = TP::getXY(this->car_s+140.0, lane, this->maps_s, this->maps_x, this->maps_y);
    
    ptsx.push_back(waypoint1[0]);
    ptsx.push_back(waypoint2[0]);
    ptsx.push_back(waypoint3[0]);
    ptsx.push_back(waypoint4[0]);
    ptsx.push_back(waypoint5[0]);
    
    ptsy.push_back(waypoint1[1]);
    ptsy.push_back(waypoint2[1]);
    ptsy.push_back(waypoint3[1]);
    ptsy.push_back(waypoint4[1]);
    ptsy.push_back(waypoint5[1]);
  }
  else {
    lastx = previous_x[previous_size-1];
    lasty = previous_y[previous_size-1];
    secondlastx = previous_x[previous_size-2];
    secondlasty = previous_y[previous_size-2];
    last_theta = atan2((lasty - secondlasty),(lastx - secondlastx));
    
    ptsx.push_back(secondlastx);
    ptsx.push_back(lastx);
    ptsy.push_back(secondlasty);
    ptsy.push_back(lasty);
    
    vector<double> last_frenet = TP::getFrenet(lastx, lasty, last_theta, this->maps_x, this->maps_y);

    vector<double> waypoint1 = TP::getXY(last_frenet[0]+40.0, lane, this->maps_s, this->maps_x, this->maps_y);
    vector<double> waypoint2 = TP::getXY(last_frenet[0]+50.0, lane, this->maps_s, this->maps_x, this->maps_y);
    vector<double> waypoint3 = TP::getXY(last_frenet[0]+85.0, lane, this->maps_s, this->maps_x, this->maps_y);
    vector<double> waypoint4 = TP::getXY(last_frenet[0]+100.0, lane, this->maps_s, this->maps_x, this->maps_y);
    vector<double> waypoint5 = TP::getXY(last_frenet[0]+125.0, lane, this->maps_s, this->maps_x, this->maps_y);
    
    ptsx.push_back(waypoint1[0]);
    ptsx.push_back(waypoint2[0]);
    ptsx.push_back(waypoint3[0]);
    ptsx.push_back(waypoint4[0]);
    ptsx.push_back(waypoint5[0]);
    
    ptsy.push_back(waypoint1[1]);
    ptsy.push_back(waypoint2[1]);
    ptsy.push_back(waypoint3[1]);
    ptsy.push_back(waypoint4[1]);
    ptsy.push_back(waypoint5[1]);
  }
  
  //cout << "Last one of current waypoints: " << lastx << " " << lasty << " Theta:" << last_theta << endl;
  //cout << "Theta " << last_theta << endl;

  //cout << "\n Shifted and rotated" << endl;
  for (int i=0; i<ptsx.size(); i++) {
    double shiftx = ptsx[i] - lastx;
    double shifty = ptsy[i] - lasty;
    
    ptsx[i] = (shiftx * cos(0-last_theta)) - (shifty * sin(0-last_theta));
    ptsy[i] = (shiftx * sin(0-last_theta)) + (shifty * cos(0-last_theta));
    
    //cout << ptsx[i] << " " << ptsy[i] << endl;
  }
  
  for (int i=0; i<previous_size; i++) {
    double px = previous_x[i];
    double py = previous_y[i];
    nextx.push_back(px);
    nexty.push_back(py);
    //cout << px << " " << py << endl;
  }
  
  //cout << "Spock" << endl;
  // Spline class. I'll call this instance spock, because... it's MY program!
  tk::spline spock;
  spock.set_points(ptsx, ptsy);
  // Now s has fitted a polynomial to ptsx and ptsy, and it will give us y for any x in LOCAL COORDINATES
  
  double targetx = 30.0;
  double targety = spock(targetx);
  double target_dist = sqrt((targetx*targetx) + (targety*targety));
  double N = target_dist / (0.02 * target_speed);
  
  //cout << "N = " << N << " target_dist: " << target_dist << "\n" << endl;
  double x_start = 0;
  //cout << "New waypoints: " << endl;
  for (int i=1; i<=50-previous_size; i++) {
    double xpoint = x_start + (target_dist / N);
    double ypoint = spock(xpoint);
    
    double xref = xpoint;
    double yref = ypoint;
    
    x_start = xpoint;
    
    double shiftx = xpoint + lastx;
    double shifty = ypoint + lasty;
    
    xpoint = xref*cos(0-last_theta) + yref*sin(0-last_theta) + lastx;
    ypoint = 0 - xref*sin(0-last_theta) + yref*cos(0-last_theta) + lasty;
    
    //cout << "After shift: " << xpoint << " " << ypoint << endl;
    
    nextx.push_back(xpoint);
    nexty.push_back(ypoint);
  }
  points.push_back(nextx);
  points.push_back(nexty);
  
  return points;
};


// The only public method accessible to the program. It will simply take the behavior command (eg keep lane, or switch left etc) and call on the respective private methods, and then output the resulting trajectory
vector< vector<double> > TP::executePlan(string command) {
  vector< vector<double> > next;
  
  /*
  if (car_speed > roadspeed - 2.0) {
    target_speed -= 0.1;
  }
  if (car_speed <= roadspeed - 4.0) {
    target_speed += 0.1;
  }
  */
  
  if(command == "keep") {
    
    if (car_d < 4.0) {
      target_lane = 2.0;
    }
    else if (car_d < 8.0) {
      target_lane = 6.0;
    } else {
      target_lane = 10.0;
    }
    
  } else if (command == "turnLeft") {
    
    if (car_d < 4.0) {
      target_lane = 2.0;
    }
    else if (car_d < 8.0) {
      target_lane = 2.0;
    } else {
      target_lane = 6.0;
    }
    
  } else if (command == "turnRight") {
    
    if (car_d < 4.0) {
      target_lane = 6.0;
    }
    else if (car_d < 8.0) {
      target_lane = 10.0;
    } else {
      target_lane = 10.0;
    }
    
  } else {
    string error = "Not a valid behavior plan for trajectory.";
    throw(error);
  }
  
  next = this->keep_lane();
  
  return next;
}
