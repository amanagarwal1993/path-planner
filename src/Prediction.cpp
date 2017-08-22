//
//  Prediction.cpp
//  
//
//  Created by Aman Agarwal on 20/08/17.
//
//

#include "Prediction.h"
#include "Trajectory.h"

using namespace std;

vector< vector<Car> > Oracle::predictions() {
  return this->all_cars;
}

void Oracle::predict(vector< vector<double> > sensor_fusion, double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<double> maps_x, vector<double> maps_y) {
  
  vector< vector<Car> > lanes(3);
  
  for (int i=0; i < sensor_fusion.size(); i++) {
    int id = sensor_fusion[i][0];
    double x = sensor_fusion[i][1];
    double y = sensor_fusion[i][2];
    double vx = sensor_fusion[i][3] * 0.447;
    double vy = sensor_fusion[i][4] * 0.447;
    double s = sensor_fusion[i][5];
    double d = sensor_fusion[i][6];
    double theta = atan2(vy, vx);
    
    vector<double> p2 = TP::getFrenet(x + (vx * 2.0), y + (vy * 2.0), theta, maps_x, maps_y);
    
    vector<double> p4 = TP::getFrenet(x + (vx * 4.0), y + (vy * 4.0), theta, maps_x, maps_y);
    
    Car new_car;
    new_car.car_id = id;
    new_car.s = s;
    new_car.d = d;
    new_car.speed = sqrt(vx*vx + vy*vy) / 0.447;
    new_car.sec2 = p2;
    new_car.sec4 = p4;
    
    if (d > 0.0 && d <= 4.0) {
      new_car.lane = "L";
      lanes[0].push_back(new_car);
    } else if (d <= 8.0) {
      new_car.lane = "C";
      lanes[1].push_back(new_car);
    } else if (d > 8.0) {
      new_car.lane = "R";
      lanes[2].push_back(new_car);
    }
  }
  
  this->all_cars = lanes;
  
  this->leaders.clear();
  for (int i=0; i<3; i++) {
    vector<Car> lane = lanes[i];
    Car leading;
    
    if (lane.size() > 0) {
      double diff = 100000;
      for (int i=0; i<lane.size(); i++) {
        if ((lane[i].s - car_s) > 2.0 && (lane[i].s - car_s) < diff) {
          leading = lane[i];
          diff = lane[i].s - car_s;
        }
      }
    }
    else {
      leading.car_id = -1;
      leading.s = 10000;
      leading.d = -10;
      leading.speed = 100.0;
    }
    this->leaders.push_back(leading);
  }
  
  
}

vector<Car> Oracle::leading_cars() {
  return this->leaders;
}
