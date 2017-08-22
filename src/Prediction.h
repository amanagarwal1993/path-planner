//
//  Prediction.hpp
//  
//
//  Created by Aman Agarwal on 20/08/17.
//
//

#ifndef Prediction_hpp
#define Prediction_hpp

#include <iostream>
#include <vector>

using namespace std;
#endif /* Prediction_hpp */

struct Car {
  int car_id;
  string lane;
  double s;
  double d;
  double speed;
  // Position of car after 2 and 4 seconds, in s and d
  vector<double> sec2;
  vector<double> sec4;
};

class Oracle {
private:
  vector< vector<Car> > all_cars;
  vector<Car> leaders;
public:
  Oracle() {};
  ~Oracle() {};
  void predict(vector< vector<double> > sensor_fusion, double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<double> maps_x, vector<double> maps_y);
  vector< vector<Car> > predictions();
  vector<Car> leading_cars();
};

