#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"
#include "Prediction.h"
#include "Trajectory.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int currentLane(double car_d) {
  if (car_d > 0.0 && car_d <= 4.0) {
    return 0;
  }
  else if (car_d <= 8.0) {
    return 1;
  }
  else if (car_d > 8.0) {
    return 2;
  }
  else return -1;
}

// Cars available to switch to (i.e legal)
vector< int > availableLanes(double car_d) {
  if (car_d > 0.0 && car_d <= 4.0) {
    return {1};
  }
  else if (car_d <= 8.0) {
    return {0,2};
  }
  else if (car_d > 8.0) {
    return {1};
  }
};


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  TP trajectory_planner;
  
  double target_speed = 0.0;
  bool prepare_for_turn = false;
  string plan = "keep";
  int previous_lane = 0;
  
  h.onMessage([&trajectory_planner,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &target_speed, &plan, &prepare_for_turn, &previous_lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          
          car_yaw = deg2rad(car_yaw);
          car_speed = TP::metersPerSecond(car_speed);

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          int n_cars = sensor_fusion.size();
          
          
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          vector< vector<double> > next;
          
          // Predictions
          Oracle oracle;
          oracle.predict(sensor_fusion, car_x, car_y, car_s, car_d, car_yaw, car_speed, map_waypoints_x, map_waypoints_y);
          
          
          // Behavior module begins
          // Objectives:
          // 1) Maintain target speed
          // 2) Avoid collisions from left, right, front
          
          vector< vector<Car> > lanes = oracle.predictions();
          
          int this_lane = currentLane(car_d);
          auto available_lanes = availableLanes(car_d);
          
          auto leaders = oracle.leading_cars();
          
           //Check previous plan
          if (plan == "turnRight" && car_d < (1.5 + previous_lane*4.0)) {
            plan = "turnRight";
          }
          else if (plan == "turnLeft" && car_d > (previous_lane*4.0 - 1.0)) {
            plan = "turnLeft";
          }
          else {
            previous_lane = this_lane;
            
            // If the leading vehicle is too far ahead or fast enough, go at speed limit
            if ((abs(leaders[this_lane].s - car_s) >= 25.0) || (leaders[this_lane].speed > 45.0)) {
              // Go at speed limit
              cout << "Go full speed!" << endl;
              trajectory_planner.roadspeed = 50.0;
              prepare_for_turn = false;
              plan = "keep";
            }
            else {
              trajectory_planner.roadspeed = leaders[this_lane].speed / 0.447;
              // Check which is fastest
              int high_speed_lane = this_lane;
              for (int i=0; i<available_lanes.size(); i++) {
                if (oracle.leading_cars()[available_lanes[i]].speed > leaders[high_speed_lane].speed) {
                  high_speed_lane = available_lanes[i];
                }
              }
              
              cout << "Fastest lane: " << high_speed_lane << endl;
              
              // If current lane is still the fastest lane
              if (high_speed_lane == this_lane) {
                cout << "Just stay in lane." << endl;
                if (abs(leaders[this_lane].s - car_s) <= 15.0) {
                  trajectory_planner.roadspeed = (leaders[this_lane].speed/0.447) - 5.0;
                }
                prepare_for_turn = false;
                plan = "keep";
              }
              // Else we must change lane
              else {
                prepare_for_turn = true;
                bool gap_open = true;
                  cout << "Check if there's an open gap for turning" << endl;
                  // car's rough position after 4 seconds
                  double future_s1 = car_s + (car_speed * 2.0);
                  double future_s2 = car_s + (car_speed * 4.0);
                  
                  for (int k=0; k < lanes[high_speed_lane].size(); k++) {
                    Car other = lanes[high_speed_lane][k];
                    if (abs(other.s - car_s) < 15.0 || abs(other.sec4[0] - future_s2) < 15.0 || abs(other.sec2[0] - future_s1) < 15.0) {
                      gap_open = false;
                    }
                  }
                  // Change the plan
                  if (gap_open) {
                    trajectory_planner.roadspeed = leaders[high_speed_lane].speed / 0.447;

                    if (high_speed_lane < this_lane) {
                      plan = "turnLeft";
                      cout << "Turn Left" << endl;
                    }
                    else {
                      plan = "turnRight";
                      cout << "Turn Right" << endl;
                    }
                  } else {
                    cout << "Can't turn yet; another car in range" << endl;
                    plan = "keep";
                  }
                }
              }
            }
          
          //cout << "Plan: " << plan << endl;
          //cout << "Left lane: " << leaders[0].speed/0.447 << " Center lane: " << leaders[1].speed/0.447 << " Right lane: " << leaders[2].speed/0.447 << endl;
          
          if (car_speed / 0.447 >= trajectory_planner.roadspeed - 7.5) {
            target_speed -= 0.225;
          }
          else if (car_speed / 0.447 < trajectory_planner.roadspeed - 9.5) {
            target_speed += 0.15;
          }
          
          trajectory_planner.Update(car_x, car_y, car_s, car_d, car_yaw, car_speed, target_speed, previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          next = trajectory_planner.executePlan(plan);
          
          next_x_vals = next[0];
          next_y_vals = next[1];
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
