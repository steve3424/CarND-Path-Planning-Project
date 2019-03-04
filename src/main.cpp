#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

	  // target lane and velocity
	  int lane = 1; // 0, 1, or 2
	  double ref_v = 49.5; // mph
	  int a = 1; // 1 for accelarate, -1 for decelerate
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"]; //mph

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

	  // create list of cars in front of us AND in the same lane
	  // get list of cars in front in same lane
	  vector<vector<double>> cars_in_front;
	  for (int i = 0; i < sensor_fusion.size(); i++) {
		double s_val = sensor_fusion[i][5];
		double d_val = sensor_fusion[i][6];
		if ((d_val > 4*lane && d_val < 4*lane+4) && s_val>car_s) {
			cars_in_front.push_back(sensor_fusion[i]);	
		}
	  }  
	  // get closest car in front of us
	  vector<double> lead_car;
	  if (cars_in_front.size() > 0) {
		lead_car = cars_in_front[0];
		for (int i = 0; i < cars_in_front.size(); i++) {
			double current_s = cars_in_front[i][5];
			if (current_s < lead_car[5]) {
				lead_car = cars_in_front[i];
			}
		}

		// check if lead car is close enough to follow
		double lead_s = lead_car[5];
		if (lead_s - car_s < 40.0) {
			double lvx = lead_car[3];
			double lvy = lead_car[4];
			double lead_v = sqrt(lvx*lvx + lvy*lvy) *2.24;
			// change 'a' to decelerate, change ref_v to lead_v
			a = -1;
			ref_v = lead_v;
		}
	  }  
	  
	  // reference variables
	  int previous_path_size = previous_path_x.size();
	  double pos_x = car_x;
	  double pos_y = car_y;
	  double yaw = deg2rad(car_yaw);
	  double current_v = car_speed;
	  // if there is a previous path available, get speed at the end of that path
	  if (previous_path_size == 1) {
	  	current_v = (distance(car_x, car_y, previous_path_x[0], previous_path_y[0])/0.02)*2.24;
	  } else if (previous_path_size > 1) {
		double x1 = previous_path_x[previous_path_size - 2];
		double y1 = previous_path_y[previous_path_size - 2];
		double x2 = previous_path_x.back();
		double y2 = previous_path_y.back();
	  	current_v = (distance(x1, y1, x2, y2)/0.02)*2.24; // mph
	  }

	  // next path values
	  vector<double> next_x_vals;
	  vector<double> next_y_vals;

	  // add previous path to next path vals
	  for (int i = 0; i < previous_path_size; i++) {
	  	next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	  }

	  // vals to fit spline
	  vector<double> ptsx;
	  vector<double> ptsy;

	  // start spline fit with current pos or end of previous path pos 
	  if (previous_path_size < 2) {
		double prev_car_x = car_x - cos(car_yaw);
		double prev_car_y = car_y - sin(car_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(car_x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(car_y);
	  } else {
	  	pos_x = previous_path_x.back();
		pos_y = previous_path_y.back();

		double pos_x_prev = previous_path_x[previous_path_size - 2];
		double pos_y_prev = previous_path_y[previous_path_size - 2];
		yaw = atan2(pos_y-pos_y_prev, pos_x-pos_x_prev);

		ptsx.push_back(pos_x_prev);
		ptsx.push_back(pos_x);

		ptsy.push_back(pos_y_prev);
		ptsy.push_back(pos_y);
	  }

	  // get points in frenet coordinates to fit spline to middle of lane
	  for (int i = 1; i <= 3; i++) {
	  	double next_s = car_s + 30*i;
		double next_d = 2+4*lane;
		vector<double> XY = getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
		ptsx.push_back(XY[0]);
		ptsy.push_back(XY[1]);
	  }

	  // transform spline points to vehicle coordinates
	  for (int i = 0; i < ptsx.size(); i++) {
	  	double shift_x = ptsx[i] - pos_x;
		double shift_y = ptsy[i] - pos_y;

		ptsx[i] = (shift_x*cos(0-yaw)-shift_y*sin(0-yaw));
		ptsy[i] = (shift_x*sin(0-yaw)+shift_y*cos(0-yaw));
	  }

	  // fit spline to center of lane
	  tk::spline s;
	  s.set_points(ptsx, ptsy);

	  // generate points along as path
	  double target_x = 30.0;
	  double target_y = s(target_x);
	  double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
	  double total_x = 0; // holds total distance from pos_x to next point 
	  double prev_v = 0.02*current_v/2.24; // in meters
	  double accel = 0.003*a;
	  

	  for (int i = 0; i < 50-previous_path_size; i++) {
		double N = (target_dist/(0.02*ref_v/2.24));
	  	double target_v = target_x/N;
		double x_point = total_x + prev_v;
		total_x = x_point;
		// accelarate until prev_v is withing threshold of target_v, then maintain target_v
		if (abs(prev_v - target_v) > 0.01) {
			x_point += accel;
			prev_v += accel;
		} else {
			prev_v = target_v;
		}
		double y_point = s(x_point);

		double x_ref = x_point;
		double y_ref = y_point;

		x_point = (x_ref *cos(yaw)-y_ref*sin(yaw));
		y_point = (x_ref *sin(yaw)+y_ref*cos(yaw));

		x_point += pos_x;
		y_point += pos_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
	  }

	  msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
