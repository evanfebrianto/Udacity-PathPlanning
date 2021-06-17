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
// #include "fsm.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// Define constants
const double MAX_VEL = 49.5;
const double MAX_ACC = .4026485; // .18m/s per .02 sec -> because the max acc is 10m/s2
const double MAX_DEC = 1.8*MAX_ACC;

const double PROJECTION_IN_METERS = 30.0;

const int LEFT_LANE = 0;
const int MIDDLE_LANE = 1;
const int RIGHT_LANE = 2;
const int INVALID_LANE = -1;

const int LEFT_LANE_MAX = 4;
const int MIDDLE_LANE_MAX = 8;
const int RIGHT_LANE_MAX = 12;

// enum States { Ready, KeepLane, FollowFront, PrepLaneSwitch };
// enum Triggers {  CarAhead, Clear, SafeToChange };
// FSM::Fsm<States, States::Ready, Triggers> fsm;
// const char * StateNames[] = { "Ready", "Keep in Lane", "Follow Vehicle in Front", 
//                               "Prepare to Lane Switch" };

// void dbg_fsm(States from_state, States to_state, Triggers trigger) {
//     if (from_state != to_state) {
//         std::cout << "State Changed To: " << StateNames[to_state] << "\n";
//     }
// }


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

  // Reference lane 1
  int lane = 1;

  // Reference velocity
  double ref_vel = 0;

  // State
  int state = 0;
  
  // Waiting counter to prevent multiple concurrent lane switches
  int waiting = 100;
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,&state,&waiting]
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
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

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

          int prev_size = previous_path_x.size();
          
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }
          
          // Initialize left/right velocities to the maximum reference, 
          // which will be lowered if a car is detected in the nearby lane
          double left_vel = MAX_VEL;
          double right_vel = MAX_VEL;
          
          // Initialize the gap as true until proven false
          bool gap_left = true;
          bool gap_right = true;
          
          // Set up variables to detect and slow down the car
          bool in_front = false;
          double front_vel = MAX_VEL;

          // Initialize detected car in left and right
          bool isLeftCar = false;
          bool isRightCar = false;

          // Iterate through and use sensor fusion data
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            // Other's car is in my lane
            float d = sensor_fusion[i][6];
            
            // Calculate the speed of the detected car
            float vx = sensor_fusion[i][3];
            float vy = sensor_fusion[i][4];
            float check_speed = sqrt(pow(vx, 2) + pow(vy, 2)); 
            float check_car_s = sensor_fusion[i][5];
            
            // If using previous points can project a value out
            check_car_s += (double)prev_size * .02 * check_speed;
            
            // Match units of m/s to be the same as ref_vel
            check_speed *= 2.24;

            int car_lane;
            
            if (d > 0 && d < LEFT_LANE_MAX)
              car_lane = LEFT_LANE;
            else if (d > LEFT_LANE_MAX && d < MIDDLE_LANE_MAX)
              car_lane = MIDDLE_LANE;
            else if (d > MIDDLE_LANE_MAX && d < RIGHT_LANE_MAX)
              car_lane = RIGHT_LANE;
            else {
              car_lane = INVALID_LANE;
              continue;
            }

            // Check ahead of current car's location
            if (car_lane == lane)
            {
              // Check s values greater than mine and s gap
              if (check_car_s - car_s > 0 && check_car_s - car_s < PROJECTION_IN_METERS)
              {
                // If a car is detected, change to state 1 to match its speed (if not over the speed limit)
                state = 1;
                in_front = true;
                front_vel = check_speed;
                
                // Prepare to switch lanes if the car in front is slow, car isn't already preparing, and the timer is up between lane switches       
                if (check_speed < 0.8*MAX_VEL && state != 2 && waiting == 0)
					        state = 2;
              }
            }
            // Check left lane of car
            else if (car_lane + 1 == lane)
            {
              isLeftCar = true;

              // Checks values greater than mine and s gap
              if (abs(car_s + PROJECTION_IN_METERS-5) > check_car_s && abs(car_s - PROJECTION_IN_METERS-15) < check_car_s)
                gap_left = false;
              
              // Obtain left lane speed
              if (abs(car_s + PROJECTION_IN_METERS-5) < check_car_s && abs(car_s + PROJECTION_IN_METERS+15) > check_car_s)
                left_vel = check_speed;
            }
            // Check right lane of car
            else if (car_lane - 1 == lane)
            {
              isRightCar = true;
              
              // Checks values greater than mine and s gap
              if (abs(car_s + PROJECTION_IN_METERS-5) > check_car_s && abs(car_s - PROJECTION_IN_METERS-15) < check_car_s)
                gap_right = false;
              
              // Obtain right lane speed
              if (abs(car_s + PROJECTION_IN_METERS-5) < check_car_s && abs(car_s + PROJECTION_IN_METERS+15) > check_car_s)
                  right_vel = check_speed;
            }
          }
          
          // Variables to determine if lane changing is safe
          bool left_turn = lane > LEFT_LANE && left_vel >= front_vel && gap_left;
          bool right_turn = lane < RIGHT_LANE && right_vel >= front_vel && gap_right;

          // Decrement counter for 1 second total (0.02s x 50)
          if (waiting > 0)
            waiting--;
          
          // Finite State Machine:
          // State 0 - Accelerating up to top reference speed with no car in front
          // State 1 - Matching the speed of the car in front (minus 1.5m/s) to keep distance
          // State 2 - Preparing + Lane Switching
          if (state == 0) // Accelerate in empty road
          {
            if (ref_vel < MAX_VEL)
              ref_vel += MAX_ACC;
          }
          else if (state == 1) // Match speed of car in front
          {
            // Extra -1 to prevent tailgating
            if (front_vel - 1.5 < ref_vel)
              ref_vel -= MAX_DEC;
            else
            {
              if (ref_vel < MAX_VEL)
                ref_vel += MAX_ACC;
            }
            if (gap_left || gap_right) {
              state = 2;
            }
          }
          else if (state == 2) // Preparing to switch lanes
          {
            if (in_front)
            {
              // Extra -1 to prevent tailgating
              if (front_vel - 1.5 < ref_vel)
                ref_vel -= MAX_DEC;
              else
              {
                if (ref_vel < MAX_VEL)
                  ref_vel += MAX_ACC;
              }
            }
            else
            {
              if (ref_vel < MAX_VEL)
                ref_vel += MAX_ACC;
              else
                ref_vel -= MAX_DEC;
            }

            // Choose faster lane to switch to
            if (left_turn && right_turn)
            {
              if (left_vel > right_vel)
                lane--;
              else
                lane++;
            }
            else if (right_turn)
              lane++;
            else if (left_turn)
              lane--;

            // Reset state and waiting counter after lane switch
            if (left_turn || right_turn)
            {
              state = 0;
              waiting = 100;
            }
          }

          // Debug
          std::string _info;
          switch (state) {
            case 0:
              _info = "No car in front, accelerating to top speed ...";
              break;
            case 1:
              _info = "There is a car in front, following the vehicle.";
              break;
            case 2:
              _info = "Preparing to switch lane.";
              break;
          }
          std::cout << "vel_l_f_r: " << left_vel << "  " << front_vel << "  " << right_vel << "\n";
          std::cout << "isCar_l_r: " << isLeftCar << "  " << isRightCar << "\n";
          std::cout << "gap_l_r: " << gap_left << "  " << gap_right << "\n";
          std::cout << "turn_l_r: " << left_turn << "  " << right_turn << "\n";
          std::cout << "State: " << _info << "\n\n";
          
          // Vectors to store to-be-generated path
          vector<double> ptsx;
          vector<double> ptsy;
          
          // Store variables and convert to radians
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // If previous data is limited, use current position as well
          if (prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // Use previous data to ensure path is smooth
          else
          {
            // Redefine reference state as previous path and point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            // Use two points that make hte point tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          // In Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + PROJECTION_IN_METERS, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + PROJECTION_IN_METERS*2, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + PROJECTION_IN_METERS*3, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          // Store path coordinates
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
        
          // Transform to s, d coordinates
          for (int i = 0; i < ptsx.size(); i++)
          {
            // Shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }
          
          // Create a spline
          tk::spline s;
          
          // Set (x,y) points to the spline
          s.set_points(ptsx, ptsy);
          
          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // Start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
          
          double x_add_on = 0;
          
          // Fill up the rest of our path planner after filling in with previous points using 50 points
          for (int i = 0; i < 50 - previous_path_x.size(); i++)
          {
            double N = (target_dist/(0.02 * ref_vel/2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Rotate back to normal after rotating it earlier
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            // Store final x y coordinates to give to the simulator
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