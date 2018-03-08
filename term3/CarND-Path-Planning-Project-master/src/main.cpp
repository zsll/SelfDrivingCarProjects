#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "consts.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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

// Find lane given d value of car: each lane is four meters, so left lane is 0 - 4 meters, and middle lane is 4 - 8 meter, right lane is 8 to 12 meters
// Todo: car is not a point so it's possible to occupy two lanes. Vehicle dimension should be considered thus multiple lanes might be returned instead of a single lane
static int findOccupiedLane(float d) {
  if ( d >= 0 && d <= 4 ) {
    return 0;
  } else if ( d > 4 && d <= 8 ) {
    return 1;
  } else if ( d > 8 && d <= 12 ) {
    return 2;
  }
  return INT_MAX;
}

static double calculateCarSpeed(double vx, double vy) {
  return sqrt(vx*vx + vy*vy);
}

// Todo: when changing lane, this does not account for speed up or down. The prediction is pretty naive since it assume in the time frame of the trajectory, the other cars maintain constant speed. Not to mention cars jumping between lanes.
static bool mayCollide(double self_driving_car_s, double other_car_s, double self_driving_car_speed, double other_car_speed, int remaining_points_num_from_pre, bool same_lane) {
  bool self_driving_car_ahead = self_driving_car_s >= other_car_s;
  if (same_lane) {
    if(self_driving_car_ahead) { // No overlap will happen
      return false;
    } else {
      for(int i = 0; i <= remaining_points_num_from_pre + 1; i++) {
        if(other_car_s - self_driving_car_s <= FRONT_SAFE_DISTANCE) { // Get closing to car ahead
          return true;
        }
        self_driving_car_s += self_driving_car_speed*0.02;
        other_car_s += other_car_speed*0.02;
      }
    }
  } else {
    // For cars on different lanes, whether they will hit given current pos and speed in the next time window
    for(int i = 0; i <= remaining_points_num_from_pre + 1; i++) {
      if( (self_driving_car_ahead && (self_driving_car_s - other_car_s) <= BACK_SAFE_DISTANCE) // The following car after caught up
         || (!self_driving_car_ahead && (other_car_s - self_driving_car_s) <= SIDE_SAFE_DISTANCE)) { // Get closing to car ahead
        return true;
      }
      self_driving_car_s += self_driving_car_speed*0.02;
      other_car_s += other_car_speed*0.02;
    }
  }
  return false;
}

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
  
  int lane = 1; // 0 is left lane. 1 is middle lane. 2 is right lane.
  double reference_velocity = 0.0; // Unit is mph. Will increase/decrease this var to accelerate/decelerate
  
  h.onMessage([&lane, & reference_velocity, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          
            /* Code Start */
            int remaining_points_num_from_pre = previous_path_x.size(); // Path points number left from previous cycle
          
            /* Part 1 Detect Surrounding Cars */
            // Decect cars around given current car state and other vehicle data out of sensor fusion results
            bool may_collide_with_car_ahead = false;
            bool may_collide_with_car_on_left = false;
            bool may_collide_with_car_on_right = false;
            for ( int i = 0; i < sensor_fusion.size(); i++ ) {
              int occupiedLane = findOccupiedLane(sensor_fusion[i][6]);
              if (occupiedLane <= 2) {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double other_car_s = sensor_fusion[i][5];
                bool may_colide = mayCollide(car_s, other_car_s, car_speed, calculateCarSpeed(vx, vy), remaining_points_num_from_pre, lane == occupiedLane);  // Whethe
                may_collide_with_car_ahead |= occupiedLane == lane && may_colide;
                may_collide_with_car_on_left |= occupiedLane == lane - 1 && may_colide;
                may_collide_with_car_on_right |= occupiedLane == lane + 1 && may_colide;
              }
            }
          
            /* Part 2 Make Decision about Acceleration and Switching Lane */
            // Make decisions (lane and acc) based on cars around self
            // Note: Hybrid A* makes sense more in dense discrete env like parking lot. On highway (a sparse continuous env) there are a ton of different maneuvers so just choose one with lowest cost
            // The code below is basically a transformed finite state machine mainly focus on feasibility and velocity
            double target_acc = 0;  // This is delta speed after 0.02s time span. Three possible values: MAX_DECELERATION (negative to slow down), 0, MAX_ACCELERATION (positive to speed up)
            if (may_collide_with_car_ahead) {
              if (!may_collide_with_car_on_left && lane > 0) {
                lane = lane - 1; // // Switch to left lane (Firstly consider left lane since we usually use left lane to overpass). Maintain speed since we assume speed is same when calculating surrounding cars (Todo: speed up when switching lane?)
              } else if (!may_collide_with_car_on_right && lane < 2){
                lane = lane + 1; // Switch to right lane. Maintain speed
              } else {
                target_acc = MAX_DECELERATION;  // Have to slow down to avoid collision
              }
            } else if (reference_velocity < MAX_SPEED) {
              target_acc = MAX_ACCELERATION;  //speed up to catch up with ref velocity
            }
          
            /* Part 3 Trajectory Generation */
            // The trajectory generation method mimics that from Project Walkthrough Q&A in aaron.cpp
            // Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
            // to be interpolated with a spline and fill it in with more points
            vector<double> ptsx;
            vector<double> ptsy;
          
            // Keep track of reference state
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);  // Need this to make sure generated track is tangent to the car
          
            // Keep track of reference state: Either going to be where the car is at or previous path's endpoint
            if ( remaining_points_num_from_pre < 2 ) {
              // Use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
              
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
              
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            } else {
              // Start ref at or previous path's endpoint
              ref_x = previous_path_x[remaining_points_num_from_pre - 1];
              ref_y = previous_path_y[remaining_points_num_from_pre - 1];
              
              double ref_x_prev = previous_path_x[remaining_points_num_from_pre - 2];
              double ref_y_prev = previous_path_y[remaining_points_num_from_pre - 2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
              // Use two points that make the path tangent to the previous path's end point
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
              
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
            }
          
            double current_traj_start = remaining_points_num_from_pre > 0 ? end_path_s : car_s;
          
            // In Frenet add evenly 30m spaced points ahead of current_traj_start
            vector<double> next_wp0 = getXY(current_traj_start + 30, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(current_traj_start + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(current_traj_start + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);
          
            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);
          
            for (int i = 0; i < ptsx.size(); i++) {
              // shift car reference angle to 0 degree (car local coordinate system)
              // and current_traj_start is at (0, 0)
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
              
              ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }
          
            // create the spline.
            tk::spline s;
            // set xy points to the spline
            s.set_points(ptsx, ptsy);
          
            // start with all of the previous path points from last time
            for ( int i = 0; i < remaining_points_num_from_pre; i++ ) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
          
            // Calculate how to break up spline points so that we travel at our desired reference velocity
            double target_x = 30.0; // After rotation, we can safely assume the angle is zero and target is 30m away
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
            double x_add_on = 0;
          
            // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
            for( int i = 1; i < 50 - remaining_points_num_from_pre; i++ ) {
              reference_velocity = std::min(MAX_SPEED, std::max(0.0, reference_velocity + target_acc));
              double N = target_dist/(0.02*reference_velocity/2.24);  // Given current velocity, how many point are required to reach 30m away
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);
              
              x_add_on = x_point;
              
              double x_ref = x_point;
              double y_ref = y_point;
              // rotate back to global
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
              
              x_point += ref_x;
              y_point += ref_y;
              
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
            /* Code End */
          
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
