///////////////////////////////////////////////////
//// Inspired by Udacity Project Walk-through /////
///////////////////////////////////////////////////

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

#define LANE_WITH 4
#define DELAY 0.02
#define MPERS_TO_MPH 2.24
#define MAX_LANE 3
#define MAX_SPEED 49.5
#define MIN_SPEED 0.01
#define SPEED_STEP 0.25
#define CAR_LENGTH 5.0
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
std::pair<double, double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
std::pair<double, double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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

struct state {

  int id = -2;
  double x = 0;
  double y = 0;
  double yaw = 0;
  double s = 0;
  double d = 0;
  double speed = 0;
  double lane = -1;
  double target_lane = 1;
  std::shared_ptr<state> future_state = nullptr;

  ~state(){future_state.reset();}
  state(){}
  void copy(state other){
    id = other.id;
    x = other.x;
    y = other.y;
    yaw = other.yaw;
    s = other.s;
    d = other.d;
    speed = other.speed;
    lane = other.lane;
    target_lane = other.target_lane;
    future_state = other.future_state; 
  }

};

class PathPlanner{

  private:

    state car;
    state refstate;
    vector<state> nearby_cars;
    double step = 30.0;
    double target_speed = MIN_SPEED;
    int n_waypoints = 50;
    int n_future_states = 0;
    // Previous path data given to the Planner
    vector<double> previous_path_x;
    vector<double> previous_path_y;
    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    json sensor_fusion;

    state current_state;
    vector<state> spline_points;
    vector<state> next_state_vals;

    tk::spline * spline = nullptr;

    double lane_to_d(int lane_idx) {
      double d_val = LANE_WITH/2 +LANE_WITH*lane_idx;
      return d_val;
    }

    state localToGlobal(state input){
      state output = input;
      output.x = input.x*cos(refstate.yaw) - input.y*sin(refstate.yaw);
      output.y = input.x*sin(refstate.yaw) + input.y*cos(refstate.yaw);
      output.x += refstate.x;
      output.y += refstate.y;
      return output;
    }

    state globalToLocal(state input){
      state output = input;
      input.x -= refstate.x;
      input.y -= refstate.y;
      output.x = input.x*cos(-refstate.yaw) - input.y*sin(-refstate.yaw);
      output.y = input.x*sin(-refstate.yaw) + input.y*cos(-refstate.yaw);
      return output;
    }

    void sync_frenet(state &car){ std::tie(car.s, car.d) = getFrenet(car.x, car.y, car.yaw, map_waypoints_x, map_waypoints_y);}
    void sync_cartesian(state &car){std::tie(car.x, car.y) = getXY(car.s, car.d, map_waypoints_s, map_waypoints_x, map_waypoints_y);}
    void get_xy_vals(){
      x_vals.clear();
      y_vals.clear();
      for (int i = 0; i < next_state_vals.size(); ++i){
        x_vals.push_back(next_state_vals[i].x);
        y_vals.push_back(next_state_vals[i].y);
      }
    }

    void read_input(const json& input){
        // Main car's localization Data
        car.id = -1;
        car.x = input["x"];
        car.y = input["y"];
        car.s = input["s"];
        car.d = input["d"];
        car.yaw = deg2rad(input["yaw"]);
        car.speed = input["speed"];
        calculate_lane(car);
        // Previous path data given to the Planner
        auto _previous_path_x = input["previous_path_x"];
        auto _previous_path_y = input["previous_path_y"];
        previous_path_x.clear();
        previous_path_y.clear();
        for (int i = 0; i < _previous_path_x.size(); ++i){
          previous_path_x.push_back(_previous_path_x[i]);
          previous_path_y.push_back(_previous_path_y[i]);
        }
        n_future_states = previous_path_x.size();
        // Previous path's end s and d values
        car.future_state.reset(new state);
        state & car_future = *car.future_state;
        car_future.copy(car);
        if(n_future_states>0){
          car_future.s = input["end_path_s"];
          car_future.d = input["end_path_d"];
          sync_cartesian(car_future);
        }
        // Sensor Fusion Data, a list of all other cars on the same side of the road.
        sensor_fusion = input["sensor_fusion"];
    }

    void generate_spline_points(){
      spline_points.clear();
      // First spline points
      if(previous_path_x.size()<2){
        state prevstate;
        prevstate.x = car.x - cos(car.yaw);
        prevstate.y = car.y - sin(car.yaw); 
        prevstate.yaw = car.yaw;
        sync_frenet(prevstate);
        sync_frenet(car);
        spline_points.push_back(prevstate);
        spline_points.push_back(car);
      } else{
        int n_spline_prev_points = 2;
        for (int i = previous_path_x.size()-n_spline_prev_points; i < previous_path_x.size(); ++i){
          state prevstate;
          prevstate.x = previous_path_x[i];
          prevstate.y = previous_path_y[i];
          //std::cout << "previous_path_x: " << prevstate.x << " previous_path_y: " << prevstate.y <<::endl;
          prevstate.yaw = atan2(previous_path_y[i] - previous_path_y[i-1],previous_path_x[i] - previous_path_x[i-1]);
          sync_frenet(prevstate);
          spline_points.push_back(prevstate);
        }
      }

      // More points
      int n_spline_new_points = 3;
      for (int i = 0; i < n_spline_new_points; ++i){ 
        state newstate;
        double check_x, check_y;
        newstate.s = spline_points[1].s+(i+1)*step;
        newstate.d = lane_to_d(car.target_lane);
        sync_cartesian(newstate);
        spline_points.push_back(newstate);
      }

      // Rotate 
      refstate = spline_points.front();
      for (int i = 0; i < spline_points.size(); ++i){
        spline_points[i] = globalToLocal(spline_points[i]);
      }
    }

    void create_spline(){
      std::sort(spline_points.begin(), spline_points.end(),[](state a, state b) { return a.x < b.x;});
      vector<double> ptx, pty;
      for (int i = 0; i<spline_points.size(); ++i){
        //cout << "local pts x : " << spline_points[i].x << " y : " << spline_points[i].y<< endl;
        ptx.push_back(spline_points[i].x);
        pty.push_back(spline_points[i].y);
      }
      if(spline){delete spline;}
      spline = new tk::spline();
      spline->set_points(ptx, pty);
    }

    void fill_waypoints(){
      next_state_vals.clear();
      for (int i = 0; i < previous_path_x.size(); ++i){
        state waypoint;
        waypoint.x =  previous_path_x[i];
        waypoint.y =  previous_path_y[i];
        if (i==0){waypoint.yaw =  car.yaw;}
        else {waypoint.yaw = atan2(previous_path_y[i] - previous_path_y[i-1],previous_path_x[i] - previous_path_x[i-1]);}
        sync_frenet(waypoint);
        //std::cout << "0 waypoint.x " << waypoint.x << " waypoint.x " << waypoint.x <<::endl;
        //std::cout << "0 waypoint.s " << waypoint.s << " waypoint.d " << waypoint.d <<::endl;
        next_state_vals.push_back(waypoint);
      }

      double target_dist = sqrt(step*step + (*spline)(step)*(*spline)(step));
      double speed = std::max(float(target_speed), float(MIN_SPEED));
      double N = abs(target_dist/(DELAY*speed/MPERS_TO_MPH));
      //std::cout << "step " << step << " (*spline)(step) " <<(*spline)(step) <<  "speed  " <<  speed <<  " target_dist  " <<  target_dist <<  " N  " <<  N <<  " step/N  " <<  step/N <<  std::endl;
      state prev_waypoint;
      if (previous_path_x.size()>0){
        prev_waypoint.x = next_state_vals.back().x;
        prev_waypoint.y = next_state_vals.back().y;
        prev_waypoint.yaw = next_state_vals.back().yaw;
        sync_frenet(prev_waypoint);
        prev_waypoint = globalToLocal(prev_waypoint);
      }
      //cout << "0 n_waypoints : " << n_waypoints << " previous_path_x.size( " << previous_path_x.size()<< endl;
      for (int i = 0; i < n_waypoints - previous_path_x.size(); ++i){
        state waypoint;
        waypoint.x = prev_waypoint.x + step/N;
        waypoint.y = (*spline)(waypoint.x);
        //std::cout << "prev_waypoint.x " << prev_waypoint.x << " waypoint.x " << waypoint.x <<  "step  " <<  step <<  "N  " <<  N <<  "step/N  " <<  step/N <<  std::endl;
        if (i==0){waypoint.yaw =  car.yaw;}
        else {waypoint.yaw = atan2(next_state_vals.back().y - waypoint.y,next_state_vals.back().x - waypoint.x);}
        sync_frenet(waypoint);
        prev_waypoint.copy(waypoint);
        waypoint = localToGlobal(waypoint);
        //std::cout << "1 waypoint.x " << waypoint.x << " waypoint.y " << waypoint.y <<::endl;
        //std::cout << "1 waypoint.s " << waypoint.s << " waypoint.d " << waypoint.d <<::endl;
        next_state_vals.push_back(waypoint);
      }
      assert(next_state_vals.size()==n_waypoints);
    }

    void calculate_lane(state & car){
      for (int lane = 0; lane <= MAX_LANE; ++lane){
        double left_side = lane_to_d(lane) - LANE_WITH/2;
        double right_side = lane_to_d(lane) + LANE_WITH/2;
        //cout << "lane: "<<lane<< "lane_to_d: "<<lane_to_d(lane)<<"" "left_side: " << left_side << " right_side: " << right_side << " d: "<<car.d <<endl;
        if ( left_side <= car.d and car.d <= right_side){ car.lane = lane; break;}
      }
      if(!(0<=car.lane)){car.lane = -1;};
    }

    state load_nearby_car(json sensor_fusion, int i){
        state nearby_car;
        nearby_car.id = sensor_fusion[i][0];
        nearby_car.x = sensor_fusion[i][1];
        nearby_car.y = sensor_fusion[i][2];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        nearby_car.speed = sqrt(vx*vx + vy*vy);
        nearby_car.s = sensor_fusion[i][5];
        nearby_car.d = sensor_fusion[i][6];
        calculate_lane(nearby_car);
        nearby_car.future_state.reset(new state);
        nearby_car.future_state->copy(nearby_car);
        state &nearby_car_future = *nearby_car.future_state;
        nearby_car_future.s = nearby_car_future.s + DELAY*nearby_car.speed*n_future_states;
        sync_cartesian(nearby_car_future);
        calculate_lane(nearby_car_future);
        return nearby_car;

    }

    bool validate_target(int new_target_lane){
      for (const auto & nearby_car:nearby_cars){
        if (will_be_too_close(nearby_car, car, new_target_lane)){ return false;}
      }
      return true;
    }

    void change_lanes(){
      double new_target_lane = car.target_lane;
      if (car.future_state->lane>0 && car.lane >0 && car.target_lane>0){new_target_lane = car.target_lane-1;}
      if (car.future_state->lane==0 && car.lane==0 && car.target_lane==0 && car.future_state->lane<MAX_LANE && car.lane<MAX_LANE && car.target_lane<MAX_LANE){new_target_lane = car.target_lane+1;}
      if(validate_target(new_target_lane)){car.target_lane=new_target_lane;}
    }

    bool is_too_close(const state &nearby_car, const state &current_car){
      bool too_close = false;
      // Past
      if (nearby_car.lane == current_car.lane or nearby_car.future_state->lane == current_car.future_state->lane){
        // Present
        if ((nearby_car.s - current_car.s < CAR_LENGTH*2) && (0<nearby_car.s - current_car.s)) { too_close = true;}
        // Future
        if ((nearby_car.future_state->s - current_car.future_state->s < step) && (0<nearby_car.future_state->s - current_car.future_state->s)) { too_close = true;}
        // cout << "\ncurrent_car.future_state->lane " << current_car.future_state->lane << endl;
        // cout << "current_car.lane " << current_car.lane << endl;
        // cout << "nearby_car.future_state->lane " << nearby_car.future_state->lane << endl;
        // cout << "nearby_car.lane " << nearby_car.lane << endl;
        // cout << "nearby_car.s - current_car.s " << nearby_car.s - current_car.s << endl;
        // cout << "(nearby_car.s - current_car.s < CAR_LENGTH*2)" << (nearby_car.s - current_car.s < CAR_LENGTH*2) << endl;
        // cout << "(0<nearby_car.s - current_car.s)" << (0<nearby_car.s - current_car.s) << endl;
        // cout << "too_close " << too_close << endl;
      } 
      return too_close;
    }

    bool will_be_too_close(const state &nearby_car, const state &current_car, int new_lane){
      state future_car = current_car;
      future_car.lane = new_lane;
      future_car.future_state->lane = new_lane;
      future_car.d = lane_to_d(new_lane);
      future_car.future_state->d = lane_to_d(new_lane);
      sync_cartesian(future_car);
      sync_cartesian((*future_car.future_state));
      return is_too_close(nearby_car, future_car);
    }

    void check_nearby(){
      bool too_close = false;

      // Load Data
      nearby_cars.clear();
      for (int i = 0; i < sensor_fusion.size(); ++i){
        state nearby_car = load_nearby_car(sensor_fusion, i);
        nearby_cars.push_back(nearby_car); 
      }

      // Check
      for (const auto & nearby_car:nearby_cars){
        if(nearby_car.lane == -1){continue;}
        if(is_too_close(nearby_car, car)){too_close=true;}  
      }

      // Action
      if (too_close){
        // cout<<"TOO CLOSE"<<endl;
        change_lanes();
        target_speed -= SPEED_STEP;
      }else if (target_speed<MAX_SPEED){
        //cout<<"NOT TOO CLOSE"<<endl;
        target_speed += SPEED_STEP;
      }
    }

  public:
    PathPlanner(){}
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
    vector<double> x_vals, y_vals;

    void plan(const json& input, json &output){
      read_input(input);
      check_nearby();
      generate_spline_points();
      create_spline();
      fill_waypoints();

      // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
      get_xy_vals();
      output["next_x"] = x_vals;
      output["next_y"] = y_vals;
      // output["next_x"];
      // output["next_y"];

      //cout << "FFF" << endl;
    }
};

int main() {

  PathPlanner planner;

  uWS::Hub h;
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
    planner.map_waypoints_x.push_back(x);
    planner.map_waypoints_y.push_back(y);
    planner.map_waypoints_s.push_back(s);
    planner.map_waypoints_dx.push_back(d_x);
    planner.map_waypoints_dy.push_back(d_y);
  }




  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    ////cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

            json msgJson;
            //cout << "DEBUG o"<< endl;
            planner.plan(j[1], msgJson);
            //cout << "DEBUG i"<< endl;

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
