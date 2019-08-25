#include <uWS/uWS.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

using Path = std::vector<double>;

class PathGenerator {
 public:
  std::tuple<Path, Path> generateXYPath(const json& j);
  Map map;
};

int main(int argc, char* argv[]) {
  uWS::Hub h;

  // Path Generator
  PathGenerator path_gen;

  // Waypoint map to read from
  std::filesystem::path exe_path(argv[0]);
  std::string map_file_ = exe_path.parent_path() / "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  std::string line;
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
    path_gen.map.xs.push_back(x);
    path_gen.map.ys.push_back(y);
    path_gen.map.ss.push_back(s);
    path_gen.map.dxs.push_back(d_x);
    path_gen.map.dys.push_back(d_y);
  }

  h.onMessage(
      [&path_gen](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
          auto s = hasData(data);

          if (s != "") {
            auto j = json::parse(s);

            std::string event = j[0].get<std::string>();

            if (event == "telemetry") {
              const auto [next_x_vals, next_y_vals] = path_gen.generateXYPath(j);

              json msgJson;
              msgJson["next_x"] = next_x_vals;
              msgJson["next_y"] = next_y_vals;

              auto msg = "42[\"control\"," + msgJson.dump() + "]";

              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }  // end "telemetry" if
          } else {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }  // end websocket if
      });  // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length) {
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

std::tuple<Path, Path> PathGenerator::generateXYPath(const json& j) {
  static double target_speed = 0.0;
  static double too_close_continue_time = 0.0;

  // j[1] is the data JSON object
  const auto& data = j[1];

  // Main car's localization Data
  const Car car(data);

  // Previous path data given to the Planner
  std::vector<double> previous_path_x = data["previous_path_x"];
  std::vector<double> previous_path_y = data["previous_path_y"];
  const auto prev_path_size = previous_path_x.size();

  // Previous path's end s and d values
  const double end_path_s = data["end_path_s"];
  const double end_path_d = data["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  const auto sensor_fusion = data["sensor_fusion"];
  std::vector<Obstacle> obstacles;
  for (const auto& j : sensor_fusion) {
    obstacles.push_back(Obstacle(j));
  }

  // Constraints
  constexpr double cycle_time = 0.02;                          // s
  constexpr double speed_margin = 2;                           // m/s
  constexpr double speed_limit = mile2mps(50 - speed_margin);  // m/s
  constexpr double max_acc = 10;                               // m/s^2
  constexpr double max_jerk = 10;                              // m/s^3

  // Lane
  constexpr double width = 4.0;
  constexpr size_t num_lanes = 3;
  constexpr double d_lane1 = 0.5 * width;
  constexpr double d_lane2 = 1.5 * width;
  constexpr double d_lane3 = 2.5 * width;

  const auto estimateLaneNumber = [](const double d) {
    if (d_lane1 - width / 2 < d && d < d_lane1 + width / 2) {
      return 1;
    } else if (d_lane2 - width / 2 < d && d < d_lane2 + width / 2) {
      return 2;
    } else if (d_lane3 - width / 2 < d && d < d_lane3 + width / 2) {
      return 3;
    }

    return -1;
  };

  const int my_lane_number = estimateLaneNumber(car.d);
  const int left_lane_number = my_lane_number != 1 ? my_lane_number - 1 : -1;
  const int right_lane_number = my_lane_number != 3 ? my_lane_number + 1 : -1;

  // Obstacle
  double dist_to_nearest_obs = 100;
  double speed_of_nearest_obs = 100;
  for (const auto& obstacle : obstacles) {
    const int obs_lane_number = estimateLaneNumber(obstacle.d);

    if (my_lane_number == obs_lane_number) {
      const auto dist = obstacle.s - car.s;
      if (0 <= dist && dist <= dist_to_nearest_obs) {
        dist_to_nearest_obs = dist;
        speed_of_nearest_obs = obstacle.v;
      }
    }
  }

  // Lane Change
  const auto is_lane_empty = [&](const auto lane_number) {
    if (lane_number == -1) {
      return false;
    }

    for (const auto& obstacle : obstacles) {
      const int obs_lane_number = estimateLaneNumber(obstacle.d);
      if (obs_lane_number != lane_number) {
        continue;
      }

      if (car.s - 30 < obstacle.s && obstacle.s < car.s + dist_to_nearest_obs) {
        return false;
      }
    }

    return true;
  };

  static int target_lane_number = 2;

  if (too_close_continue_time >= 3.0) {
    if (is_lane_empty(left_lane_number)) {
      target_lane_number = my_lane_number - 1;
    } else if (is_lane_empty(right_lane_number)) {
      target_lane_number = my_lane_number + 1;
    }
  }

  double d_target_lane;
  if (target_lane_number == 1) {
    d_target_lane = d_lane1;
  } else if (target_lane_number == 2) {
    d_target_lane = d_lane2;
  } else if (target_lane_number == 3) {
    d_target_lane = d_lane3;
  } else {
    d_target_lane = d_lane2;
  }

  // Find the last points
  double prev_x, prev_y;
  double ref_x, ref_y, ref_yaw;
  if (previous_path_x.size() <= 1) {
    prev_x = car.x - cos(car.yaw);
    prev_y = car.y - sin(car.yaw);
    ref_x = car.x;
    ref_y = car.y;
    ref_yaw = car.yaw;
  } else {
    prev_x = previous_path_x.at(prev_path_size - 2);
    prev_y = previous_path_y.at(prev_path_size - 2);
    ref_x = previous_path_x.at(prev_path_size - 1);
    ref_y = previous_path_y.at(prev_path_size - 1);
    ref_yaw = atan2(ref_y - prev_y, ref_x - prev_x);
  }

  // Calculate ref_s and ref_d
  const auto [ref_s, ref_d] = getFrenet(ref_x, ref_y, ref_yaw, map);

  // Create Spline
  std::vector<double> xs_append{prev_x - ref_x, 0};
  std::vector<double> ys_append{prev_y - ref_y, 0};
  const auto d = hypot(prev_x - ref_x, prev_y - ref_y);
  std::vector<double> ts{-d, 0};
  for (const auto t : {30, 60, 90}) {
    const auto [x, y] = getXY(ref_s + t, d_target_lane, map);

    ts.push_back(t);
    xs_append.push_back(x - ref_x);
    ys_append.push_back(y - ref_y);
  }

  tk::spline s_x;
  tk::spline s_y;
  s_x.set_points(ts, xs_append);
  s_y.set_points(ts, ys_append);

  // Acceleration
  constexpr double max_speed_change = 0.1;

  const bool too_close = dist_to_nearest_obs < 30;
  if (too_close) {
    too_close_continue_time += 0.02;

    const double speed_margin = 0.0;  // TODO: calculate by distance
    if (target_speed > speed_of_nearest_obs - speed_margin) {
      target_speed -= max_speed_change;
    }
  } else {
    too_close_continue_time = 0.0;
    target_speed += max_speed_change;
  }
  target_speed = std::min(speed_limit, target_speed);

  // Path Generation
  std::vector<double> next_x_vals = previous_path_x;
  std::vector<double> next_y_vals = previous_path_y;

  // Resampling
  const size_t num_points = 50;  // 0.02s * 50 = 1s
  for (auto i = 0; i < num_points - prev_path_size; ++i) {
    const double t = (i + 1) * target_speed * cycle_time;
    const double x = ref_x + s_x(t);
    const double y = ref_y + s_y(t);

    next_x_vals.push_back(x);
    next_y_vals.push_back(y);
  }

  return {next_x_vals, next_y_vals};
}
