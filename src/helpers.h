#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "json.hpp"

// for convenience
using nlohmann::json;

constexpr double mile2mps(const double mile) { return mile / 3.6 * 1.60934; }

struct Car {
  Car(const json& data)
      : x(data["x"]),
        y(data["y"]),
        s(data["s"]),
        d(data["d"]),
        yaw(data["yaw"]),
        speed(data["speed"]),
        v(mile2mps(speed)) {}

  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  double v;
};

struct Map {
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> ss;
  std::vector<double> dxs;
  std::vector<double> dys;
};

struct Obstacle {
  Obstacle(const json& data)
      : id(data[0]),
        x(data[1]),
        y(data[2]),
        vx(data[3]),
        vy(data[4]),
        s(data[5]),
        d(data[6]),
        v(hypot(vx, vy)) {}

  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
  double v;
};

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
constexpr double deg2rad(double x) { return x * pi() / 180; }
constexpr double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const Map& map) {
  double closestLen = 100000;  // large number
  int closestWaypoint = 0;

  for (int i = 0; i < map.xs.size(); ++i) {
    double map_x = map.xs[i];
    double map_y = map.ys[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const Map& map) {
  int closestWaypoint = ClosestWaypoint(x, y, map);

  double map_x = map.xs[closestWaypoint];
  double map_y = map.ys[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 2) {
    ++closestWaypoint;
    if (closestWaypoint == map.xs.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::tuple<double, double> getFrenet(double x, double y, double theta, const Map& map) {
  int next_wp = NextWaypoint(x, y, theta, map);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = map.xs.size() - 1;
  }

  double n_x = map.xs[next_wp] - map.xs[prev_wp];
  double n_y = map.ys[next_wp] - map.ys[prev_wp];
  double x_x = x - map.xs[prev_wp];
  double x_y = y - map.ys[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  // see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - map.xs[prev_wp];
  double center_y = 2000 - map.ys[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(map.xs[i], map.ys[i], map.xs[i + 1], map.ys[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::tuple<double, double> getXY(double s, double d, const Map& map) {
  int prev_wp = -1;

  while (s > map.ss[prev_wp + 1] && (prev_wp < (int)(map.ss.size() - 1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp + 1) % map.xs.size();

  double heading = atan2((map.ys[wp2] - map.ys[prev_wp]), (map.xs[wp2] - map.xs[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - map.ss[prev_wp]);

  double seg_x = map.xs[prev_wp] + seg_s * cos(heading);
  double seg_y = map.ys[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

#endif  // HELPERS_H
