// A simple program to sum numbers and report it to the console.

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "simple.h"
#include "wgs84.h"

// Main function, sums integers and prints outputs.
int main() {
  std::ifstream file("./SciTec_code_problem_data.csv");
  std::vector<PositionLLA> lla_data;

  if (file.is_open()) {
    std::string line;
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string value;
      PositionLLA lla_entry;

      int index = 0;
      while(std::getline(ss, value, ',')) {
        switch (index) {
          case 0: {
            std::string span = value;
            size_t pos = span.find('.');
            lla_entry.t.tv_sec = std::stol(span.substr(0, pos));
            lla_entry.t.tv_nsec = int (std::stod(span.substr(pos)) * 1e9);
            break;
          }
          case 1:
            lla_entry.latitude = std::stod(value);
            break;
          case 2:
            lla_entry.longitude = std::stod(value);
            break;
          case 3:
            lla_entry.altitude = std::stod(value);
          default:
            break;
        }
        index++;
      }

      lla_data.push_back(lla_entry);
    }

    file.close();
  } else {
    std::cerr << "Error opening file." << std::endl;
    return 1;
  }

  int index = 0;
  std::vector<PositionVelocityECEF> ecef_data;
  for (const auto &entry : lla_data) {
    // std::cout << "(" << entry.t.tv_sec << "," << entry.t.tv_nsec << ")\t";
    // std::cout << std::fixed << std::setprecision(30);
    // std::cout << entry.latitude << "\t" << entry.longitude << "\t" << entry.altitude;
    // std::cout << std::endl;

    ecef_data.push_back(ConvertLLAtoECEF(entry, WGS84Params::A, WGS84Params::B, WGS84Params::E));

    // Don't calculate velocity for first data point
    if (&entry == &lla_data.front()) {
      index++;
      continue;
    }

    CalculateVelocityECEF(ecef_data[index - 1], &ecef_data[index]);

    // std::cout << "(" << ecef_data[index].t.tv_sec << "," << ecef_data[index].t.tv_nsec << ")\t";
    // std::cout << std::fixed << std::setprecision(10);
    // std::cout << ecef_data[index].x << "\t" << ecef_data[index].y << "\t" << ecef_data[index].z << "\t";
    // std::cout << ecef_data[index].v_x << "\t" << ecef_data[index].v_y << "\t" << ecef_data[index].v_z;
    // std::cout << std::endl;

    index++;
  }

  timespec poi_ts {1532335268, 0};
  // timespec poi_ts {1532334000, 0};

  int idx_before_poi;
  int idx_after_poi;

  int idx = 0;
  for (const auto &entry : lla_data) {
    if (IsBefore(entry.t, poi_ts)) {
      idx++;
      continue;
    } else {
      idx_after_poi = idx;
      idx_before_poi = idx - 1;
    }
  }

  PositionVelocityECEF before_before = ConvertLLAtoECEF(lla_data[idx_before_poi - 1], WGS84Params::A, WGS84Params::B, WGS84Params::E);
  PositionVelocityECEF before = ConvertLLAtoECEF(lla_data[idx_before_poi], WGS84Params::A, WGS84Params::B, WGS84Params::E);
  PositionVelocityECEF after = ConvertLLAtoECEF(lla_data[idx_after_poi], WGS84Params::A, WGS84Params::B, WGS84Params::E);

  CalculateVelocityECEF(before_before, &before);
  CalculateVelocityECEF(before, &after);

  double t_before = timespec_to_double(before.t);
  PointCartesian2D v_x_t_before {before.v_x, t_before};
  PointCartesian2D v_y_t_before {before.v_y, t_before};
  PointCartesian2D v_z_t_before {before.v_z, t_before};

  double t_after = timespec_to_double(before.t);
  PointCartesian2D v_x_t_after {after.v_x, t_after};
  PointCartesian2D v_y_t_after {after.v_y, t_after};
  PointCartesian2D v_z_t_after {after.v_z, t_after};

  double result_v_x = Interpolate(v_x_t_before, v_x_t_after, timespec_to_double(poi_ts)).y;
  double result_v_y = Interpolate(v_y_t_before, v_y_t_after, timespec_to_double(poi_ts)).y;
  double result_v_z = Interpolate(v_z_t_before, v_z_t_after, timespec_to_double(poi_ts)).y;

  std::cout << std::fixed << std::setprecision(10);
  std::cout << result_v_x << "\t" << result_v_y << "\t" << result_v_z;
  std::cout << std::endl;


  // 1532333672.6644752026   1532334037.8173997402   1532333564.3896265030
  // 1532335244.9414927959   1532335296.9903407097   1532335291.3677251339

  return 0;
}