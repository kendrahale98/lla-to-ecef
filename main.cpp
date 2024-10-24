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

    std::cout << "(" << ecef_data[index].t.tv_sec << "," << ecef_data[index].t.tv_nsec << ")\t";
    std::cout << std::fixed << std::setprecision(10);
    std::cout << ecef_data[index].x << "\t" << ecef_data[index].y << "\t" << ecef_data[index].z << "\t";
    std::cout << ecef_data[index].v_x << "\t" << ecef_data[index].v_y << "\t" << ecef_data[index].v_z;
    std::cout << std::endl;

    index++;
  }

  return 0;
}