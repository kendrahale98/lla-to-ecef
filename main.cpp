/**
 * @file main.cpp
 *
 * @brief Program to ingest LLA position data and produce ECEF velocity points.
 *
 * @author Kendra Hale, <kendrahale98@gmail.com>
 *
 * @date 10/25/24
 */
#include <string>
#include <vector>

#include "./lla_to_ecef.h"

int main() {
  std::vector<PositionLLA> lla_data = read_csv_lla("./SciTec_code_problem_data.csv");

  timespec point_of_interest1 {1532335268, 0};
  timespec point_of_interest2 {1532334000, 0};

  std::vector<double> result;
  get_ecef_vel_at_pt(lla_data, point_of_interest1, result, true);
  get_ecef_vel_at_pt(lla_data, point_of_interest2, result, true);

  return 0;
}
