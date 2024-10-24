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

  timespec point_of_interest1 {1532334879, 40000000};
  timespec point_of_interest2 {1532335268, 0};
  timespec point_of_interest3 {1532334000, 0};

  get_ecef_vel_at_poi(lla_data, point_of_interest1, true);
  get_ecef_vel_at_poi(lla_data, point_of_interest2, true);
  get_ecef_vel_at_poi(lla_data, point_of_interest3, true);

// Exact match found for timestamp: 1532334879 s, 40000000 ns
//         ECEF velocity at timestamp is:
//                 v_x [m/s]: -2810.935244203265938
//                 v_y [m/s]: -147.397902255505329
//                 v_z [m/s]: -3055.698853656277151

// No exact match found for timestamp: 1532335268 s, 0 ns. Interpolating from nearby points.
//         ECEF velocity at timestamp is:
//                 v_x [m/s]: -3471.021283076547661
//                 v_y [m/s]: 1760.257887866156580
//                 v_z [m/s]: -4867.476062697967791

// No exact match found for timestamp: 1532334000 s, 0 ns. Interpolating from nearby points.
//         ECEF velocity at timestamp is:
//                 v_x [m/s]: -995.915268748095855
//                 v_y [m/s]: -2514.438893975414885
//                 v_z [m/s]: 55.921220051752961

  return 0;
}
