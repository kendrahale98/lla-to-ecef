// A simple program to sum numbers and report it to the console.

#include <string>
#include <vector>

#include "simple.h"

// Main function, sums integers and prints outputs.
int main() {
  std::vector<PositionLLA> lla_data = ReadCSVFileLLA("./SciTec_code_problem_data.csv");

  timespec point_of_interest1 {1532334879, 40000000};
  timespec point_of_interest2 {1532335268, 0};
  timespec point_of_interest3 {1532334000, 0};

  GetVelocityAtTime(lla_data, point_of_interest1, true);
  GetVelocityAtTime(lla_data, point_of_interest2, true);
  GetVelocityAtTime(lla_data, point_of_interest3, true);

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