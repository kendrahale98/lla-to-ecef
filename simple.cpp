// A simple program to sum numbers and report it to the console.

#include <cmath>
#include <format>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include <math.h>

#include "simple.h"
#include "wgs84.h"

#define DEG_TO_RAD M_PI / 180
#define KM_TO_M 1000

/**
 * Interpolate
 * @brief Performs linear interpolation for y, given two known points,
 * (x0, y0) and (x1, y1); and an x-value within the interval (x0, x1).
 *
 * Linear interpolation formula is derived by solving for the unkown y-value
 * from the equation of slopes:
 *
 *          slope = (y - y0) / (x - x0) = (y1 - y0) / (x1 - x0)
 *
 * @param p0  (x, y) of first known point
 * @param p1  (x, y) of second known point
 * @param x   x-value of point for which y is unknown
 *
 * @return new point with linearly interpolated y-value at the given x
 *
 * @throws Runtime error if slope between given points is undefined.
 */
PointCartesian2D Interpolate(PointCartesian2D p0, PointCartesian2D p1, double x) {

    if (p0.x == p1.x) {
        throw std::runtime_error(std::format(
        "Slope is undefined between ({}, {}) and ({}, {}). "
        "A y-value cannot be calculated between these points.",
        p0.x, p0.y, p1.x, p1.y));
    }

    double y = ((p0.y * (p1.x - x)) + (p1.y * (x - p0.x))) / (p1.x - p0.x);
  
    return PointCartesian2D {x, y};
}

/**
 * ConvertLLAtoECEF
 * @brief Converts a position in LLA coordinates to ECEF coordinates.
 *
 * Uses closed formulas to transform from LLA to ECEF coordinates:
 *
 *    x_ecef = (r_curve + h) * cos(lat) * cos(long)
 *    y_ecef = (r_curve + h) * cos(lat) * sin(long)
 *    z_ecef = ( ((b * b * r_curve) / (a * a)) + h) * cos(lat) * cos(long)
 *
 * where a and b are the semi-major and semi-minor axes of the reference
 * ellipsoid in meters, h is the height above the reference ellipsoid in meters,
 * and N is the radius of curvature, defined as:
 *
 *    r_curve = a / sqrt[1 - ( (e * e) * (sin(lat) * sin(lat)) )]
 *
 * For more information see Datum Transformation of GPS Positions, at:
 * https://community.ptc.com/sejnu66972/attachments/sejnu66972/PTCMathcad/57177/2/Datum%20Transformations.pdf
 *
 * @param pos_lla a PositionLLA struct containing:
 *                  - time as timespec struct, since Unix epoch
 *                  - latitude [degrees]
 *                  - longitude [degrees]
 *                  - altitude data [meters]
 * @param a       reference ellipsoid semi-major axis [meters]
 * @param b       reference ellipsoid semi-minor axis [meters]
 * @param e       reference ellipsoid eccentricity [meters]
 *
 * @return a PositionVelocityECEF struct with the following:
 *                  - same as LLA position: time as timespec struct, since Unix epoch
 *                  - x, y, and z ECEF position fields as calculated, in meters
 *                  - x, y, and z ECEF velocities are left uninitialized
 */
PositionVelocityECEF ConvertLLAtoECEF(PositionLLA pos_lla, double a, double b, double e) {
    double rad_of_curve = CalculateRadiusOfCurvature(a, e, pos_lla.latitude);

    double cos_lat = std::cos(pos_lla.latitude * DEG_TO_RAD);
    double sin_lat = std::sin(pos_lla.latitude * DEG_TO_RAD);
    double cos_long = std::cos(pos_lla.longitude * DEG_TO_RAD);
    double sin_long = std::sin(pos_lla.longitude * DEG_TO_RAD);

    double x_ecef = (rad_of_curve + pos_lla.altitude) * cos_lat * cos_long;
    double y_ecef = (rad_of_curve + pos_lla.altitude) * cos_lat * sin_long;
    double z_ecef = ((std::pow(b, 2) * rad_of_curve / std::pow(a, 2)) + pos_lla.altitude) * sin_lat;

    return PositionVelocityECEF {pos_lla.t, x_ecef, y_ecef, z_ecef};
}

/**
 * CalculateRadiusOfCurvature
 * @brief Calculates a radius of curvature for conversion between LLA and ECEF.
 *
 * Uses the formula:
 *
 *    r_curve = a / sqrt[1 - ( (e * e) * (sin(lat) * sin(lat)) )]
 *
 * For more information see Datum Transformation of GPS Positions, at:
 * https://community.ptc.com/sejnu66972/attachments/sejnu66972/PTCMathcad/57177/2/Datum%20Transformations.pdf
 *
 * @param a         semi-major axis of the reference ellipsoid [meters]
 * @param e         eccentricity of the reference ellipsoid [meters]
 * @param latitude  [degrees]
 *
 * @return the calculated radius of curvature in meters
 */
double CalculateRadiusOfCurvature(double a, double e, double latitude) {
    return (a / std::sqrt(1 - (std::pow(e, 2) * std::pow(std::sin(latitude * DEG_TO_RAD), 2))));
}

void CalculateVelocityECEF(PositionVelocityECEF previous, PositionVelocityECEF* current) {
    
    if (!ts_is_before(previous.t, current->t)) {
      throw std::runtime_error(std::format(
        "Current point must occur after previous point."));
    }
    
    double time_diff = (current->t.tv_sec - previous.t.tv_sec)
      + ((current->t.tv_nsec - previous.t.tv_nsec) / 1e-9);
    current->v_x = (current->x - previous.x) / time_diff;
    current->v_y = (current->y - previous.y) / time_diff;
    current->v_z = (current->z - previous.z) / time_diff;
}

// bool ts_is_before(const timespec& ts1, const timespec &ts2) {
//     if (ts1.tv_sec < ts2.tv_sec) {
//         return true;
//     } else if (ts1.tv_sec == ts2.tv_sec) {
//         return (ts1.tv_nsec < ts2.tv_nsec);
//     } else {
//         return false;
//     }
// }

// bool IsEqual(const timespec& ts1, const timespec& ts2) {
//     return ts1.tv_sec == ts2.tv_sec && ts1.tv_nsec == ts2.tv_nsec;
// }

// double ts_to_double(const timespec& ts) {
//   return ts.tv_sec + (double)ts.tv_nsec / 1e9;
// }

std::vector<int> SearchPoints(const std::vector<PositionLLA>& data, timespec point_of_interest) {
    std::vector<int> indices {INDEX_ERR, INDEX_ERR, INDEX_ERR}; // before, at, after

    int idx = 0;
    for (const auto &entry : data) {
        if (ts_is_equal(entry.t, point_of_interest)) {
            indices[1] = idx;
            break;
        } else if (ts_is_before(entry.t, point_of_interest)) {
            idx++;
            continue;
        } else {
            indices[0] = idx - 1;
            indices[2] = idx;
        }
    }

    return indices;
}

std::vector<double> InterpolateECEFVelocities(const PositionVelocityECEF& before, const PositionVelocityECEF& after, const timespec& point_of_interest) {
    double t_before = ts_to_double(before.t);
    PointCartesian2D v_x_t_before {t_before, before.v_x};
    PointCartesian2D v_y_t_before {t_before, before.v_y};
    PointCartesian2D v_z_t_before {t_before, before.v_z};

    double t_after = ts_to_double(after.t);
    PointCartesian2D v_x_t_after {t_after, after.v_x};
    PointCartesian2D v_y_t_after {t_after, after.v_y};
    PointCartesian2D v_z_t_after {t_after, after.v_z};

    std::vector<double> result;
    result.push_back(Interpolate(v_x_t_before, v_x_t_after, ts_to_double(point_of_interest)).y);
    result.push_back(Interpolate(v_y_t_before, v_y_t_after, ts_to_double(point_of_interest)).y);
    result.push_back(Interpolate(v_z_t_before, v_z_t_after, ts_to_double(point_of_interest)).y);

    return result;
}

std::vector<double> GetVelocityAtTime(const std::vector<PositionLLA>& lla_data, const timespec& point_of_interest, const bool& print_output) {
  
  // Search data for point of interest
  std::vector<int> search_results = SearchPoints(lla_data, point_of_interest);

  std::vector<double> velocity_result;
  
  // If no exact time match for point of interest was found, interpolate
  if (search_results[1] == INDEX_ERR) {
    if (print_output) {
        std::cout << "No exact match found for timestamp: ";
        std::cout << point_of_interest.tv_sec << " s, ";
        std::cout << point_of_interest.tv_nsec << " ns";
        std::cout << ". Interpolating from nearby points." << std::endl;
    }
    
    // Get nearby points in ECEF
    PositionVelocityECEF before_before = ConvertLLAtoECEF(lla_data[search_results[0] - 1], WGS84Params::A, WGS84Params::B, WGS84Params::E);
    PositionVelocityECEF before = ConvertLLAtoECEF(lla_data[search_results[0]], WGS84Params::A, WGS84Params::B, WGS84Params::E);
    PositionVelocityECEF after = ConvertLLAtoECEF(lla_data[search_results[2]], WGS84Params::A, WGS84Params::B, WGS84Params::E);

    // Calculate velocity at nearby points
    CalculateVelocityECEF(before_before, &before);
    CalculateVelocityECEF(before, &after);

    // Interpolate to find velocity at point of interest
    velocity_result = InterpolateECEFVelocities(before, after, point_of_interest);

  } else {
    if (print_output) {
        std::cout << "Exact match found for timestamp: ";
        std::cout << point_of_interest.tv_sec << " s, ";
        std::cout << point_of_interest.tv_nsec << " ns" << std::endl;
    }
    
    PositionVelocityECEF pv_ecef_before_poi = ConvertLLAtoECEF(lla_data[search_results[1] - 1], WGS84Params::A, WGS84Params::B, WGS84Params::E);
    PositionVelocityECEF pv_ecef_poi = ConvertLLAtoECEF(lla_data[search_results[1]], WGS84Params::A, WGS84Params::B, WGS84Params::E);
    CalculateVelocityECEF(pv_ecef_before_poi, &pv_ecef_poi);

    velocity_result = {pv_ecef_poi.v_x, pv_ecef_poi.v_y, pv_ecef_poi.v_z};
  }

  if (print_output) {
      std::cout << "\tECEF velocity at timestamp is:" << std::endl;
      std::cout << std::fixed << std::setprecision(15);
      std::cout << "\t\tv_x [m/s]: " << velocity_result[0] << std::endl;
      std::cout << "\t\tv_y [m/s]: " << velocity_result[1] << std::endl;
      std::cout << "\t\tv_z [m/s]: " << velocity_result[2] << std::endl;
      std::cout << std::endl;
  }

  return velocity_result;
}

std::vector<PositionLLA> ReadCSVFileLLA(const std::string& filename) {
    std::ifstream file(filename);
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
                        lla_entry.altitude = std::stod(value) * KM_TO_M;
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
    }

    return lla_data;
}