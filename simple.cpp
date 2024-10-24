// A simple program to sum numbers and report it to the console.

#include <cmath>
#include <format>
#include <iostream>
#include <math.h>
#include <stdexcept>

#include "simple.h"
#include "wgs84.h"

#define DEG_TO_RAD M_PI / 180

// Returns the sum of two integers.
int Sum(int a, int b) {
    int sum = a + b;
    return sum;
}

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
    double rad_of_curve = RadiusOfCurvature(a, e, pos_lla.latitude);

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
 * RadiusOfCurvature
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
double RadiusOfCurvature(double a, double e, double latitude) {
    return (a / std::sqrt(1 - (std::pow(e, 2) * std::pow(std::sin(latitude * DEG_TO_RAD), 2))));
}