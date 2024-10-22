// A simple program to sum numbers and report it to the console.

#include <format>
#include <iostream>
#include <stdexcept>

#include "simple.h"

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
