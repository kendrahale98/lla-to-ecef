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
 * @param x0  x-value of first known point
 * @param y0  y-value of first known point
 * @param x1  x-value of second known point
 * @param y1  y-value of second known point
 * @param x   x-value of point for which y is unknown
 *
 * @return linearly interpolated y-value at the given x
 *
 * @throws Runtime error if slope between given points is undefined.
 */
double Interpolate(double x0, double y0, double x1, double y1, double x) {

  if (x1 == x0) {
    throw std::runtime_error(std::format(
      "Slope is undefined between ({}, {}) and ({}, {}). "
      "A y-value cannot be calculated between these points.",
      x0, y0, x1, y1));
    // return 0; // TODO how to indicate error?
  }
  
  return ((y0 * (x1 - x)) + (y1 * (x - x0))) / (x1 - x0);
}
