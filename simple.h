// A simple program to sum numbers and report it to the console.

#include <time.h>

#ifndef SIMPLE_H_
#define SIMPLE_H_

struct PointCartesian2D {
    double x;
    double y;
};

struct PositionLLA {
    struct timespec t;  // time since Unix epoch
    double latitutde;   // degrees
    double longitude;   // degrees
    double altitude;    // kilometers
};

struct PositionVelocityECEF {
    struct timespec t;  // time since Unix epoch
    double x;           // kilometers
    double y;           // kilometers
    double z;           // kilometers
    double v_x;         // kilometers/second
    double v_y;         // kilometers/second
    double v_z;         // kilometers/second
};

// Returns the sum of the given integers.
int Sum(int a, int b);

double Interpolate(double x0, double y0, double x1, double y1, double x);

#endif  // SIMPLE_H_