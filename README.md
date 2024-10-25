# lla-to-ecef
Project to create an LLA to ECEF converter with ability to calculate and interpolate ECEF velocites.

## Build Instructions

This project was developed using C++20 on Ubuntu 24.04 LTS.

### Prerequisites

User must have installed:

- `g++`
- `gtest`

to use these build instructions.

### Build and Run Program

The provided input data is hard-coded in `main.cpp`.

To build and run the main code:

```bash
g++ -std=c++20 lla_to_ecef.cpp time_utils.cpp main.cpp -o build/lla_to_ecef_run; ./build/lla_to_ecef_run
```

### Build and Run Tests

To build and run unit tests for the main lla_to_ecef module:

```bash
g++ -std=c++20 lla_to_ecef.cpp time_utils.cpp lla_to_ecef_unittest.cpp -lgtest -lgtest_main -o build/lla_to_ecef_unittest; ./build/lla_to_ecef_unittest
```
To build and run unit tests for the time utilities:

```bash
g++ -std=c++20 time_utils.cpp time_utils_unittest.cpp -lgtest -lgtest_main -o build/time_utils_unittest; ./build/time_utils_unittest
```
