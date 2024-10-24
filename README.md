# lla-to-ecef
Project to create an LLA to ECEF converter with ability to calculate and interpolate ECEF velocites.

## Build Instructions

This project was developed using C++20. 

### Prerequisites

User must have installed:

- `g++`
- `gtest`

to use these build instructions.

### Build and Run Program

```bash
g++ -std=c++20 simple.cpp time_utils.cpp main.cpp -o build/lla_to_ecef_run; ./build/lla_to_ecef_run
```

### Build and Run Tests

```bash
g++ -std=c++20 simple.cpp time_utils.cpp simple_unittest.cpp -lgtest -lgtest_main -o build/lla_to_ecef_unittest; ./build/lla_to_ecef_unittest
```


```bash
g++ -std=c++20 time_utils.cpp time_utils_unittest.cpp -lgtest -lgtest_main -o build/time_utils_unittest; ./build/time_utils_unittest
```