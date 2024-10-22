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
g++ -std=c++20 simple.cpp main.cpp -o build/simple-app.o; ./build/simple-app.o
```

### Build and Run Tests

```bash
g++ -std=c++20 simple.cpp simple_unittest.cpp -lgtest -lgtest_main -o build/unit-test.o; ./build/unit-test.o
```

