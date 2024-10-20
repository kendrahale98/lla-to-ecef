# lla-to-ecef
Project to create an LLA to ECEF converter with on-demand velocity request feature.

## Build Instructions

### Prerequisites

User must have installed:

- `g++`
- `gtest`

to use these build instructions.

### Build and Run Program

```bash
g++ simple.cc -o simple-app
./simple-app
```

### Build and Run Tests

```bash
g++ simple.cc simple_unittest.cc -lgtest -lgtest_main -o unit-test
./unit-test
```