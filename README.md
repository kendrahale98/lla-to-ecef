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
g++ simple.cpp main.cpp -o simple-app; ./simple-app
```

### Build and Run Tests

```bash
g++ simple.cpp simple_unittest.cpp -lgtest -lgtest_main -o unit-test; ./unit-test
```

