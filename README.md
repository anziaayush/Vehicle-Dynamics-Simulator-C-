# Vehicle Dynamics Simulator (C++)

Simplified longitudinal vehicle dynamics model simulating acceleration, velocity, and position. Supports various driving profiles with CSV export for analysis.

## Goal

- Implement basic vehicle longitudinal dynamics
- Simulate acceleration, braking, constant speed profiles
- Generate time-series data for analysis
- Clean C++17 implementation with CMake

## Features

- Realistic physics (force, mass, drag, friction)
- Multiple driving profiles (acceleration, braking, cruise)
- High-precision double arithmetic
- CSV export with timestamps
- Configurable simulation parameters

## Build & Run

```bash
git clone https://github.com/<your-username>/vehicle-dynamics-simulator.git
cd vehicle-dynamics-simulator

mkdir build && cd build
cmake .. 
make -j4

# Run simulation
./vehicle_sim
