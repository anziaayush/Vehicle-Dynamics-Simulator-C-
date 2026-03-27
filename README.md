# Vehicle Dynamics Simulator

A C++ simulation of real-world vehicle physics, modelling engine force, aerodynamic drag, and rolling friction across four driving scenarios. Built to demonstrate applied physics, numerical integration, and clean C++ architecture.

---

## What It Does

This simulator models how a car accelerates, brakes, and cruises by applying Newton's second law at every timestep. At each 0.01-second interval, the program calculates the forces acting on the vehicle, derives acceleration, and integrates velocity and position forward in time.

The results are exported as CSV files, ready for analysis or plotting in Python, MATLAB, or Excel.

---

## Physics Model

At every timestep the following forces are calculated:

```
Total Force = Engine Force − Aerodynamic Drag − Rolling Friction

Aerodynamic Drag  = 0.5 × drag_coefficient × velocity²
Rolling Friction  = rolling_resistance × mass × g

Acceleration = Total Force / mass        (Newton's 2nd Law: F = ma)
Velocity     = velocity + acceleration × dt
Position     = position + velocity × dt
```

This is a first-order Euler integration method — the same foundational approach used in real automotive and aerospace simulation.

---

## The 4 Simulation Scenarios

| Scenario | Description | Output File |
|---|---|---|
| Full Acceleration | Full throttle from 0, runs for 10 seconds | `data/acceleration.csv` |
| Emergency Braking | Starting at 100 km/h, maximum brake force applied | `data/braking.csv` |
| Cruise Control | Maintains 80 km/h using a proportional speed controller | `data/cruise.csv` |
| Mixed Profile | Accelerate (0–5s) → Cruise (5–15s) → Brake (15–20s) | `data/simulation_results.csv` |

---

## Project Structure

```
Vehicle-Dynamics-Simulator-C-/
├── src/
│   ├── Main.cpp        — Entry point, runs all 4 simulations
│   ├── Vehicle.cpp     — Physics engine implementation
│   └── Vehicle.h       — VehicleSimulator class and VehicleState struct
├── data/               — CSV output files (generated on run)
├── .vscode/
│   └── tasks.json      — VS Code build configuration
├── CMakeLists.txt      — CMake build alternative
└── .gitignore
```

---

## How to Build and Run

### Prerequisites
- macOS or Linux
- g++ (C++17 or later)

### Using the terminal

```bash
# Clone the repository
git clone https://github.com/anziaayush/Vehicle-Dynamics-Simulator-C-.git
cd Vehicle-Dynamics-Simulator-C-

# Create output directories
mkdir -p src/output data

# Compile both source files together
g++ -Wall -Wextra -g3 src/Main.cpp src/Vehicle.cpp -o src/output/Vehicle

# Run
./src/output/Vehicle
```

### Using VS Code
Open the project folder in VS Code and press **Cmd + Shift + B** to build, then run the output binary from the terminal.

---

## Example Output

```
=== Vehicle Dynamics Simulator ===

1. Full Acceleration...
2. Emergency Braking...
3. Cruise Control (80 km/h)...
4. Mixed Profile...

All simulations completed!
Results saved to data/*.csv

Mixed profile summary:
Max velocity: 83.4 km/h
Final position: 842.3 m
Total time: 20.0 s
```

---

## CSV Output Format

Each output file contains one row per timestep with the following columns:

| Column | Unit | Description |
|---|---|---|
| time | s | Simulation time |
| position | m | Distance travelled |
| velocity | m/s | Current speed |
| acceleration | m/s² | Current acceleration |
| engine_force | N | Force applied by engine |
| drag_force | N | Aerodynamic resistance |
| friction_force | N | Rolling resistance |

---

## Technologies

- **Language:** C++17
- **Build:** g++ / CMake
- **Output:** CSV (compatible with Excel, Python, MATLAB)

---

## Author

**Aayush Anzi**  
[github.com/anziaayush](https://github.com/anziaayush)
