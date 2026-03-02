#include <iomanip>
#include <iostream>

#include "vehicle.h"

int main() {
  VehicleSimulator sim;

  std::cout << "=== Vehicle Dynamics Simulator ===\n\n";

  // 1. Acceleration test
  std::cout << "1. Acceleration (0-100 km/h)...\n";
  sim.simulate_acceleration(10.0);
  sim.save_to_csv("data/acceleration.csv");

  // 2. Braking test
  std::cout << "2. Emergency braking...\n";
  sim.simulate_braking(5.0);
  sim.save_to_csv("data/braking.csv");

  // 3. Cruise control
  std::cout << "3. Cruise control (80 km/h)...\n";
  sim.simulate_cruise(22.22, 10.0);  // 80 km/h = 22.22 m/s
  sim.save_to_csv("data/cruise.csv");

  // 4. Mixed profile
  std::cout << "4. Mixed profile...\n";
  sim.simulate_mixed_profile(20.0);
  sim.save_to_csv("data/simulation_results.csv");

  std::cout << "\n✅ All simulations completed!\n";
  std::cout << "📁 Check data/ folder for CSV files\n";

  return 0;
}
