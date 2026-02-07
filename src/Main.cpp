#include "vehicle.h"
#include <iostream>
#include <iomanip>

int main() {
    VehicleSimulator sim;
    
    std::cout << "=== Vehicle Dynamics Simulator ===\n\n";
    
    // Run all simulations
    std::cout << "1. Full Acceleration...\n";
    sim.simulate_acceleration(10.0);
    sim.save_to_csv("data/acceleration.csv");
    
    std::cout << "2. Emergency Braking...\n";
    sim.simulate_braking(5.0);
    sim.save_to_csv("data/braking.csv");
    
    std::cout << "3. Cruise Control (80 km/h)...\n";
    sim.simulate_cruise(22.22, 10.0);  // 80 km/h
    sim.save_to_csv("data/cruise.csv");
    
    std::cout << "4. Mixed Profile...\n";
    sim.simulate_mixed_profile(20.0);
    sim.save_to_csv("data/simulation_results.csv");
    
    std::cout << "\nAll simulations completed!\n";
    std::cout << "Results saved to data/*.csv\n";
    
    auto states = sim.get_states();
    std::cout << "\nMixed profile summary:\n";
    std::cout << "Max velocity: " << std::fixed << std::setprecision(1) 
              << sim.get_states().back().velocity * 3.6 << " km/h\n";
    std::cout << "Final position: " << states.back().position << " m\n";
    std::cout << "Total time: " << states.back().time << " s\n";
    
    return 0;
}
