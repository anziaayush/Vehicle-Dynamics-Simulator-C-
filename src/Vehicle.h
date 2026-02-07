#pragma once
#include <vector>
#include <string>

struct VehicleState {
    double time;
    double position;
    double velocity;
    double acceleration;
    double engine_force;
    double drag_force;
    double friction_force;
};

class VehicleSimulator {
private:
    // Vehicle parameters
    double mass = 1500.0;        // kg
    double drag_coeff = 0.32;    // CdA
    double rolling_resist = 0.01;
    double max_engine_force = 4000.0;  // N
    double max_brake_force = 8000.0;   // N
    
    std::vector<VehicleState> states;
    
public:
    void simulate_acceleration(double duration, double dt = 0.01);
    void simulate_braking(double duration, double dt = 0.01);
    void simulate_cruise(double target_speed_ms, double duration, double dt = 0.01);
    void simulate_mixed_profile(double total_time, double dt = 0.01);
    
    void save_to_csv(const std::string& filename) const;
    std::vector<VehicleState> get_states() const { return states; }
};
