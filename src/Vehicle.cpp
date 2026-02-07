#include "vehicle.h"
#include <fstream>
#include <iomanip>
#include <cmath>

void VehicleSimulator::simulate_acceleration(double duration, double dt) {
    states.clear();
    double t = 0.0;
    double pos = 0.0;
    double vel = 0.0;
    
    while (t <= duration) {
        double engine_force = max_engine_force;
        double drag = 0.5 * drag_coeff * vel * vel;
        double friction = rolling_resist * mass * 9.81;
        double total_force = engine_force - drag - friction;
        double accel = total_force / mass;
        
        vel += accel * dt;
        pos += vel * dt;
        
        states.push_back({t, pos, vel, accel, engine_force, drag, friction});
        t += dt;
    }
}

void VehicleSimulator::simulate_braking(double duration, double dt) {
    states.clear();
    double t = 0.0;
    double pos = 0.0;
    double vel = 27.78;  // 100 km/h
    
    while (t <= duration) {
        double brake_force = max_brake_force;
        double drag = 0.5 * drag_coeff * vel * vel;
        double friction = rolling_resist * mass * 9.81;
        double total_force = -brake_force - drag - friction;
        double accel = total_force / mass;
        
        vel = std::max(0.0, vel + accel * dt);
        pos += vel * dt;
        
        states.push_back({t, pos, vel, accel, 0.0, drag, friction});
        t += dt;
    }
}

void VehicleSimulator::simulate_cruise(double target_speed_ms, double duration, double dt) {
    states.clear();
    double t = 0.0;
    double pos = 0.0;
    double vel = 0.0;
    
    while (t <= duration) {
        double speed_error = target_speed_ms - vel;
        double engine_force = std::clamp(speed_error * 1000.0, 0.0, max_engine_force);
        double drag = 0.5 * drag_coeff * vel * vel;
        double friction = rolling_resist * mass * 9.81;
        double total_force = engine_force - drag - friction;
        double accel = total_force / mass;
        
        vel += accel * dt;
        pos += vel * dt;
        
        states.push_back({t, pos, vel, accel, engine_force, drag, friction});
        t += dt;
    }
}

void VehicleSimulator::simulate_mixed_profile(double total_time, double dt) {
    states.clear();
    
    // Phase 1: Accelerate to 25 m/s (90 km/h)
    double t = 0.0;
    double pos = 0.0;
    double vel = 0.0;
    
    // Phase 1: Acceleration (0-5s)
    while (t < 5.0) {
        double engine_force = max_engine_force;
        double drag = 0.5 * drag_coeff * vel * vel;
        double friction = rolling_resist * mass * 9.81;
        double accel = (engine_force - drag - friction) / mass;
        
        vel += accel * dt;
        pos += vel * dt;
        states.push_back({t, pos, vel, accel, engine_force, drag, friction});
        t += dt;
    }
    
    // Phase 2: Cruise (5-15s)
    double target_vel = vel;
    while (t < 15.0) {
        double speed_error = target_vel - vel;
        double engine_force = std::clamp(speed_error * 1000.0, 0.0, max_engine_force);
        double drag = 0.5 * drag_coeff * vel * vel;
        double friction = rolling_resist * mass * 9.81;
        double accel = (engine_force - drag - friction) / mass;
        
        vel += accel * dt;
        pos += vel * dt;
        states.push_back({t, pos, vel, accel, engine_force, drag, friction});
        t += dt;
    }
    
    // Phase 3: Braking (15-20s)
    while (t <= 20.0) {
        double brake_force = max_brake_force * (20.0 - t) / 5.0;
        double drag = 0.5 * drag_coeff * vel * vel;
        double friction = rolling_resist * mass * 9.81;
        double accel = (-brake_force - drag - friction) / mass;
        
        vel = std::max(0.0, vel + accel * dt);
        pos += vel * dt;
        states.push_back({t, pos, vel, accel, 0.0, drag, friction});
        t += dt;
    }
}

void VehicleSimulator::save_to_csv(const std::string& filename) const {
    std::ofstream file(filename);
    file << std::fixed << std::setprecision(3);
    file << "time,position,velocity,acceleration,engine_force,drag_force,friction_force\n";
    
    for (const auto& state : states) {
        file << state.time << "," << state.position << "," 
             << state.velocity << "," << state.acceleration << ","
             << state.engine_force << "," << state.drag_force << "," 
             << state.friction_force << "\n";
    }
    file.close();
}
