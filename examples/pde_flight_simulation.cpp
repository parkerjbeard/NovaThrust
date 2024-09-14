#include "flight_dynamics/flight_dynamics_integrator.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>

class SimplePDESimulator : public PDESimulator {
public:
    void updateConditions(const FlightDynamicsIntegrator::PDEFeedback& feedback) override {
        // Simple PDE model that adjusts thrust based on altitude and Mach number
        double thrust_factor = std::exp(-feedback.altitude / 10000.0) * (1.0 + 0.2 * feedback.mach_number);
        current_thrust = base_thrust * thrust_factor;
    }

    PDEState computeCycle(double dt) override {
        PDEState state;
        state.fuel_mass_flow_rate = 0.5;  // kg/s
        state.oxidizer_mass_flow_rate = 2.0;  // kg/s
        state.chamber_pressure = 2e6;  // Pa
        state.chamber_temperature = 2500;  // K
        state.exhaust_velocity = 2500;  // m/s
        return state;
    }

    Eigen::Vector3d getThrust() const {
        return Eigen::Vector3d(current_thrust, 0, 0);  // Assume thrust is always along the x-axis
    }

private:
    double base_thrust = 50000.0;  // N
    double current_thrust = base_thrust;
};

void writeTrajectoryToFile(const std::vector<FlightDynamicsIntegrator::FlightState>& trajectory, const std::string& filename) {
    std::ofstream file(filename);
    file << "Time,X,Y,Z,Vx,Vy,Vz,Qw,Qx,Qy,Qz\n";
    for (const auto& state : trajectory) {
        file << state.time << ","
             << state.vehicle_state.position.x() << ","
             << state.vehicle_state.position.y() << ","
             << state.vehicle_state.position.z() << ","
             << state.vehicle_state.velocity.x() << ","
             << state.vehicle_state.velocity.y() << ","
             << state.vehicle_state.velocity.z() << ","
             << state.vehicle_state.orientation.w() << ","
             << state.vehicle_state.orientation.x() << ","
             << state.vehicle_state.orientation.y() << ","
             << state.vehicle_state.orientation.z() << "\n";
    }
}

int main() {
    // Initialize FlightDynamicsIntegrator
    double initial_mass = 1000.0;  // kg
    Eigen::Matrix3d initial_inertia;
    initial_inertia << 1000, 0, 0,
                       0, 2000, 0,
                       0, 0, 1500;
    double reference_area = 10.0;  // m^2
    double reference_length = 5.0;  // m

    FlightDynamicsIntegrator fdi(initial_mass, initial_inertia, reference_area, reference_length);

    // Initialize PDE simulator
    auto pde_sim = std::make_shared<SimplePDESimulator>();
    fdi.setPDESimulator(pde_sim);

    // Set up simulation parameters
    double dt = 0.01;  // Time step (s)
    double total_time = 60.0;  // Total simulation time (s)
    int num_steps = static_cast<int>(total_time / dt);

    // Set initial conditions
    FlightDynamicsIntegrator::FlightState& initial_state = fdi.getFlightState();
    initial_state.vehicle_state.position = Eigen::Vector3d(0, 0, 1000);  // Start at 1000m altitude
    initial_state.vehicle_state.velocity = Eigen::Vector3d(100, 0, 0);   // Initial velocity 100 m/s in x direction

    // Prepare for data collection
    std::vector<FlightDynamicsIntegrator::FlightState> trajectory;

    // Run simulation loop
    for (int step = 0; step < num_steps; ++step) {
        // Get current flight state
        const auto& current_state = fdi.getFlightState();

        // Store current state in trajectory
        trajectory.push_back(current_state);

        // Print some information every second
        if (step % 100 == 0) {
            std::cout << "Time: " << current_state.time
                      << " s, Altitude: " << (current_state.vehicle_state.position.norm() - 6371000.0)
                      << " m, Velocity: " << current_state.vehicle_state.velocity.norm() << " m/s\n";
        }

        // Run a simulation step
        fdi.runSimulationStep(dt);

        // Update PDE thrust based on new conditions
        FlightDynamicsIntegrator::PDEFeedback feedback = fdi.getPDEFeedback();
        pde_sim->updateConditions(feedback);
        Eigen::Vector3d new_thrust = pde_sim->getThrust();
        fdi.setEngineThrust(new_thrust);

        // Update control surfaces (in a real simulation, this would be done by a control system)
        AerodynamicsModel::ControlSurfaces controls = {0.1 * std::sin(0.1 * current_state.time), 0, 0};
        fdi.setControlSurfaces(controls);
    }

    // Write trajectory to file
    writeTrajectoryToFile(trajectory, "pde_flight_trajectory.csv");

    std::cout << "Simulation complete. Trajectory written to 'pde_flight_trajectory.csv'.\n";

    return 0;
}
