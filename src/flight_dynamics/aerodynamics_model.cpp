#include "flight_dynamics/aerodynamics_model.hpp"
#include <algorithm>

AerodynamicsModel::AerodynamicsModel(const AtmosphericModel& atm_model)
    : atm_model(atm_model) {
    // Initialize aerodynamic coefficients (these should be loaded from a file in a real implementation)
    cl_alpha = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
    cd_alpha = {0.01, 0.015, 0.02, 0.03, 0.04, 0.06, 0.08, 0.1, 0.12, 0.15};
    cm_alpha = {0.0, -0.01, -0.02, -0.03, -0.04, -0.05, -0.06, -0.07, -0.08, -0.09};
    cy_beta = -0.1;
    cn_beta = 0.05;
    cl_beta = -0.01;

    // Control surface effectiveness
    cl_elevator = 0.1;
    cm_elevator = -0.2;
    cl_aileron = 0.1;
    cn_rudder = 0.1;
}

AerodynamicsModel::AeroForces AerodynamicsModel::computeAeroForces(const VehicleState& state, const ControlSurfaces& controls) const {
    double mach = calculateMachNumber(state);
    auto [alpha, beta] = calculateAngleOfAttackAndSideslip(state);

    AeroForces forces;
    if (mach < 0.8) {
        forces = computeSubsonicAero(state, alpha, beta);
    } else if (mach < 5.0) {
        forces = computeSupersonicAero(state, alpha, beta);
    } else {
        forces = computeHypersonicAero(state, alpha, beta);
    }

    updateControlSurfaceEffects(forces, state, controls);
    return forces;
}

AerodynamicsModel::AeroForces AerodynamicsModel::computeAeroMoments(const VehicleState& state, const ControlSurfaces& controls) const {
    // For simplicity, we'll compute moments along with forces
    return computeAeroForces(state, controls);
}

void AerodynamicsModel::updateControlSurfaceEffects(AeroForces& forces, const VehicleState& state, const ControlSurfaces& controls) const {
    double q = 0.5 * atm_model.getAtmosphericProperties(state.position[2]).density * 
               (state.velocity[0]*state.velocity[0] + state.velocity[1]*state.velocity[1] + state.velocity[2]*state.velocity[2]);

    // Update lift and pitch moment due to elevator
    forces.force[0] += q * state.reference_area * cl_elevator * controls.elevator;
    forces.moment[1] += q * state.reference_area * state.reference_length * cm_elevator * controls.elevator;

    // Update roll moment due to aileron
    forces.moment[0] += q * state.reference_area * state.reference_length * cl_aileron * controls.aileron;

    // Update yaw moment due to rudder
    forces.moment[2] += q * state.reference_area * state.reference_length * cn_rudder * controls.rudder;
}

double AerodynamicsModel::calculateMachNumber(const VehicleState& state) const {
    double altitude = std::sqrt(state.position[0]*state.position[0] + state.position[1]*state.position[1] + state.position[2]*state.position[2]) - 6371000.0; // Earth radius in meters
    double temperature = atm_model.getAtmosphericProperties(altitude).temperature;
    double speed_of_sound = atm_model.getSpeedOfSound(temperature);
    double velocity_magnitude = std::sqrt(state.velocity[0]*state.velocity[0] + state.velocity[1]*state.velocity[1] + state.velocity[2]*state.velocity[2]);
    return velocity_magnitude / speed_of_sound;
}

std::array<double, 2> AerodynamicsModel::calculateAngleOfAttackAndSideslip(const VehicleState& state) const {
    // This is a simplified calculation and assumes the body frame is aligned with the velocity vector when attitude is zero
    double vx = state.velocity[0] * std::cos(state.attitude[1]) * std::cos(state.attitude[2]) +
                state.velocity[1] * (std::sin(state.attitude[0]) * std::sin(state.attitude[1]) * std::cos(state.attitude[2]) - std::cos(state.attitude[0]) * std::sin(state.attitude[2])) +
                state.velocity[2] * (std::cos(state.attitude[0]) * std::sin(state.attitude[1]) * std::cos(state.attitude[2]) + std::sin(state.attitude[0]) * std::sin(state.attitude[2]));
    
    double vy = state.velocity[0] * std::cos(state.attitude[1]) * std::sin(state.attitude[2]) +
                state.velocity[1] * (std::sin(state.attitude[0]) * std::sin(state.attitude[1]) * std::sin(state.attitude[2]) + std::cos(state.attitude[0]) * std::cos(state.attitude[2])) +
                state.velocity[2] * (std::cos(state.attitude[0]) * std::sin(state.attitude[1]) * std::sin(state.attitude[2]) - std::sin(state.attitude[0]) * std::cos(state.attitude[2]));
    
    double vz = -state.velocity[0] * std::sin(state.attitude[1]) +
                state.velocity[1] * std::sin(state.attitude[0]) * std::cos(state.attitude[1]) +
                state.velocity[2] * std::cos(state.attitude[0]) * std::cos(state.attitude[1]);

    double alpha = std::atan2(vz, vx);
    double beta = std::atan2(vy, vx);

    return {alpha, beta};
}

double AerodynamicsModel::interpolateCoefficient(const std::vector<double>& coeff_table, double alpha) const {
    double alpha_deg = alpha * 180.0 / M_PI;
    int index = std::min(static_cast<int>(alpha_deg / 10.0), static_cast<int>(coeff_table.size()) - 2);
    double t = (alpha_deg - index * 10.0) / 10.0;
    return coeff_table[index] * (1.0 - t) + coeff_table[index + 1] * t;
}

AerodynamicsModel::AeroForces AerodynamicsModel::computeSubsonicAero(const VehicleState& state, double alpha, double beta) const {
    AeroForces forces;
    double q = 0.5 * atm_model.getAtmosphericProperties(state.position[2]).density * 
               (state.velocity[0]*state.velocity[0] + state.velocity[1]*state.velocity[1] + state.velocity[2]*state.velocity[2]);

    double cl = interpolateCoefficient(cl_alpha, alpha);
    double cd = interpolateCoefficient(cd_alpha, alpha);
    double cm = interpolateCoefficient(cm_alpha, alpha);

    forces.force[0] = q * state.reference_area * cl;  // Lift
    forces.force[1] = q * state.reference_area * cd;  // Drag
    forces.force[2] = q * state.reference_area * cy_beta * beta;  // Side force

    forces.moment[0] = q * state.reference_area * state.reference_length * cl_beta * beta;  // Roll
    forces.moment[1] = q * state.reference_area * state.reference_length * cm;  // Pitch
    forces.moment[2] = q * state.reference_area * state.reference_length * cn_beta * beta;  // Yaw

    return forces;
}

AerodynamicsModel::AeroForces AerodynamicsModel::computeSupersonicAero(const VehicleState& state, double alpha, double beta) const {
    // For simplicity, we'll use the same method as subsonic, but in a real implementation this would be different
    return computeSubsonicAero(state, alpha, beta);
}

AerodynamicsModel::AeroForces AerodynamicsModel::computeHypersonicAero(const VehicleState& state, double alpha, double beta) const {
    // For simplicity, we'll use the same method as subsonic, but in a real implementation this would be different
    return computeSubsonicAero(state, alpha, beta);
}