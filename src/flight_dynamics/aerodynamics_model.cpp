#include "flight_dynamics/aerodynamics_model.hpp"
#include <algorithm>
#include <Eigen/Geometry>

// Constants for WGS84
constexpr double WGS84_A = 6378137.0;            // Semi-major axis in meters
constexpr double WGS84_F = 1.0 / 298.257223563;  // Flattening
constexpr double WGS84_B = WGS84_A * (1.0 - WGS84_F); // Semi-minor axis

AerodynamicsModel::AerodynamicsModel(const AtmosphericModel& atm_model)
    : atm_model(atm_model) {
    // Initialize aerodynamic coefficients for subsonic flow
    cl_alpha = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
    cd_alpha = {0.01, 0.015, 0.02, 0.03, 0.04, 0.06, 0.08, 0.1, 0.12, 0.15};
    cm_alpha = {0.0, -0.01, -0.02, -0.03, -0.04, -0.05, -0.06, -0.07, -0.08, -0.09};
    
    // Initialize aerodynamic coefficients for supersonic flow
    cl_alpha_supersonic = {0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45};
    cd_alpha_supersonic = {0.02, 0.025, 0.03, 0.035, 0.04, 0.045, 0.05, 0.055, 0.06, 0.065};
    cm_alpha_supersonic = {0.0, -0.005, -0.010, -0.015, -0.020, -0.025, -0.030, -0.035, -0.040, -0.045};
    
    // Initialize aerodynamic coefficients for hypersonic flow
    cl_alpha_hypersonic = {0.0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.14, 0.16, 0.18};
    cd_alpha_hypersonic = {0.03, 0.035, 0.040, 0.045, 0.050, 0.055, 0.060, 0.065, 0.070, 0.075};
    cm_alpha_hypersonic = {0.0, -0.002, -0.004, -0.006, -0.008, -0.010, -0.012, -0.014, -0.016, -0.018};
    
    cy_beta = -0.1;
    cn_beta = 0.05;
    cl_beta = -0.01;

    // Control surface effectiveness
    cl_elevator = 0.1;
    cm_elevator = -0.2;
    cl_aileron = 0.1;
    cn_rudder = 0.1;

    // Documenting body frame axes
    // X - Forward
    // Y - Right
    // Z - Down
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
    double velocity_magnitude = state.velocity.norm();
    double q = 0.5 * atm_model.getAtmosphericProperties(state.position[2]).density * 
               (velocity_magnitude * velocity_magnitude);

    // Clamp control surface deflections to realistic limits to handle potential nonlinearities
    double elevator = std::max(std::min(controls.elevator, 0.3), -0.3); // radians
    double aileron = std::max(std::min(controls.aileron, 0.3), -0.3);   // radians
    double rudder = std::max(std::min(controls.rudder, 0.3), -0.3);     // radians

    // Update lift and pitch moment due to elevator
    forces.force[0] += q * state.reference_area * cl_elevator * elevator;
    forces.moment[1] += q * state.reference_area * state.reference_length * cm_elevator * elevator;

    // Update roll moment due to aileron
    forces.moment[0] += q * state.reference_area * state.reference_length * cl_aileron * aileron;

    // Update yaw moment due to rudder
    forces.moment[2] += q * state.reference_area * state.reference_length * cn_rudder * rudder;
}

double AerodynamicsModel::calculateMachNumber(const VehicleState& state) const {
    double altitude = calculateGeodeticAltitude(state.position);
    double temperature = atm_model.getAtmosphericProperties(altitude).temperature;
    double speed_of_sound = atm_model.getSpeedOfSound(temperature);
    double velocity_magnitude = state.velocity.norm();
    return velocity_magnitude / speed_of_sound;
}

std::array<double, 2> AerodynamicsModel::calculateAngleOfAttackAndSideslip(const VehicleState& state) const {
    // Transformation from ECEF to Body frame using rotation matrix
    // Body frame axes:
    // X - Forward
    // Y - Right
    // Z - Down

    Eigen::Vector3d velocity_body;
    Eigen::Vector3d velocity_ecef = state.velocity;

    // Create rotation matrix from roll, pitch, yaw
    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(state.attitude[2], Eigen::Vector3d::UnitZ()) *  // Yaw
               Eigen::AngleAxisd(state.attitude[1], Eigen::Vector3d::UnitY()) *  // Pitch
               Eigen::AngleAxisd(state.attitude[0], Eigen::Vector3d::UnitX());   // Roll

    velocity_body = rotation * velocity_ecef;

    double vx = velocity_body[0];
    double vy = velocity_body[1];
    double vz = velocity_body[2];

    double alpha = std::atan2(vz, vx);
    double beta = std::atan2(vy, vx);

    return {alpha, beta};
}

double AerodynamicsModel::interpolateCoefficient(const std::vector<double>& coeff_table, double alpha, double mach) const {
    // Convert alpha to degrees for interpolation
    double alpha_deg = alpha * 180.0 / M_PI;

    // Determine which coefficient table to use based on Mach number
    const std::vector<double>& table = cl_alpha_supersonic; // Example for supersonic
    // Implement higher-order interpolation (cubic)
    if (coeff_table.empty()) {
        return 0.0;
    }

    // Clamp alpha_deg to the range of the table
    double alpha_min = 0.0;
    double alpha_max = 90.0; // Assuming table is from 0 to 90 degrees
    if (alpha_deg < alpha_min) {
        alpha_deg = alpha_min;
    } else if (alpha_deg > alpha_max) {
        alpha_deg = alpha_max;
    }

    // Assuming the table has entries every 10 degrees
    int index = static_cast<int>(alpha_deg / 10.0);
    if (index >= static_cast<int>(coeff_table.size()) - 1) {
        index = coeff_table.size() - 2;
    }

    double t = (alpha_deg - index * 10.0) / 10.0;

    // Cubic interpolation using surrounding points
    // Ensure indices are within bounds
    int i0 = std::max(index - 1, 0);
    int i1 = index;
    int i2 = index + 1;
    int i3 = std::min(index + 2, static_cast<int>(coeff_table.size()) - 1);

    double y0 = coeff_table[i0];
    double y1 = coeff_table[i1];
    double y2 = coeff_table[i2];
    double y3 = coeff_table[i3];

    // Cubic Hermite spline interpolation
    double a = (-0.5*y0 + 1.5*y1 - 1.5*y2 + 0.5*y3);
    double b = (y0 - 2.5*y1 + 2.0*y2 - 0.5*y3);
    double c = (-0.5*y0 + 0.5*y2);
    double d = y1;

    double interpolated = a * t * t * t + b * t * t + c * t + d;
    return interpolated;
}

AerodynamicsModel::AeroForces AerodynamicsModel::computeSubsonicAero(const VehicleState& state, double alpha, double beta) const {
    AeroForces forces;
    double velocity_magnitude = state.velocity.norm();
    double q = 0.5 * atm_model.getAtmosphericProperties(state.position[2]).density * 
               (velocity_magnitude * velocity_magnitude);

    double cl = interpolateCoefficient(cl_alpha, alpha, 0.7); // Example Mach number for subsonic
    double cd = interpolateCoefficient(cd_alpha, alpha, 0.7);
    double cm = interpolateCoefficient(cm_alpha, alpha, 0.7);

    forces.force[0] = q * state.reference_area * cl;  // Lift
    forces.force[1] = q * state.reference_area * cd;  // Drag
    forces.force[2] = q * state.reference_area * cy_beta * beta;  // Side force

    forces.moment[0] = q * state.reference_area * state.reference_length * cl_beta * beta;  // Roll
    forces.moment[1] = q * state.reference_area * state.reference_length * cm;  // Pitch
    forces.moment[2] = q * state.reference_area * state.reference_length * cn_beta * beta;  // Yaw

    return forces;
}

AerodynamicsModel::AeroForces AerodynamicsModel::computeSupersonicAero(const VehicleState& state, double alpha, double beta) const {
    AeroForces forces;
    double velocity_magnitude = state.velocity.norm();
    double q = 0.5 * atm_model.getAtmosphericProperties(state.position[2]).density * 
               (velocity_magnitude * velocity_magnitude);

    double cl = interpolateCoefficient(cl_alpha_supersonic, alpha, 2.0); // Example Mach number for supersonic
    double cd = interpolateCoefficient(cd_alpha_supersonic, alpha, 2.0);
    double cm = interpolateCoefficient(cm_alpha_supersonic, alpha, 2.0);

    forces.force[0] = q * state.reference_area * cl;  // Lift
    forces.force[1] = q * state.reference_area * cd;  // Drag
    forces.force[2] = q * state.reference_area * cy_beta * beta;  // Side force

    forces.moment[0] = q * state.reference_area * state.reference_length * cl_beta * beta;  // Roll
    forces.moment[1] = q * state.reference_area * state.reference_length * cm;  // Pitch
    forces.moment[2] = q * state.reference_area * state.reference_length * cn_beta * beta;  // Yaw

    return forces;
}

AerodynamicsModel::AeroForces AerodynamicsModel::computeHypersonicAero(const VehicleState& state, double alpha, double beta) const {
    AeroForces forces;
    double velocity_magnitude = state.velocity.norm();
    double q = 0.5 * atm_model.getAtmosphericProperties(state.position[2]).density * 
               (velocity_magnitude * velocity_magnitude);

    double cl = interpolateCoefficient(cl_alpha_hypersonic, alpha, 10.0); // Example Mach number for hypersonic
    double cd = interpolateCoefficient(cd_alpha_hypersonic, alpha, 10.0);
    double cm = interpolateCoefficient(cm_alpha_hypersonic, alpha, 10.0);

    forces.force[0] = q * state.reference_area * cl;  // Lift
    forces.force[1] = q * state.reference_area * cd;  // Drag
    forces.force[2] = q * state.reference_area * cy_beta * beta;  // Side force

    forces.moment[0] = q * state.reference_area * state.reference_length * cl_beta * beta;  // Roll
    forces.moment[1] = q * state.reference_area * state.reference_length * cm;  // Pitch
    forces.moment[2] = q * state.reference_area * state.reference_length * cn_beta * beta;  // Yaw

    return forces;
}

double AerodynamicsModel::calculateGeodeticAltitude(const Eigen::Vector3d& position_ecef) const {
    double x = position_ecef[0];
    double y = position_ecef[1];
    double z = position_ecef[2];

    double a = WGS84_A;
    double e_sq = 2 * WGS84_F - WGS84_F * WGS84_F;
    double b = WGS84_B;
    double ep_sq = (a*a - b*b) / b*b;
    double p = std::sqrt(x*x + y*y);
    double th = std::atan2(a * z, b * p);
    double sin_th = std::sin(th);
    double cos_th = std::cos(th);

    double latitude = std::atan((z + ep_sq * b * sin_th * sin_th * sin_th) / (p - e_sq * a * cos_th * cos_th * cos_th));
    double N = a / std::sqrt(1 - e_sq * std::sin(latitude) * std::sin(latitude));
    double altitude = p / std::cos(latitude) - N;

    return altitude;
}