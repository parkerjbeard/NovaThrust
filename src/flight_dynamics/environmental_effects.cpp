#include "flight_dynamics/environmental_effects.hpp"
#include <cmath>

EnvironmentalEffects::EnvironmentalEffects(const AtmosphericModel& atm_model)
    : atm_model(atm_model) {}

void EnvironmentalEffects::applyWindModel(VehicleState& state, double time) const {
    double altitude = calculateAltitude(state.position);
    AtmosphericModel::WindVector wind = atm_model.computeWindEffects(altitude, time);

    // Convert wind from local horizontal frame to ECEF
    double lat = std::asin(state.position.z() / EARTH_RADIUS);
    double lon = std::atan2(state.position.y(), state.position.x());

    Eigen::Vector3d wind_ecef;
    wind_ecef.x() = -wind.velocity * std::sin(wind.direction) * std::sin(lon) - 
                     wind.velocity * std::cos(wind.direction) * std::sin(lat) * std::cos(lon);
    wind_ecef.y() =  wind.velocity * std::sin(wind.direction) * std::cos(lon) - 
                     wind.velocity * std::cos(wind.direction) * std::sin(lat) * std::sin(lon);
    wind_ecef.z() =  wind.velocity * std::cos(wind.direction) * std::cos(lat);

    // Apply wind effect to velocity
    state.velocity += wind_ecef;
}

Eigen::Vector3d EnvironmentalEffects::applyGravityModel(const Eigen::Vector3d& position) const {
    Eigen::Vector3d acceleration;
    double r = position.norm();
    double r3 = r * r * r;

    // Central gravity term
    acceleration = -EARTH_MU * position / r3;

    // Add J2 perturbation
    acceleration += calculateJ2Perturbation(position);

    return acceleration;
}

Eigen::Vector3d EnvironmentalEffects::computeSolarEffects(const VehicleState& state, double time) const {
    // Simplified solar radiation pressure model
    // Assuming the vehicle is always in sunlight and the Sun is in the +X direction of the ECEF frame
    
    double solar_constant = SOLAR_FLUX / 299792458.0; // N/m^2
    double pressure = solar_constant * (1.0 + 0.007 * std::cos(2 * M_PI * time / (365.25 * 24 * 3600))); // Annual variation

    Eigen::Vector3d acceleration;
    double force = pressure * state.area / state.mass;
    acceleration << force, 0.0, 0.0;

    return acceleration;
}

double EnvironmentalEffects::calculateAltitude(const Eigen::Vector3d& position) const {
    return position.norm() - EARTH_RADIUS;
}

Eigen::Vector3d EnvironmentalEffects::calculateJ2Perturbation(const Eigen::Vector3d& position) const {
    Eigen::Vector3d perturbation;
    double r = position.norm();
    double r2 = r * r;
    double r4 = r2 * r2;

    perturbation = -1.5 * J2 * EARTH_MU * EARTH_RADIUS * EARTH_RADIUS * position * 
        (5 * position.z() * position.z() / r2 - 1) / r4;

    perturbation.z() += -1.5 * J2 * EARTH_MU * EARTH_RADIUS * EARTH_RADIUS * position.z() * 
        (5 * position.z() * position.z() / r2 - 3) / r4;

    return perturbation;
}
