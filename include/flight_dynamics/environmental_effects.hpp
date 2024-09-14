#ifndef ENVIRONMENTAL_EFFECTS_HPP
#define ENVIRONMENTAL_EFFECTS_HPP

#include <Eigen/Dense>
#include "atmospheric_model.hpp"

class EnvironmentalEffects {
public:
    struct VehicleState {
        Eigen::Vector3d position;  // x, y, z in Earth-centered Earth-fixed (ECEF) frame
        Eigen::Vector3d velocity;  // vx, vy, vz in ECEF frame
        double mass;               // kg
        double area;               // m^2 (cross-sectional area for wind effects)
    };

    EnvironmentalEffects(const AtmosphericModel& atm_model);
    ~EnvironmentalEffects() = default;

    void applyWindModel(VehicleState& state, double time) const;
    Eigen::Vector3d applyGravityModel(const Eigen::Vector3d& position) const;
    Eigen::Vector3d computeSolarEffects(const VehicleState& state, double time) const;

private:
    const AtmosphericModel& atm_model;

    static constexpr double EARTH_RADIUS = 6371000.0;  // m
    static constexpr double EARTH_MU = 3.986004418e14; // m^3/s^2
    static constexpr double J2 = 1.08263e-3;           // Earth's J2 coefficient
    static constexpr double SOLAR_FLUX = 1361.0;       // W/m^2 at 1 AU

    double calculateAltitude(const Eigen::Vector3d& position) const;
    Eigen::Vector3d calculateJ2Perturbation(const Eigen::Vector3d& position) const;
};

#endif // ENVIRONMENTAL_EFFECTS_HPP
