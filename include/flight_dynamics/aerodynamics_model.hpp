#ifndef AERODYNAMICS_MODEL_HPP
#define AERODYNAMICS_MODEL_HPP

#include <array>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "atmospheric_model.hpp"

class AerodynamicsModel {
public:
    struct VehicleState {
        Eigen::Vector3d position;  // x, y, z in Earth-centered Earth-fixed (ECEF) frame
        Eigen::Vector3d velocity;  // vx, vy, vz in ECEF frame
        Eigen::Vector3d attitude;  // roll, pitch, yaw in radians
        double mass;                     // kg
        double reference_area;           // m^2
        double reference_length;         // m
    };

    struct ControlSurfaces {
        double elevator;     // Elevator deflection in radians
        double aileron;      // Aileron deflection in radians
        double rudder;       // Rudder deflection in radians
    };

    struct AeroForces {
        Eigen::Vector3d force;     // Lift, Drag, Side force
        Eigen::Vector3d moment;    // Pitch, Roll, Yaw moment
    };

    AerodynamicsModel(const AtmosphericModel& atm_model);
    ~AerodynamicsModel() = default;

    AeroForces computeAeroForces(const VehicleState& state, const ControlSurfaces& controls) const;
    AeroForces computeAeroMoments(const VehicleState& state, const ControlSurfaces& controls) const;
    void updateControlSurfaceEffects(AeroForces& forces, const VehicleState& state, const ControlSurfaces& controls) const;

private:
    const AtmosphericModel& atm_model;

    // Aerodynamic coefficients (these would typically be loaded from a database or file)
    std::vector<double> cl_alpha;  // Lift coefficient vs angle of attack
    std::vector<double> cd_alpha;  // Drag coefficient vs angle of attack
    std::vector<double> cm_alpha;  // Pitch moment coefficient vs angle of attack
    double cy_beta;                // Side force coefficient vs sideslip angle
    double cn_beta;                // Yaw moment coefficient vs sideslip angle
    double cl_beta;                // Roll moment coefficient vs sideslip angle

    // Control surface effectiveness
    double cl_elevator;
    double cm_elevator;
    double cl_aileron;
    double cn_rudder;

    double calculateMachNumber(const VehicleState& state) const;
    std::array<double, 2> calculateAngleOfAttackAndSideslip(const VehicleState& state) const;
    double interpolateCoefficient(const std::vector<double>& coeff_table, double alpha) const;
    AeroForces computeSubsonicAero(const VehicleState& state, double alpha, double beta) const;
    AeroForces computeSupersonicAero(const VehicleState& state, double alpha, double beta) const;
    AeroForces computeHypersonicAero(const VehicleState& state, double alpha, double beta) const;
};

#endif // AERODYNAMICS_MODEL_HPP
