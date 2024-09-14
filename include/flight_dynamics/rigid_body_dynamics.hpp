#ifndef RIGID_BODY_DYNAMICS_HPP
#define RIGID_BODY_DYNAMICS_HPP

#include <array>
#include <cmath>
#include <Eigen/Dense>

class RigidBodyDynamics {
public:
    struct VehicleState {
        Eigen::Vector3d position;    // Position in ECEF frame (m)
        Eigen::Vector3d velocity;    // Velocity in ECEF frame (m/s)
        Eigen::Quaterniond orientation; // Orientation quaternion
        Eigen::Vector3d angular_velocity; // Angular velocity in body frame (rad/s)
        double mass;                 // Current mass (kg)
        Eigen::Matrix3d inertia;     // Current inertia tensor (kg*m^2)
    };

    struct Forces {
        Eigen::Vector3d force;       // Total force in body frame (N)
        Eigen::Vector3d moment;      // Total moment in body frame (N*m)
    };

    RigidBodyDynamics(double initial_mass, const Eigen::Matrix3d& initial_inertia);
    ~RigidBodyDynamics() = default;

    Eigen::Vector3d computeLinearAcceleration(const VehicleState& state, const Forces& forces) const;
    Eigen::Vector3d computeAngularAcceleration(const VehicleState& state, const Forces& forces) const;
    void integrateMotion(VehicleState& state, const Forces& forces, double dt) const;
    void updateMassProperties(VehicleState& state, double mass_flow_rate, double dt) const;

private:
    static constexpr double EARTH_ANGULAR_VELOCITY = 7.2921150e-5; // rad/s

    Eigen::Matrix3d computeInertiaMatrixInverse(const Eigen::Matrix3d& inertia) const;
    Eigen::Vector3d computeCoriolisEffect(const VehicleState& state) const;
    Eigen::Matrix3d computeRotationMatrix(const Eigen::Quaterniond& q) const;
};

#endif // RIGID_BODY_DYNAMICS_HPP
