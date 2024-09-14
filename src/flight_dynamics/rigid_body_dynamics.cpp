#include "flight_dynamics/rigid_body_dynamics.hpp"

RigidBodyDynamics::RigidBodyDynamics(double initial_mass, const Eigen::Matrix3d& initial_inertia) {}

Eigen::Vector3d RigidBodyDynamics::computeLinearAcceleration(const VehicleState& state, const Forces& forces) const {
    Eigen::Vector3d acceleration = forces.force / state.mass;
    
    // Add Coriolis effect
    acceleration += computeCoriolisEffect(state);

    // Transform acceleration from body frame to ECEF frame
    Eigen::Matrix3d R = computeRotationMatrix(state.orientation);
    return R * acceleration;
}

Eigen::Vector3d RigidBodyDynamics::computeAngularAcceleration(const VehicleState& state, const Forces& forces) const {
    Eigen::Matrix3d I_inv = computeInertiaMatrixInverse(state.inertia);
    Eigen::Vector3d angular_momentum = state.inertia * state.angular_velocity;
    
    return I_inv * (forces.moment - state.angular_velocity.cross(angular_momentum));
}

void RigidBodyDynamics::integrateMotion(VehicleState& state, const Forces& forces, double dt) const {
    // Compute accelerations
    Eigen::Vector3d linear_acceleration = computeLinearAcceleration(state, forces);
    Eigen::Vector3d angular_acceleration = computeAngularAcceleration(state, forces);

    // Update position and velocity (using semi-implicit Euler integration)
    state.velocity += linear_acceleration * dt;
    state.position += state.velocity * dt;

    // Update orientation and angular velocity
    state.angular_velocity += angular_acceleration * dt;
    Eigen::Quaterniond dq(1, 0.5 * state.angular_velocity.x() * dt,
                             0.5 * state.angular_velocity.y() * dt,
                             0.5 * state.angular_velocity.z() * dt);
    state.orientation = (state.orientation * dq).normalized();
}

void RigidBodyDynamics::updateMassProperties(VehicleState& state, double mass_flow_rate, double dt) const {
    double new_mass = state.mass - mass_flow_rate * dt;
    
    // Simple linear scaling of inertia with mass (assuming uniform density change)
    double mass_ratio = new_mass / state.mass;
    Eigen::Matrix3d new_inertia = state.inertia * mass_ratio;

    state.mass = new_mass;
    state.inertia = new_inertia;
}

Eigen::Matrix3d RigidBodyDynamics::computeInertiaMatrixInverse(const Eigen::Matrix3d& inertia) const {
    return inertia.inverse();
}

Eigen::Vector3d RigidBodyDynamics::computeCoriolisEffect(const VehicleState& state) const {
    Eigen::Vector3d omega_earth(0, 0, EARTH_ANGULAR_VELOCITY);
    return -2 * omega_earth.cross(state.velocity);
}

Eigen::Matrix3d RigidBodyDynamics::computeRotationMatrix(const Eigen::Quaterniond& q) const {
    return q.toRotationMatrix();
}
