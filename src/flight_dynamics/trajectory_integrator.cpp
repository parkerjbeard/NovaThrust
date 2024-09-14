#include "flight_dynamics/trajectory_integrator.hpp"
#include <iostream>

TrajectoryIntegrator::TrajectoryIntegrator(const RigidBodyDynamics& rbd, 
                                           const AerodynamicsModel& aero, 
                                           const EnvironmentalEffects& env)
    : rbd(rbd), aero(aero), env(env) {}

std::vector<TrajectoryIntegrator::TrajectoryPoint> TrajectoryIntegrator::integrateFlightPath(
    const RigidBodyDynamics::VehicleState& initial_state,
    const AerodynamicsModel::ControlSurfaces& controls,
    double start_time, double end_time, double dt) {

    std::vector<TrajectoryPoint> trajectory;
    RigidBodyDynamics::VehicleState current_state = initial_state;
    double current_time = start_time;

    while (current_time <= end_time) {
        TrajectoryPoint point;
        point.time = current_time;
        point.position = current_state.position;
        point.velocity = current_state.velocity;
        point.orientation = current_state.orientation;
        point.angular_velocity = current_state.angular_velocity;
        trajectory.push_back(point);

        current_state = integrateState(current_state, controls, dt);
        current_time += dt;
    }

    return trajectory;
}

std::vector<TrajectoryIntegrator::TrajectoryPoint> TrajectoryIntegrator::predictTrajectory(
    const RigidBodyDynamics::VehicleState& current_state,
    const AerodynamicsModel::ControlSurfaces& controls,
    double prediction_time, double dt) {

    return integrateFlightPath(current_state, controls, 0, prediction_time, dt);
}

Eigen::Vector3d TrajectoryIntegrator::transformCoordinates(const Eigen::Vector3d& vec, 
                                                           const Eigen::Quaterniond& orientation, 
                                                           bool body_to_earth) const {
    if (body_to_earth) {
        return orientation * vec;
    } else {
        return orientation.inverse() * vec;
    }
}

RigidBodyDynamics::VehicleState TrajectoryIntegrator::integrateState(
    const RigidBodyDynamics::VehicleState& state,
    const AerodynamicsModel::ControlSurfaces& controls,
    double dt) {

    RigidBodyDynamics::Forces forces;
    forces.force = computeTotalForce(state, controls, 0);
    forces.moment = computeTotalMoment(state, controls, 0);

    RigidBodyDynamics::VehicleState new_state = state;
    rbd.integrateMotion(new_state, forces, dt);

    // Update mass properties (assuming a constant mass flow rate for simplicity)
    double mass_flow_rate = 1.0;  // kg/s, replace with actual mass flow rate calculation
    rbd.updateMassProperties(new_state, mass_flow_rate, dt);

    return new_state;
}

Eigen::Vector3d TrajectoryIntegrator::computeTotalForce(
    const RigidBodyDynamics::VehicleState& state,
    const AerodynamicsModel::ControlSurfaces& controls,
    double time) {

    // Compute aerodynamic forces
    AerodynamicsModel::VehicleState aero_state;
    aero_state.position = {state.position.x(), state.position.y(), state.position.z()};
    aero_state.velocity = {state.velocity.x(), state.velocity.y(), state.velocity.z()};
    Eigen::Vector3d euler_angles = state.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    aero_state.attitude = {euler_angles.x(), euler_angles.y(), euler_angles.z()};
    aero_state.mass = state.mass;
    aero_state.reference_area = 10.0;  // Replace with actual reference area
    aero_state.reference_length = 5.0;  // Replace with actual reference length

    AerodynamicsModel::AeroForces aero_forces = aero.computeAeroForces(aero_state, controls);

    // Compute gravitational force
    Eigen::Vector3d gravity = env.applyGravityModel(state.position);

    // Compute thrust (replace with actual thrust model)
    Eigen::Vector3d thrust(10000, 0, 0);  // Assuming constant thrust along body x-axis

    // Transform forces to body frame
    Eigen::Vector3d aero_force_body = transformCoordinates(Eigen::Vector3d(aero_forces.force[0], aero_forces.force[1], aero_forces.force[2]), state.orientation, false);
    Eigen::Vector3d gravity_body = transformCoordinates(gravity, state.orientation, false);

    // Sum all forces in body frame
    return aero_force_body + gravity_body + thrust;
}

Eigen::Vector3d TrajectoryIntegrator::computeTotalMoment(
    const RigidBodyDynamics::VehicleState& state,
    const AerodynamicsModel::ControlSurfaces& controls,
    double time) {

    // Compute aerodynamic moments
    AerodynamicsModel::VehicleState aero_state;
    aero_state.position = {state.position.x(), state.position.y(), state.position.z()};
    aero_state.velocity = {state.velocity.x(), state.velocity.y(), state.velocity.z()};
    Eigen::Vector3d euler_angles = state.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    aero_state.attitude = {euler_angles.x(), euler_angles.y(), euler_angles.z()};
    aero_state.mass = state.mass;
    aero_state.reference_area = 10.0;  // Replace with actual reference area
    aero_state.reference_length = 5.0;  // Replace with actual reference length

    AerodynamicsModel::AeroForces aero_forces = aero.computeAeroMoments(aero_state, controls);

    // Sum all moments (assuming no additional moments from thrust or other sources)
    return Eigen::Vector3d(aero_forces.moment[0], aero_forces.moment[1], aero_forces.moment[2]);
}