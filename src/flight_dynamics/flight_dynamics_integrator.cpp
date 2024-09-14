#include "flight_dynamics/flight_dynamics_integrator.hpp"

FlightDynamicsIntegrator::FlightDynamicsIntegrator(double initial_mass, const Eigen::Matrix3d& initial_inertia,
                                                   double reference_area, double reference_length)
    : atmospheric_model(),
      environmental_effects(atmospheric_model),
      aerodynamics_model(atmospheric_model),
      rigid_body_dynamics(initial_mass, initial_inertia),
      reference_area(reference_area),
      reference_length(reference_length) {
    
    flight_state.vehicle_state.mass = initial_mass;
    flight_state.vehicle_state.inertia = initial_inertia;
    flight_state.vehicle_state.position = Eigen::Vector3d::Zero();
    flight_state.vehicle_state.velocity = Eigen::Vector3d::Zero();
    flight_state.vehicle_state.orientation = Eigen::Quaterniond::Identity();
    flight_state.vehicle_state.angular_velocity = Eigen::Vector3d::Zero();
    flight_state.control_surfaces = {0, 0, 0};
    flight_state.thrust_vector = Eigen::Vector3d::Zero();
    flight_state.time = 0.0;
}

void FlightDynamicsIntegrator::updateFlightState(double dt) {
    Eigen::Vector3d total_force = calculateTotalForce();
    Eigen::Vector3d total_moment = calculateTotalMoment();

    RigidBodyDynamics::Forces forces;
    forces.force = total_force;
    forces.moment = total_moment;

    rigid_body_dynamics.integrateMotion(flight_state.vehicle_state, forces, dt);

    if (pde_simulator) {
        PDESimulator::PDEState pde_state = pde_simulator->computeCycle(dt);
        double mass_flow_rate = pde_state.fuel_mass_flow_rate + pde_state.oxidizer_mass_flow_rate;
        rigid_body_dynamics.updateMassProperties(flight_state.vehicle_state, mass_flow_rate, dt);
    }

    flight_state.time += dt;
}

FlightDynamicsIntegrator::PDEFeedback FlightDynamicsIntegrator::getPDEFeedback() const {
    PDEFeedback feedback;

    double earth_radius = 6371000.0;  // meters
    feedback.altitude = flight_state.vehicle_state.position.norm() - earth_radius;

    double temperature = atmospheric_model.getAtmosphericProperties(feedback.altitude).temperature;
    double speed_of_sound = atmospheric_model.getSpeedOfSound(temperature);
    feedback.mach_number = flight_state.vehicle_state.velocity.norm() / speed_of_sound;

    double density = atmospheric_model.getAtmosphericProperties(feedback.altitude).density;
    feedback.dynamic_pressure = 0.5 * density * flight_state.vehicle_state.velocity.squaredNorm();

    feedback.velocity = flight_state.vehicle_state.velocity;
    feedback.orientation = flight_state.vehicle_state.orientation;

    return feedback;
}

void FlightDynamicsIntegrator::setEngineThrust(const Eigen::Vector3d& thrust) {
    flight_state.thrust_vector = thrust;
}

void FlightDynamicsIntegrator::runSimulationStep(double dt) {
    updateFlightState(dt);
    
    if (pde_simulator) {
        PDEFeedback feedback = getPDEFeedback();
        pde_simulator->updateConditions(feedback);
    }
}

Eigen::Vector3d FlightDynamicsIntegrator::calculateTotalForce() const {
    AerodynamicsModel::VehicleState aero_state;
    aero_state.position = flight_state.vehicle_state.position;
    aero_state.velocity = flight_state.vehicle_state.velocity;
    aero_state.attitude = flight_state.vehicle_state.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    aero_state.mass = flight_state.vehicle_state.mass;
    aero_state.reference_area = reference_area;
    aero_state.reference_length = reference_length;

    AerodynamicsModel::AeroForces aero_forces = aerodynamics_model.computeAeroForces(aero_state, flight_state.control_surfaces);

    Eigen::Vector3d gravity_force = environmental_effects.applyGravityModel(flight_state.vehicle_state.position);

    EnvironmentalEffects::VehicleState env_state;
    env_state.position = flight_state.vehicle_state.position;
    env_state.velocity = flight_state.vehicle_state.velocity;
    env_state.mass = flight_state.vehicle_state.mass;
    env_state.area = aero_state.reference_area;
    Eigen::Vector3d env_force = environmental_effects.computeSolarEffects(env_state, flight_state.time);

    return aero_forces.force + gravity_force + env_force + flight_state.thrust_vector;
}

Eigen::Vector3d FlightDynamicsIntegrator::calculateTotalMoment() const {
    AerodynamicsModel::VehicleState aero_state;
    aero_state.position = flight_state.vehicle_state.position;
    aero_state.velocity = flight_state.vehicle_state.velocity;
    aero_state.attitude = flight_state.vehicle_state.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    aero_state.mass = flight_state.vehicle_state.mass;
    aero_state.reference_area = reference_area;
    aero_state.reference_length = reference_length;

    AerodynamicsModel::AeroForces aero_forces = aerodynamics_model.computeAeroMoments(aero_state, flight_state.control_surfaces);

    return aero_forces.moment;
}
