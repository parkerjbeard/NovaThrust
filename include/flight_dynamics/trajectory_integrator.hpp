#ifndef TRAJECTORY_INTEGRATOR_HPP
#define TRAJECTORY_INTEGRATOR_HPP

#include <vector>
#include <Eigen/Dense>
#include "rigid_body_dynamics.hpp"
#include "aerodynamics_model.hpp"
#include "environmental_effects.hpp"

class TrajectoryIntegrator {
public:
    struct TrajectoryPoint {
        double time;
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Quaterniond orientation;
        Eigen::Vector3d angular_velocity;
    };

    TrajectoryIntegrator(const RigidBodyDynamics& rbd, 
                         const AerodynamicsModel& aero, 
                         const EnvironmentalEffects& env);
    ~TrajectoryIntegrator() = default;

    std::vector<TrajectoryPoint> integrateFlightPath(const RigidBodyDynamics::VehicleState& initial_state,
                                                     const AerodynamicsModel::ControlSurfaces& controls,
                                                     double start_time, double end_time, double dt);

    std::vector<TrajectoryPoint> predictTrajectory(const RigidBodyDynamics::VehicleState& current_state,
                                                   const AerodynamicsModel::ControlSurfaces& controls,
                                                   double prediction_time, double dt);

    Eigen::Vector3d transformCoordinates(const Eigen::Vector3d& vec, 
                                         const Eigen::Quaterniond& orientation, 
                                         bool body_to_earth) const;

private:
    const RigidBodyDynamics& rbd;
    const AerodynamicsModel& aero;
    const EnvironmentalEffects& env;

    RigidBodyDynamics::VehicleState integrateState(const RigidBodyDynamics::VehicleState& state,
                                                   const AerodynamicsModel::ControlSurfaces& controls,
                                                   double dt);

    Eigen::Vector3d computeTotalForce(const RigidBodyDynamics::VehicleState& state,
                                      const AerodynamicsModel::ControlSurfaces& controls,
                                      double time);

    Eigen::Vector3d computeTotalMoment(const RigidBodyDynamics::VehicleState& state,
                                       const AerodynamicsModel::ControlSurfaces& controls,
                                       double time);
};

#endif // TRAJECTORY_INTEGRATOR_HPP
