#ifndef FLIGHT_DYNAMICS_INTEGRATOR_HPP
#define FLIGHT_DYNAMICS_INTEGRATOR_HPP

#include <Eigen/Dense>
#include "atmospheric_model.hpp"
#include "environmental_effects.hpp"
#include "aerodynamics_model.hpp"
#include "rigid_body_dynamics.hpp"
#include <memory>

class PDESimulator;

class FlightDynamicsIntegrator {
public:
    struct FlightState {
        RigidBodyDynamics::VehicleState vehicle_state;
        AerodynamicsModel::ControlSurfaces control_surfaces;
        Eigen::Vector3d thrust_vector;
        double time;
    };

    struct PDEFeedback {
        double altitude;
        double mach_number;
        double dynamic_pressure;
        Eigen::Vector3d velocity;
        Eigen::Quaterniond orientation;
    };

    FlightDynamicsIntegrator(double initial_mass, const Eigen::Matrix3d& initial_inertia,
                             double reference_area, double reference_length);
    ~FlightDynamicsIntegrator() = default;

    void updateFlightState(double dt);
    PDEFeedback getPDEFeedback() const;
    void setEngineThrust(const Eigen::Vector3d& thrust);
    void runSimulationStep(double dt);

    const FlightState& getFlightState() const { return flight_state; }
    FlightState& getFlightState() { return flight_state; }
    void setControlSurfaces(const AerodynamicsModel::ControlSurfaces& controls) {
        flight_state.control_surfaces = controls;
    }

    void setPDESimulator(std::shared_ptr<PDESimulator> pde_sim) {
        pde_simulator = pde_sim;
    }

private:
    AtmosphericModel atmospheric_model;
    EnvironmentalEffects environmental_effects;
    AerodynamicsModel aerodynamics_model;
    RigidBodyDynamics rigid_body_dynamics;

    double reference_area;
    double reference_length;

    FlightState flight_state;
    std::shared_ptr<PDESimulator> pde_simulator;

    Eigen::Vector3d calculateTotalForce() const;
    Eigen::Vector3d calculateTotalMoment() const;

    static Eigen::Vector3d arrayToVector3d(const std::array<double, 3>& arr);
    static std::array<double, 3> vector3dToArray(const Eigen::Vector3d& vec);
};

class PDESimulator {
public:
    struct PDEState {
        double fuel_mass_flow_rate;
        double oxidizer_mass_flow_rate;
        double chamber_pressure;
        double chamber_temperature;
        double exhaust_velocity;
    };

    virtual void updateConditions(const FlightDynamicsIntegrator::PDEFeedback& feedback) = 0;
    virtual PDEState computeCycle(double dt) = 0;
    virtual ~PDESimulator() = default;
};

#endif // FLIGHT_DYNAMICS_INTEGRATOR_HPP
