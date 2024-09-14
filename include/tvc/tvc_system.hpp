#ifndef TVC_SYSTEM_HPP
#define TVC_SYSTEM_HPP

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include "flight_dynamics/flight_dynamics_integrator.hpp"
#include "multi_tube_pde/multi_tube_configuration.hpp"
#include "performance_metrics/performance_metrics_calculator.hpp"

class PIDController;
class KalmanFilter;

class TVCSystem {
public:
    struct TVCState {
        Eigen::Vector3d thrust_vector;
        std::vector<int> pulse_sequence;
        double gimbal_angle_pitch;
        double gimbal_angle_yaw;
    };

    struct TVCParameters {
        double max_gimbal_angle;
        double max_gimbal_rate;
        double control_frequency;
        double min_thrust;
        double max_thrust;
    };

    TVCSystem(const TVCParameters& params);
    ~TVCSystem();

    void initialize(const FlightDynamicsIntegrator::FlightState& initial_state);
    void update(const FlightDynamicsIntegrator::FlightState& current_state,
                const Eigen::Vector3d& desired_acceleration);

    TVCState getTVCState() const;
    void setMultiTubeConfiguration(std::shared_ptr<tvc::MultiTubePDEConfiguration> multi_tube_config);
    void setPerformanceMetricsCalculator(std::shared_ptr<PerformanceMetricsCalculator> perf_metrics_calc);

private:
    TVCParameters m_params;
    TVCState m_current_state;
    std::unique_ptr<PIDController> m_pitch_controller;
    std::unique_ptr<PIDController> m_yaw_controller;
    std::unique_ptr<KalmanFilter> m_state_estimator;
    std::shared_ptr<tvc::MultiTubePDEConfiguration> m_multi_tube_config;
    std::shared_ptr<PerformanceMetricsCalculator> m_perf_metrics_calc;

    Eigen::Vector3d calculateDesiredThrustVector(const FlightDynamicsIntegrator::FlightState& current_state,
                                                 const Eigen::Vector3d& desired_acceleration);
    void updateGimbalAngles(const Eigen::Vector3d& desired_thrust_vector);
    void optimizePulseSequence(const Eigen::Vector3d& desired_thrust_vector);
    void applyGimbalLimits();
    void updateStateEstimator(const FlightDynamicsIntegrator::FlightState& current_state);
    void updateThrustVector();
    void updatePerformanceMetrics(const FlightDynamicsIntegrator::FlightState& current_state);
    double calculateAmbientPressure(double altitude);
};

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double min_output, double max_output);
    double update(double setpoint, double measured_value, double dt);
    void reset();

private:
    double m_kp, m_ki, m_kd;
    double m_min_output, m_max_output;
    double m_integral, m_previous_error;
};

class KalmanFilter {
public:
    KalmanFilter(const Eigen::MatrixXd& F, const Eigen::MatrixXd& H, 
                 const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
    void predict();
    void update(const Eigen::VectorXd& measurement);
    Eigen::VectorXd getState() const;

private:
    Eigen::MatrixXd m_F, m_H, m_Q, m_R;
    Eigen::MatrixXd m_P;
    Eigen::VectorXd m_x;
};

#endif // TVC_SYSTEM_HPP