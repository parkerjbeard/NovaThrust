#include "tvc/tvc_system.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include "multi_tube_pde/multi_tube_configuration.hpp"

TVCSystem::TVCSystem(const TVCParameters& params)
    : m_params(params),
      m_pitch_controller(std::make_unique<PIDController>(2.0, 0.1, 0.5, -params.max_gimbal_rate, params.max_gimbal_rate)),
      m_yaw_controller(std::make_unique<PIDController>(2.0, 0.1, 0.5, -params.max_gimbal_rate, params.max_gimbal_rate)),
      m_state_estimator(std::make_unique<KalmanFilter>(
          Eigen::MatrixXd::Identity(12, 12),  // State transition matrix (position, velocity, acceleration, angular velocity)
          Eigen::MatrixXd::Identity(6, 12),   // Observation matrix (we only measure position and velocity)
          Eigen::MatrixXd::Identity(12, 12) * 0.01,  // Process noise covariance
          Eigen::MatrixXd::Identity(6, 6) * 0.1      // Measurement noise covariance
      ))
{
    m_current_state.thrust_vector = Eigen::Vector3d::Zero();
    m_current_state.gimbal_angle_pitch = 0.0;
    m_current_state.gimbal_angle_yaw = 0.0;
}

TVCSystem::~TVCSystem() = default;

void TVCSystem::initialize(const FlightDynamicsIntegrator::FlightState& initial_state) {
    m_current_state.thrust_vector = initial_state.thrust_vector;
    m_current_state.gimbal_angle_pitch = 0.0;
    m_current_state.gimbal_angle_yaw = 0.0;
    
    Eigen::VectorXd initial_x(12);
    initial_x << initial_state.vehicle_state.position, 
                 initial_state.vehicle_state.velocity, 
                 Eigen::Vector3d::Zero(),  // Initial acceleration
                 initial_state.vehicle_state.angular_velocity;
    m_state_estimator->update(initial_x.head<6>());
}

void TVCSystem::update(const FlightDynamicsIntegrator::FlightState& current_state,
                       const Eigen::Vector3d& desired_acceleration) {
    updateStateEstimator(current_state);
    
    Eigen::Vector3d desired_thrust_vector = calculateDesiredThrustVector(current_state, desired_acceleration);
    updateGimbalAngles(desired_thrust_vector);
    optimizePulseSequence(desired_thrust_vector);
    applyGimbalLimits();
    
    updatePerformanceMetrics(current_state);
}

TVCSystem::TVCState TVCSystem::getTVCState() const {
    return m_current_state;
}

void TVCSystem::setMultiTubeConfiguration(std::shared_ptr<tvc::MultiTubePDEConfiguration> multi_tube_config) {
    m_multi_tube_config = std::move(multi_tube_config);
}

void TVCSystem::setPerformanceMetricsCalculator(std::shared_ptr<PerformanceMetricsCalculator> perf_metrics_calc) {
    m_perf_metrics_calc = std::move(perf_metrics_calc);
}

Eigen::Vector3d TVCSystem::calculateDesiredThrustVector(const FlightDynamicsIntegrator::FlightState& current_state,
                                                        const Eigen::Vector3d& desired_acceleration) {
    double mass = current_state.vehicle_state.mass;
    Eigen::Vector3d gravity(0, 0, -9.81);  // Assuming z-axis is up
    Eigen::Vector3d required_force = mass * (desired_acceleration - gravity);
    
    Eigen::Vector3d desired_thrust = required_force;
    double thrust_magnitude = desired_thrust.norm();
    
    thrust_magnitude = std::clamp(thrust_magnitude, m_params.min_thrust, m_params.max_thrust);
    
    if (thrust_magnitude > 0) {
        desired_thrust.normalize();
        desired_thrust *= thrust_magnitude;
    } else {
        desired_thrust.setZero();
    }
    
    return desired_thrust;
}

void TVCSystem::updateGimbalAngles(const Eigen::Vector3d& desired_thrust_vector) {
    Eigen::Vector3d current_thrust = m_current_state.thrust_vector;
    
    double desired_pitch = std::atan2(desired_thrust_vector.y(), desired_thrust_vector.x());
    double desired_yaw = std::atan2(std::sqrt(desired_thrust_vector.x()*desired_thrust_vector.x() + 
                                              desired_thrust_vector.y()*desired_thrust_vector.y()),
                                    desired_thrust_vector.z());
    
    double current_pitch = std::atan2(current_thrust.y(), current_thrust.x());
    double current_yaw = std::atan2(std::sqrt(current_thrust.x()*current_thrust.x() + 
                                              current_thrust.y()*current_thrust.y()),
                                    current_thrust.z());
    
    double dt = 1.0 / m_params.control_frequency;
    double pitch_rate = m_pitch_controller->update(desired_pitch, current_pitch, dt);
    double yaw_rate = m_yaw_controller->update(desired_yaw, current_yaw, dt);
    
    m_current_state.gimbal_angle_pitch += pitch_rate * dt;
    m_current_state.gimbal_angle_yaw += yaw_rate * dt;
    
    updateThrustVector();
}

void TVCSystem::updateThrustVector() {
    double thrust_magnitude = m_current_state.thrust_vector.norm();
    double pitch = m_current_state.gimbal_angle_pitch;
    double yaw = m_current_state.gimbal_angle_yaw;
    
    m_current_state.thrust_vector = Eigen::Vector3d(
        thrust_magnitude * std::cos(yaw) * std::cos(pitch),
        thrust_magnitude * std::cos(yaw) * std::sin(pitch),
        thrust_magnitude * std::sin(yaw)
    );
}

void TVCSystem::optimizePulseSequence(const Eigen::Vector3d& desired_thrust_vector) {
    if (m_multi_tube_config) {
        ThrustVector thrust_vector;
        thrust_vector.x = desired_thrust_vector.x();
        thrust_vector.y = desired_thrust_vector.y();
        thrust_vector.z = desired_thrust_vector.z();
        
        m_current_state.pulse_sequence = m_multi_tube_config->optimizeFiringSequence(thrust_vector);
    } else {
        throw std::runtime_error("MultiTubePDEConfiguration not set in TVCSystem");
    }
}

void TVCSystem::applyGimbalLimits() {
    m_current_state.gimbal_angle_pitch = std::clamp(m_current_state.gimbal_angle_pitch, 
                                                    -m_params.max_gimbal_angle, 
                                                    m_params.max_gimbal_angle);
    m_current_state.gimbal_angle_yaw = std::clamp(m_current_state.gimbal_angle_yaw, 
                                                  -m_params.max_gimbal_angle, 
                                                  m_params.max_gimbal_angle);
}

void TVCSystem::updateStateEstimator(const FlightDynamicsIntegrator::FlightState& current_state) {
    m_state_estimator->predict();
    
    Eigen::VectorXd measurement(6);
    measurement << current_state.vehicle_state.position, current_state.vehicle_state.velocity;
    m_state_estimator->update(measurement);
}

void TVCSystem::updatePerformanceMetrics(const FlightDynamicsIntegrator::FlightState& current_state) {
    if (m_perf_metrics_calc && m_multi_tube_config) {
        // Get the states of all tubes
        std::vector<PDETubeState> tube_states = m_multi_tube_config->getTubeStates();
        
        // Calculate total exit area
        double total_exit_area = 0.0;
        for (const auto& state : tube_states) {
            total_exit_area += state.exit_area;
        }
        
        // Calculate ambient pressure
        double ambient_pressure = calculateAmbientPressure(current_state.vehicle_state.position[2]);
        
        // Calculate total mass flow rate and average detonation frequency
        double total_mass_flow_rate = 0.0;
        double avg_detonation_frequency = 0.0;
        for (const auto& state : tube_states) {
            total_mass_flow_rate += state.mass_flow_rate;
            avg_detonation_frequency += state.detonation_frequency;
        }
        if (!tube_states.empty()) {
            avg_detonation_frequency /= tube_states.size();
        }
        
        // Calculate exhaust velocity
        double exhaust_velocity = 0.0;
        if (total_mass_flow_rate > 0) {
            exhaust_velocity = m_current_state.thrust_vector.norm() / total_mass_flow_rate;
        }
        
        // Use the first tube's state for chamber and exit pressure (assuming they're similar for all tubes)
        double chamber_pressure = tube_states.empty() ? 101325.0 : tube_states[0].chamber_pressure;
        double exit_pressure = tube_states.empty() ? 101325.0 : tube_states[0].pressure; // Assuming exit pressure equals tube pressure post-detonation
        
        // Calculate nozzle expansion ratio (assuming all tubes have the same ratio)
        double nozzle_expansion_ratio = tube_states.empty() ? 1.0 : (tube_states[0].exit_area / tube_states[0].throat_area);

        m_perf_metrics_calc->setThrustParameters(
            total_exit_area,
            ambient_pressure,
            total_mass_flow_rate,
            exhaust_velocity,
            avg_detonation_frequency,
            chamber_pressure,
            exit_pressure,
            nozzle_expansion_ratio
        );
    }
}

double TVCSystem::calculateAmbientPressure(double altitude) {
    // Simple atmospheric model (exponential atmosphere)
    const double p0 = 101325;  // Sea level standard atmospheric pressure in Pa
    const double h0 = 7400;    // Scale height in m
    return p0 * std::exp(-altitude / h0);
}

// PIDController implementation
PIDController::PIDController(double kp, double ki, double kd, double min_output, double max_output)
    : m_kp(kp), m_ki(ki), m_kd(kd), m_min_output(min_output), m_max_output(max_output),
      m_integral(0.0), m_previous_error(0.0) {}

double PIDController::update(double setpoint, double measured_value, double dt) {
    double error = setpoint - measured_value;
    m_integral += error * dt;
    double derivative = (error - m_previous_error) / dt;
    
    double output = m_kp * error + m_ki * m_integral + m_kd * derivative;
    output = std::clamp(output, m_min_output, m_max_output);
    
    m_previous_error = error;
    return output;
}

void PIDController::reset() {
    m_integral = 0.0;
    m_previous_error = 0.0;
}

// KalmanFilter implementation
KalmanFilter::KalmanFilter(const Eigen::MatrixXd& F, const Eigen::MatrixXd& H, 
                           const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
    : m_F(F), m_H(H), m_Q(Q), m_R(R) {
    int n = F.rows();
    m_x = Eigen::VectorXd::Zero(n);
    m_P = Eigen::MatrixXd::Identity(n, n);
}

void KalmanFilter::predict() {
    m_x = m_F * m_x;
    m_P = m_F * m_P * m_F.transpose() + m_Q;
}

void KalmanFilter::update(const Eigen::VectorXd& measurement) {
    Eigen::VectorXd y = measurement - m_H * m_x;
    Eigen::MatrixXd S = m_H * m_P * m_H.transpose() + m_R;
    Eigen::MatrixXd K = m_P * m_H.transpose() * S.inverse();
    
    m_x = m_x + K * y;
    m_P = (Eigen::MatrixXd::Identity(m_x.size(), m_x.size()) - K * m_H) * m_P;
}

Eigen::VectorXd KalmanFilter::getState() const {
    return m_x;
}