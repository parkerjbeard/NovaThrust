#include "tvc/state_estimator.hpp"
#include <cmath>

namespace tvc {

SensorModel::SensorModel() : m_generator(), m_normalDist(0.0, 1.0) {}

void SensorModel::setNoiseParameters(const NoiseParameters& params) {
    m_noiseParams = params;
}

SensorData SensorModel::generateSensorData(const Eigen::VectorXd& trueState, double timestamp) {
    SensorData data;
    data.timestamp = timestamp;

    // Add noise to sensor readings
    auto addNoise = [this](double value, double noiseSigma) {
        return value + m_normalDist(m_generator) * noiseSigma;
    };

    if (!m_sensorFailure[0]) { // IMU
        data.acceleration = trueState.segment<3>(3) + Eigen::Vector3d::Random() * 0.1;
        data.angularVelocity = trueState.segment<3>(9) + Eigen::Vector3d::Random() * 0.01;
    }

    if (!m_sensorFailure[1]) { // GPS
        data.position = trueState.segment<3>(0) + Eigen::Vector3d::Random() * 5.0;
        data.velocity = trueState.segment<3>(3) + Eigen::Vector3d::Random() * 0.1;
    }

    if (!m_sensorFailure[2]) { // Air data
        data.altitude = addNoise(trueState(2), 1.0);
        data.airspeed = addNoise(trueState.segment<3>(3).norm(), 0.5);
    }

    return data;
}

void SensorModel::simulateSensorFailure(int sensorType, bool failed) {
    if (sensorType >= 0 && sensorType < 3) {
        m_sensorFailure[sensorType] = failed;
    }
}

StateEstimator::StateEstimator(int stateDim, int measurementDim)
    : m_state(Eigen::VectorXd::Zero(stateDim)),
      m_covariance(Eigen::MatrixXd::Identity(stateDim, stateDim)),
      m_processNoise(Eigen::MatrixXd::Identity(stateDim, stateDim)),
      m_measurementNoise(Eigen::MatrixXd::Identity(measurementDim, measurementDim)),
      m_sensorModel(std::make_unique<SensorModel>()) {}

void StateEstimator::updateEstimate(const SensorData& sensorData) {
    static double lastTimestamp = sensorData.timestamp;
    double dt = sensorData.timestamp - lastTimestamp;
    lastTimestamp = sensorData.timestamp;

    performPredictionStep(dt);
    performUpdateStep(sensorData);
}

Eigen::VectorXd StateEstimator::getPredictedState(double predictionTime) const {
    return stateTransitionModel(m_state, predictionTime);
}

void StateEstimator::setProcessNoise(const NoiseParameters& params) {
    m_processNoise = params.processNoise;
    m_measurementNoise = params.measurementNoise;
    m_sensorModel->setNoiseParameters(params);
}

Eigen::VectorXd StateEstimator::getEstimatedState() const {
    return m_state;
}

Eigen::MatrixXd StateEstimator::getEstimationCovariance() const {
    return m_covariance;
}

Eigen::VectorXd StateEstimator::stateTransitionModel(const Eigen::VectorXd& state, double dt) const {
    Eigen::VectorXd newState = state;
    newState.segment<3>(0) += state.segment<3>(3) * dt; // position update
    newState.segment<3>(3) += state.segment<3>(6) * dt; // velocity update
    // Implement quaternion integration for orientation update
    // Add other state transition logic as needed
    return newState;
}

Eigen::MatrixXd StateEstimator::calculateJacobian(const Eigen::VectorXd& state, double dt) const {
    int stateDim = state.size();
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(stateDim, stateDim);
    jacobian.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    jacobian.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
    // Add other Jacobian elements as needed
    return jacobian;
}

Eigen::VectorXd StateEstimator::measurementModel(const Eigen::VectorXd& state) const {
    // Implement measurement model based on your sensor configuration
    Eigen::VectorXd measurement(9); // Example: 3 for position, 3 for velocity, 3 for orientation
    measurement.segment<3>(0) = state.segment<3>(0); // position
    measurement.segment<3>(3) = state.segment<3>(3); // velocity
    measurement.segment<3>(6) = state.segment<3>(9); // orientation (assuming Euler angles)
    return measurement;
}

void StateEstimator::performPredictionStep(double dt) {
    // Predict state
    m_state = stateTransitionModel(m_state, dt);

    // Predict covariance
    Eigen::MatrixXd F = calculateJacobian(m_state, dt);
    m_covariance = F * m_covariance * F.transpose() + m_processNoise;
}

void StateEstimator::performUpdateStep(const SensorData& measurement) {
    Eigen::VectorXd predictedMeasurement = measurementModel(m_state);
    Eigen::VectorXd measurementResidual = measurement.position.homogeneous() - predictedMeasurement;

    Eigen::MatrixXd H = calculateJacobian(m_state, 0); // Measurement Jacobian
    Eigen::MatrixXd S = H * m_covariance * H.transpose() + m_measurementNoise;
    Eigen::MatrixXd K = m_covariance * H.transpose() * S.inverse();

    m_state += K * measurementResidual;
    m_covariance = (Eigen::MatrixXd::Identity(m_state.size(), m_state.size()) - K * H) * m_covariance;
}

} // namespace tvc
