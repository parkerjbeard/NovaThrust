#ifndef STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <memory>
#include <random>

namespace tvc {

struct SensorData {
    Eigen::Vector3d acceleration;
    Eigen::Vector3d angularVelocity;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    double altitude;
    double airspeed;
    double timestamp;
};

struct NoiseParameters {
    Eigen::MatrixXd processNoise;
    Eigen::MatrixXd measurementNoise;
};

class SensorModel {
public:
    SensorModel();
    void setNoiseParameters(const NoiseParameters& params);
    SensorData generateSensorData(const Eigen::VectorXd& trueState, double timestamp);
    void simulateSensorFailure(int sensorType, bool failed);

private:
    NoiseParameters m_noiseParams;
    std::default_random_engine m_generator;
    std::normal_distribution<double> m_normalDist;
    bool m_sensorFailure[3] = {false, false, false}; // IMU, GPS, Air data
};

class StateEstimator {
public:
    StateEstimator(int stateDim, int measurementDim);
    void updateEstimate(const SensorData& sensorData);
    Eigen::VectorXd getPredictedState(double predictionTime) const;
    void setProcessNoise(const NoiseParameters& params);
    Eigen::VectorXd getEstimatedState() const;
    Eigen::MatrixXd getEstimationCovariance() const;

private:
    Eigen::VectorXd m_state;
    Eigen::MatrixXd m_covariance;
    Eigen::MatrixXd m_processNoise;
    Eigen::MatrixXd m_measurementNoise;
    std::unique_ptr<SensorModel> m_sensorModel;

    Eigen::VectorXd stateTransitionModel(const Eigen::VectorXd& state, double dt) const;
    Eigen::MatrixXd calculateJacobian(const Eigen::VectorXd& state, double dt) const;
    Eigen::VectorXd measurementModel(const Eigen::VectorXd& state) const;
    void performPredictionStep(double dt);
    void performUpdateStep(const SensorData& measurement);
};

} // namespace tvc

#endif // STATE_ESTIMATOR_HPP
