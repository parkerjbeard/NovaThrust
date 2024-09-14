#include "tvc/control_law.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <numeric>

namespace tvc {

// NMPCController implementation

NMPCController::NMPCController(int horizonLength, double controlInterval)
    : m_horizonLength(horizonLength), m_controlInterval(controlInterval) {
    // Initialize model parameters with a discrete-time linear model
    // Example: state transition matrix (A) and control matrix (B)
    // Assuming a simple double integrator model for demonstration
    double dt = m_controlInterval;
    m_A = Eigen::MatrixXd::Identity(6, 6);
    // Position update
    m_A.block<3,3>(0,3) = Eigen::MatrixXd::Identity(3,3) * dt;
    // Control matrix
    m_B = Eigen::MatrixXd::Zero(6, 4);
    m_B.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3) * 0.5 * std::pow(dt, 2);
    m_B.block<3,1>(3,3) = Eigen::MatrixXd::Identity(3,1) * dt;
    
    // Initialize cost matrices
    m_Q = Eigen::MatrixXd::Identity(6, 6) * 10.0; // State cost
    m_R = Eigen::MatrixXd::Identity(4, 4) * 1.0;  // Control effort cost
}

ControlCommand NMPCController::computeControlCommand(const Eigen::VectorXd& state, const Trajectory& desiredTrajectory) {
    std::vector<ControlCommand> optimalSequence = optimizeControlSequence(state, desiredTrajectory);
    return optimalSequence.front(); // Return the first control command in the sequence
}

void NMPCController::updateModelParameters(const SystemIdentificationData& data) {
    // Perform system identification using least squares estimation
    // Assuming data.inputData contains control inputs and data.outputData contains state derivatives

    if (data.inputData.cols() == 0 || data.outputData.cols() == 0) {
        throw std::invalid_argument("SystemIdentificationData contains no data.");
    }

    // Estimate A and B matrices for the linear model: dx/dt = A * x + B * u
    // Using least squares: [A B] = Y * [X; U]^T * ( [X; U] * [X; U]^T )^{-1}

    Eigen::MatrixXd X = data.inputData.leftCols(data.inputData.cols() - 1); // Current states
    Eigen::MatrixXd U = data.inputData.rightCols(4); // Current control inputs
    Eigen::MatrixXd Y = data.outputData.leftCols(data.outputData.cols() - 1); // Next states

    Eigen::MatrixXd Xu(X.rows(), X.cols() + U.cols());
    Xu << X, U;

    Eigen::MatrixXd Xu_pinv = Xu.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd AB = Y * Xu_pinv;

    m_A = AB.leftCols(6);
    m_B = AB.rightCols(4);
}

Eigen::VectorXd NMPCController::predictState(const Eigen::VectorXd& initialState, const std::vector<ControlCommand>& controlSequence) const {
    Eigen::VectorXd currentState = initialState;
    for (const auto& command : controlSequence) {
        Eigen::VectorXd controlInput(4);
        controlInput.head<3>() = command.thrustVector;
        controlInput(3) = command.thrustMagnitude;
        currentState = m_A * currentState + m_B * controlInput;
    }
    return currentState;
}

double NMPCController::computeCost(const std::vector<Eigen::VectorXd>& stateSequence, const std::vector<ControlCommand>& controlSequence, const Trajectory& desiredTrajectory) const {
    double cost = 0.0;
    for (size_t i = 0; i < stateSequence.size(); ++i) {
        // Desired state interpolation
        Eigen::VectorXd desiredState = interpolateDesiredState(desiredTrajectory, i * m_controlInterval);
        Eigen::VectorXd stateError = stateSequence[i] - desiredState;
        cost += stateError.transpose() * m_Q * stateError;

        if (i < controlSequence.size()) {
            Eigen::VectorXd controlError(4);
            controlError.head<3>() = controlSequence[i].thrustVector;
            controlError(3) = controlSequence[i].thrustMagnitude;
            cost += controlError.transpose() * m_R * controlError;
        }
    }
    return cost;
}

std::vector<ControlCommand> NMPCController::optimizeControlSequence(const Eigen::VectorXd& initialState, const Trajectory& desiredTrajectory) {
    // Use Sequential Quadratic Programming (SQP) for optimization
    // For simplicity, we'll implement a basic gradient-based optimizer

    std::vector<ControlCommand> controlSequence(m_horizonLength, ControlCommand());
    double learningRate = 0.01;
    int maxIterations = 200;
    double tolerance = 1e-3;

    for (int iter = 0; iter < maxIterations; ++iter) {
        // Forward simulation
        std::vector<Eigen::VectorXd> stateSequence;
        stateSequence.reserve(m_horizonLength + 1);
        stateSequence.push_back(initialState);
        for (const auto& control : controlSequence) {
            Eigen::VectorXd controlInput(4);
            controlInput.head<3>() = control.thrustVector;
            controlInput(3) = control.thrustMagnitude;
            Eigen::VectorXd nextState = m_A * stateSequence.back() + m_B * controlInput;
            stateSequence.push_back(nextState);
        }

        double cost = computeCost(stateSequence, controlSequence, desiredTrajectory);

        // Compute gradients using finite differences
        std::vector<Eigen::Vector4d> gradients(m_horizonLength, Eigen::Vector4d::Zero());

        double epsilon = 1e-5;

        for (int i = 0; i < m_horizonLength; ++i) {
            for (int j = 0; j < 4; ++j) {
                // Perturb control input
                std::vector<ControlCommand> perturbedSequence = controlSequence;
                if (j < 3) {
                    perturbedSequence[i].thrustVector[j] += epsilon;
                } else {
                    perturbedSequence[i].thrustMagnitude += epsilon;
                }

                // Forward simulation with perturbed controls
                std::vector<Eigen::VectorXd> perturbedStateSequence;
                perturbedStateSequence.reserve(m_horizonLength + 1);
                perturbedStateSequence.push_back(initialState);
                for (const auto& control : perturbedSequence) {
                    Eigen::VectorXd controlInput(4);
                    controlInput.head<3>() = control.thrustVector;
                    controlInput(3) = control.thrustMagnitude;
                    Eigen::VectorXd nextState = m_A * perturbedStateSequence.back() + m_B * controlInput;
                    perturbedStateSequence.push_back(nextState);
                }

                double perturbedCost = computeCost(perturbedStateSequence, perturbedSequence, desiredTrajectory);
                gradients[i][j] = (perturbedCost - cost) / epsilon;
            }
        }

        // Update control sequence
        double maxGradient = 0.0;
        for (int i = 0; i < m_horizonLength; ++i) {
            controlSequence[i].thrustVector -= learningRate * gradients[i].head<3>();
            controlSequence[i].thrustMagnitude -= learningRate * gradients[i][3];
            maxGradient = std::max(maxGradient, gradients[i].head<3>().cwiseAbs().maxCoeff());
            maxGradient = std::max(maxGradient, std::abs(gradients[i][3]));
        }

        if (maxGradient < tolerance) {
            break;
        }
    }

    return controlSequence;
}

Eigen::VectorXd NMPCController::interpolateDesiredState(const Trajectory& desiredTrajectory, double time) const {
    // Simple linear interpolation for desired state
    if (desiredTrajectory.timestamps.empty()) {
        throw std::invalid_argument("Desired trajectory is empty.");
    }

    if (time <= desiredTrajectory.timestamps.front()) {
        Eigen::VectorXd desiredState(6);
        desiredState.head<3>() = desiredTrajectory.positions.front();
        desiredState.tail<3>() = desiredTrajectory.velocities.front();
        return desiredState;
    }

    if (time >= desiredTrajectory.timestamps.back()) {
        Eigen::VectorXd desiredState(6);
        desiredState.head<3>() = desiredTrajectory.positions.back();
        desiredState.tail<3>() = desiredTrajectory.velocities.back();
        return desiredState;
    }

    // Find the interval
    auto it = std::lower_bound(desiredTrajectory.timestamps.begin(), desiredTrajectory.timestamps.end(), time);
    int idx = std::distance(desiredTrajectory.timestamps.begin(), it) - 1;

    double t1 = desiredTrajectory.timestamps[idx];
    double t2 = desiredTrajectory.timestamps[idx + 1];
    double alpha = (time - t1) / (t2 - t1);

    Eigen::VectorXd desiredState(6);
    desiredState.head<3>() = (1 - alpha) * desiredTrajectory.positions[idx] + alpha * desiredTrajectory.positions[idx + 1];
    desiredState.tail<3>() = (1 - alpha) * desiredTrajectory.velocities[idx] + alpha * desiredTrajectory.velocities[idx + 1];
    return desiredState;
}

// AdaptiveController implementation

AdaptiveController::AdaptiveController(const Eigen::MatrixXd& initialGain)
    : m_adaptiveGain(initialGain), m_referenceModel(Eigen::VectorXd::Zero(6)) {}

ControlCommand AdaptiveController::computeControlCommand(const Eigen::VectorXd& state, const Trajectory& desiredTrajectory) {
    double currentTime = 0.0; // You should replace this with the actual current time
    Eigen::VectorXd desiredState = interpolateDesiredState(desiredTrajectory, currentTime);
    Eigen::VectorXd error = state - desiredState;
    
    Eigen::VectorXd controlInput = computeControlLaw(state, desiredState);
    updateAdaptiveGain(state, desiredState, error);

    ControlCommand command;
    command.thrustVector = controlInput.head<3>();
    command.thrustMagnitude = controlInput.head<3>().norm();
    // Valve commands can be set based on controlInput or other logic
    // For simplicity, setting all valves to false
    command.valveCommands = std::vector<std::pair<ValveType, bool>>{
        {ValveType::FUEL, false},
        {ValveType::OXIDIZER, false}
    };
    return command;
}

void AdaptiveController::updateModelParameters(const SystemIdentificationData& data) {
    // Update reference model based on system identification data
    if (data.outputData.cols() == 0) {
        throw std::invalid_argument("SystemIdentificationData outputData is empty.");
    }
    m_referenceModel = data.outputData.colwise().mean();
}

Eigen::VectorXd AdaptiveController::computeControlLaw(const Eigen::VectorXd& state, const Eigen::VectorXd& desiredState) const {
    Eigen::VectorXd error = state - desiredState;
    return -m_adaptiveGain * error;
}

void AdaptiveController::updateAdaptiveGain(const Eigen::VectorXd& state, const Eigen::VectorXd& desiredState, const Eigen::VectorXd& error) {
    double adaptationRate = 0.001;
    m_adaptiveGain -= adaptationRate * error * state.transpose();
}

Eigen::VectorXd AdaptiveController::interpolateDesiredState(const Trajectory& desiredTrajectory, double time) const {
    // Simple linear interpolation for desired state
    if (desiredTrajectory.timestamps.empty()) {
        throw std::invalid_argument("Desired trajectory is empty.");
    }

    if (time <= desiredTrajectory.timestamps.front()) {
        Eigen::VectorXd desiredState(6);
        desiredState.head<3>() = desiredTrajectory.positions.front();
        desiredState.tail<3>() = desiredTrajectory.velocities.front();
        return desiredState;
    }

    if (time >= desiredTrajectory.timestamps.back()) {
        Eigen::VectorXd desiredState(6);
        desiredState.head<3>() = desiredTrajectory.positions.back();
        desiredState.tail<3>() = desiredTrajectory.velocities.back();
        return desiredState;
    }

    // Find the interval
    auto it = std::lower_bound(desiredTrajectory.timestamps.begin(), desiredTrajectory.timestamps.end(), time);
    int idx = std::distance(desiredTrajectory.timestamps.begin(), it) - 1;

    double t1 = desiredTrajectory.timestamps[idx];
    double t2 = desiredTrajectory.timestamps[idx + 1];
    double alpha = (time - t1) / (t2 - t1);

    Eigen::VectorXd desiredState(6);
    desiredState.head<3>() = (1 - alpha) * desiredTrajectory.positions[idx] + alpha * desiredTrajectory.positions[idx + 1];
    desiredState.tail<3>() = (1 - alpha) * desiredTrajectory.velocities[idx] + alpha * desiredTrajectory.velocities[idx + 1];
    return desiredState;
}

// Factory function implementation

std::unique_ptr<ControlLaw> createController(const std::string& controllerType, const Eigen::MatrixXd& initialParams) {
    if (controllerType == "NMPC") {
        return std::make_unique<NMPCController>(20, 0.1); // 20 steps horizon, 0.1s control interval
    } else if (controllerType == "Adaptive") {
        return std::make_unique<AdaptiveController>(initialParams);
    } else {
        throw std::invalid_argument("Unknown controller type: " + controllerType);
    }
}

} // namespace tvc