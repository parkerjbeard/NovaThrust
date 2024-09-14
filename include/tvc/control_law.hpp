#ifndef CONTROL_LAW_HPP
#define CONTROL_LAW_HPP

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <utility>
#include <string>
#include "state_estimator.hpp"
#include "multi_tube_pde/valve_controller.hpp"

namespace tvc {

struct Trajectory {
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector3d> velocities;
    std::vector<Eigen::Vector3d> accelerations;
    std::vector<double> timestamps;
};

struct ControlCommand {
    Eigen::Vector3d thrustVector;
    double thrustMagnitude;
    std::vector<std::pair<ValveType, bool>> valveCommands;

    // Add copy constructor and copy assignment operator
    ControlCommand() = default;
    ControlCommand(const ControlCommand&) = default;
    ControlCommand& operator=(const ControlCommand&) = default;
};

struct SystemIdentificationData {
    Eigen::MatrixXd inputData;
    Eigen::MatrixXd outputData;
    // Add more fields as needed for system identification
};

class ControlLaw {
public:
    virtual ~ControlLaw() = default;
    virtual ControlCommand computeControlCommand(const Eigen::VectorXd& state, const Trajectory& desiredTrajectory) = 0;
    virtual void updateModelParameters(const SystemIdentificationData& data) = 0;
};

class NMPCController : public ControlLaw {
public:
    NMPCController(int horizonLength, double controlInterval);
    ControlCommand computeControlCommand(const Eigen::VectorXd& state, const Trajectory& desiredTrajectory) override;
    void updateModelParameters(const SystemIdentificationData& data) override;

private:
    int m_horizonLength;
    double m_controlInterval;
    Eigen::MatrixXd m_modelParameters;

    // Added member variables
    Eigen::MatrixXd m_A;
    Eigen::MatrixXd m_B;
    Eigen::MatrixXd m_Q;
    Eigen::MatrixXd m_R;

    // Added method
    Eigen::VectorXd interpolateDesiredState(const Trajectory& desiredTrajectory, double time) const;

    Eigen::VectorXd predictState(const Eigen::VectorXd& initialState, const std::vector<ControlCommand>& controlSequence) const;
    double computeCost(const std::vector<Eigen::VectorXd>& stateSequence, const std::vector<ControlCommand>& controlSequence, const Trajectory& desiredTrajectory) const;
    std::vector<ControlCommand> optimizeControlSequence(const Eigen::VectorXd& initialState, const Trajectory& desiredTrajectory);
};

class AdaptiveController : public ControlLaw {
public:
    AdaptiveController(const Eigen::MatrixXd& initialGain);
    ControlCommand computeControlCommand(const Eigen::VectorXd& state, const Trajectory& desiredTrajectory) override;
    void updateModelParameters(const SystemIdentificationData& data) override;

private:
    Eigen::MatrixXd m_adaptiveGain;
    Eigen::VectorXd m_referenceModel;

    Eigen::VectorXd computeControlLaw(const Eigen::VectorXd& state, const Eigen::VectorXd& desiredState) const;
    void updateAdaptiveGain(const Eigen::VectorXd& state, const Eigen::VectorXd& desiredState, const Eigen::VectorXd& error);
    Eigen::VectorXd interpolateDesiredState(const Trajectory& desiredTrajectory, double time) const;
};

// Factory function to create different types of controllers
std::unique_ptr<ControlLaw> createController(const std::string& controllerType, const Eigen::MatrixXd& initialParams);

} // namespace tvc

#endif // CONTROL_LAW_HPP
