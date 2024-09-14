#ifndef PERFORMANCE_OBJECTIVES_HPP
#define PERFORMANCE_OBJECTIVES_HPP

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "control_law.hpp"

namespace tvc {

struct FlightState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angularVelocity;
    double time;
    double time_step; // Add this line
    double fuel_mass; // Add this line
    double oxidizer_mass; // Add this line
};

struct EnginePerformanceData {
    double fuelMassFlow;
    double thrustMagnitude;
    double time;
};

struct PerformanceMetrics {
    double controlError;
    double specificImpulse;
    double structuralLoad;
    double fuelEfficiency;
    double stabilityMargin;
    double trajectoryError;
    double fuelConsumption;
    double thermalLoad;
};

class ObjectiveFunction {
public:
    virtual ~ObjectiveFunction() = default;
    virtual double evaluate(const FlightState& state, const Trajectory& desiredTrajectory, const ControlCommand& command) const = 0;
    virtual double getWeight() const = 0;
    virtual void setWeight(double weight) = 0;
};

class ControlErrorObjective : public ObjectiveFunction {
public:
    ControlErrorObjective(double weight = 1.0);
    double evaluate(const FlightState& state, const Trajectory& desiredTrajectory, const ControlCommand& command) const override;
    double getWeight() const override { return m_weight; }
    void setWeight(double weight) override { m_weight = weight; }

private:
    double m_weight;
};

class SpecificImpulseObjective : public ObjectiveFunction {
public:
    SpecificImpulseObjective(double weight = 1.0);
    double evaluate(const FlightState& state, const Trajectory& desiredTrajectory, const ControlCommand& command) const override;
    double getWeight() const override { return m_weight; }
    void setWeight(double weight) override { m_weight = weight; }

private:
    double m_weight;
};

class StructuralLoadObjective : public ObjectiveFunction {
public:
    StructuralLoadObjective(double weight = 1.0);
    double evaluate(const FlightState& state, const Trajectory& desiredTrajectory, const ControlCommand& command) const override;
    double getWeight() const override { return m_weight; }
    void setWeight(double weight) override { m_weight = weight; }

private:
    double m_weight;
};

class PerformanceObjectives {
public:
    PerformanceObjectives();

    PerformanceMetrics calculateMetrics(const FlightState& state, const Trajectory& desiredTrajectory, const ControlCommand& command);
    double calculateOverallObjective(const FlightState& state, const Trajectory& desiredTrajectory, const ControlCommand& command);

    void addObjectiveFunction(std::unique_ptr<ObjectiveFunction> objective);
    void setObjectiveWeights(const std::vector<double>& weights);

    // Sensitivity analysis
    std::vector<double> calculateSensitivity(const FlightState& state, const Trajectory& desiredTrajectory, const ControlCommand& command);

    // Multi-objective optimization
    std::vector<PerformanceMetrics> generateParetoFront(const FlightState& state, const Trajectory& desiredTrajectory, const ControlCommand& command, int numPoints = 100);

private:
    std::vector<std::unique_ptr<ObjectiveFunction>> m_objectives;

    double calculateControlError(const FlightState& state, const Trajectory& desiredTrajectory) const;
    double calculateSpecificImpulse(const EnginePerformanceData& data) const;
    double calculateStructuralLoad(const FlightState& state, const ControlCommand& command) const;
    double calculateFuelEfficiency(const EnginePerformanceData& data) const;
    double calculateStabilityMargin(const FlightState& state, const ControlCommand& command) const;
};

} // namespace tvc

#endif // PERFORMANCE_OBJECTIVES_HPP
