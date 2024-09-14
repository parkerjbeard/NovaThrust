#include "tvc/performance_objectives.hpp"
#include <algorithm>
#include <cmath>
#include <random>

namespace tvc {

// ObjectiveFunction implementations

ControlErrorObjective::ControlErrorObjective(double weight) : m_weight(weight) {}

double ControlErrorObjective::evaluate(const FlightState& state, const Trajectory& desiredTrajectory, const ControlCommand& command) const {
    // Simplified control error calculation
    Eigen::Vector3d positionError = state.position - desiredTrajectory.positions.front();
    return m_weight * positionError.norm();
}

SpecificImpulseObjective::SpecificImpulseObjective(double weight) : m_weight(weight) {}

double SpecificImpulseObjective::evaluate(const FlightState& state, const Trajectory& desiredTrajectory, const ControlCommand& command) const {
    // Simplified specific impulse calculation
    EnginePerformanceData data{0.1, command.thrustMagnitude, 1.0}; // Example data
    double isp = (command.thrustMagnitude * 9.81) / data.fuelMassFlow;
    return m_weight * (1.0 / isp); // Minimize inverse of Isp
}

StructuralLoadObjective::StructuralLoadObjective(double weight) : m_weight(weight) {}

double StructuralLoadObjective::evaluate(const FlightState& state, const Trajectory& desiredTrajectory, const ControlCommand& command) const {
    // Simplified structural load calculation
    double dynamicPressure = 0.5 * 1.225 * state.velocity.squaredNorm(); // Assuming sea level density
    double loadFactor = command.thrustMagnitude / (9.81 * 1000.0); // Assuming 1000 kg vehicle mass
    return m_weight * dynamicPressure * loadFactor;
}

// PerformanceObjectives implementation

PerformanceObjectives::PerformanceObjectives() {
    addObjectiveFunction(std::make_unique<ControlErrorObjective>());
    addObjectiveFunction(std::make_unique<SpecificImpulseObjective>());
    addObjectiveFunction(std::make_unique<StructuralLoadObjective>());
}

PerformanceMetrics PerformanceObjectives::calculateMetrics(const FlightState& state, const Trajectory& desiredTrajectory, const ControlCommand& command) {
    PerformanceMetrics metrics;
    metrics.controlError = calculateControlError(state, desiredTrajectory);
    
    EnginePerformanceData engineData{0.1, command.thrustMagnitude, 1.0}; // Example data
    metrics.specificImpulse = calculateSpecificImpulse(engineData);
    metrics.structuralLoad = calculateStructuralLoad(state, command);
    metrics.fuelEfficiency = calculateFuelEfficiency(engineData);
    metrics.stabilityMargin = calculateStabilityMargin(state, command);

    // Assign new members
    metrics.trajectoryError = metrics.controlError; // Example assignment, adjust as needed
    metrics.fuelConsumption = engineData.fuelMassFlow; // Example assignment
    metrics.thermalLoad = metrics.structuralLoad; // Example assignment, adjust as needed

    return metrics;
}

double PerformanceObjectives::calculateOverallObjective(const FlightState& state, const Trajectory& desiredTrajectory, const ControlCommand& command) {
    double totalObjective = 0.0;
    for (const auto& objective : m_objectives) {
        totalObjective += objective->evaluate(state, desiredTrajectory, command);
    }
    return totalObjective;
}

void PerformanceObjectives::addObjectiveFunction(std::unique_ptr<ObjectiveFunction> objective) {
    m_objectives.push_back(std::move(objective));
}

void PerformanceObjectives::setObjectiveWeights(const std::vector<double>& weights) {
    if (weights.size() != m_objectives.size()) {
        throw std::invalid_argument("Number of weights must match number of objectives");
    }
    for (size_t i = 0; i < weights.size(); ++i) {
        m_objectives[i]->setWeight(weights[i]);
    }
}

std::vector<double> PerformanceObjectives::calculateSensitivity(const FlightState& state, const Trajectory& desiredTrajectory, const ControlCommand& command) {
    std::vector<double> sensitivities;
    double baseObjective = calculateOverallObjective(state, desiredTrajectory, command);
    
    for (size_t i = 0; i < m_objectives.size(); ++i) {
        double originalWeight = m_objectives[i]->getWeight();
        m_objectives[i]->setWeight(originalWeight * 1.01); // 1% increase
        double perturbedObjective = calculateOverallObjective(state, desiredTrajectory, command);
        m_objectives[i]->setWeight(originalWeight); // Reset weight
        
        double sensitivity = (perturbedObjective - baseObjective) / (0.01 * originalWeight);
        sensitivities.push_back(sensitivity);
    }
    
    return sensitivities;
}

std::vector<PerformanceMetrics> PerformanceObjectives::generateParetoFront(const FlightState& state, const Trajectory& desiredTrajectory, const ControlCommand& command, int numPoints) {
    std::vector<PerformanceMetrics> paretoFront;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    for (int i = 0; i < numPoints; ++i) {
        // Generate random weights
        std::vector<double> weights;
        for (size_t j = 0; j < m_objectives.size(); ++j) {
            weights.push_back(dis(gen));
        }
        
        // Normalize weights
        double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
        for (auto& w : weights) {
            w /= sum;
        }
        
        setObjectiveWeights(weights);
        paretoFront.push_back(calculateMetrics(state, desiredTrajectory, command));
    }

    return paretoFront;
}

double PerformanceObjectives::calculateControlError(const FlightState& state, const Trajectory& desiredTrajectory) const {
    Eigen::Vector3d positionError = state.position - desiredTrajectory.positions.front();
    Eigen::Vector3d velocityError = state.velocity - desiredTrajectory.velocities.front();
    return positionError.norm() + velocityError.norm();
}

double PerformanceObjectives::calculateSpecificImpulse(const EnginePerformanceData& data) const {
    return (data.thrustMagnitude * 9.81) / data.fuelMassFlow;
}

double PerformanceObjectives::calculateStructuralLoad(const FlightState& state, const ControlCommand& command) const {
    double dynamicPressure = 0.5 * 1.225 * state.velocity.squaredNorm(); // Assuming sea level density
    double loadFactor = command.thrustMagnitude / (9.81 * 1000.0); // Assuming 1000 kg vehicle mass
    return dynamicPressure * loadFactor;
}

double PerformanceObjectives::calculateFuelEfficiency(const EnginePerformanceData& data) const {
    return data.thrustMagnitude / data.fuelMassFlow;
}

double PerformanceObjectives::calculateStabilityMargin(const FlightState& state, const ControlCommand& command) const {
    // Simplified stability margin calculation
    Eigen::Vector3d thrustVector = command.thrustVector.normalized();
    double angleOfAttack = std::acos(thrustVector.dot(state.velocity.normalized()));
    return std::cos(angleOfAttack); // Higher value indicates better stability
}

} // namespace tvc