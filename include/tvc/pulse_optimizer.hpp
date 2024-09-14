#ifndef PULSE_OPTIMIZER_HPP
#define PULSE_OPTIMIZER_HPP

#include <vector>
#include <memory>
#include "tvc/control_law.hpp"
#include "tvc/performance_objectives.hpp"
#include "tvc/state_estimator.hpp"
#include "multi_tube_pde/multi_tube_configuration.hpp"  // Include the actual header

namespace tvc {

enum class OptimizationAlgorithm {
    GradientDescent,
    EvolutionaryAlgorithm
};

struct OptimizationConfig {
    OptimizationAlgorithm algorithm;
    int maxIterations;
    double mutationRate;
    int minTimeBetweenFirings;
    int maxConsecutiveFiringsPerTube;
    // Add any other relevant optimization parameters
};

struct PulseOptimizationResult {
    std::vector<int> firingSequence;
    double pulseFrequency;
    PerformanceMetrics performanceMetrics;
};

class PulseOptimizer {
public:
    PulseOptimizer(std::shared_ptr<MultiTubePDEConfiguration> pdeConfig,
                   std::shared_ptr<PerformanceObjectives> performanceObjectives);
    ~PulseOptimizer() = default;

    PulseOptimizationResult optimizePulses(const FlightState& currentState,
                                           const Trajectory& desiredTrajectory,
                                           const ControlCommand& controlCommand);

    void setOptimizationParameters(const OptimizationConfig& config);

private:
    std::shared_ptr<MultiTubePDEConfiguration> m_pdeConfig;
    std::shared_ptr<PerformanceObjectives> m_performanceObjectives;
    OptimizationConfig m_optimizationConfig;
    std::vector<int> m_currentOptimizedSequence;

    std::vector<int> optimizeFiringSequence(const FlightState& currentState,
                                            const Trajectory& desiredTrajectory,
                                            const ControlCommand& controlCommand);
    double optimizePulseFrequency(const FlightState& currentState);
    bool isSequenceValid(const std::vector<int>& sequence) const;
    double calculateObjectiveFunction(const std::vector<int>& firingSequence,
                                      const FlightState& currentState,
                                      const Trajectory& desiredTrajectory,
                                      const ControlCommand& controlCommand);

    // Optimization algorithm methods
    std::vector<int> gradientDescentOptimization(const FlightState& currentState,
                                                 const Trajectory& desiredTrajectory,
                                                 const ControlCommand& controlCommand);
    std::vector<int> evolutionaryAlgorithmOptimization(const FlightState& currentState,
                                                       const Trajectory& desiredTrajectory,
                                                       const ControlCommand& controlCommand);

    // Helper methods for evolutionary algorithm
    int selectParent(const std::vector<double>& fitness);
    std::pair<std::vector<int>, std::vector<int>> crossover(const std::vector<int>& parent1,
                                                            const std::vector<int>& parent2);
    void mutate(std::vector<int>& individual);
};

} // namespace tvc

#endif // PULSE_OPTIMIZER_HPP