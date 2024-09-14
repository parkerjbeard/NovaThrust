#include "tvc/pulse_optimizer.hpp"
#include <algorithm>
#include <random>
#include <cmath>
#include "tvc/performance_objectives.hpp" // Ensure this is included
#include "tvc/control_law.hpp" // Include ControlCommand

namespace tvc {

PulseOptimizer::PulseOptimizer(std::shared_ptr<MultiTubePDEConfiguration> pdeConfig,
                               std::shared_ptr<PerformanceObjectives> performanceObjectives)
    : m_pdeConfig(std::move(pdeConfig)),
      m_performanceObjectives(std::move(performanceObjectives)) {}

PulseOptimizationResult PulseOptimizer::optimizePulses(const FlightState& currentState,
                                                       const Trajectory& desiredTrajectory,
                                                       const ControlCommand& controlCommand) {
    PulseOptimizationResult result;
    result.firingSequence = optimizeFiringSequence(currentState, desiredTrajectory, controlCommand);
    result.pulseFrequency = optimizePulseFrequency(currentState);
    
    FlightState simulatedState = currentState;
    m_pdeConfig->simulateFiringSequence(simulatedState, result.firingSequence, controlCommand);
    
    // Pass the controlCommand as the third argument
    result.performanceMetrics = m_performanceObjectives->calculateMetrics(simulatedState, desiredTrajectory, controlCommand);
    return result;
}

void PulseOptimizer::setOptimizationParameters(const OptimizationConfig& config) {
    m_optimizationConfig = config;
}

std::vector<int> PulseOptimizer::optimizeFiringSequence(const FlightState& currentState,
                                                        const Trajectory& desiredTrajectory,
                                                        const ControlCommand& controlCommand) {
    if (m_optimizationConfig.algorithm == OptimizationAlgorithm::GradientDescent) {
        return gradientDescentOptimization(currentState, desiredTrajectory, controlCommand);
    } else {
        return evolutionaryAlgorithmOptimization(currentState, desiredTrajectory, controlCommand);
    }
}

double PulseOptimizer::optimizePulseFrequency(const FlightState& currentState) {
    // Implement pulse frequency optimization based on flight conditions
    double machNumber = currentState.velocity.norm() / 343.0; // Assuming speed of sound is 343 m/s
    double altitude = currentState.position.z();
    
    // Basic frequency scaling based on Mach number and altitude
    double baseFrequency = 100.0; // Hz
    double machScaling = std::max(1.0, machNumber);
    double altitudeScaling = std::exp(-altitude / 10000.0); // Decrease frequency with altitude
    
    return baseFrequency * machScaling * altitudeScaling;
}

bool PulseOptimizer::isSequenceValid(const std::vector<int>& sequence) const {
    int consecutiveFirings = 0;
    int lastFiringIndex = -1;

    for (int i = 0; i < sequence.size(); ++i) {
        if (sequence[i] == 1) {
            if (lastFiringIndex != -1 && (i - lastFiringIndex) < m_optimizationConfig.minTimeBetweenFirings) {
                return false;
            }
            lastFiringIndex = i;
            consecutiveFirings++;
            if (consecutiveFirings > m_optimizationConfig.maxConsecutiveFiringsPerTube) {
                return false;
            }
        } else {
            consecutiveFirings = 0;
        }
    }

    return true;
}

double PulseOptimizer::calculateObjectiveFunction(const std::vector<int>& firingSequence,
                                                  const FlightState& currentState,
                                                  const Trajectory& desiredTrajectory,
                                                  const ControlCommand& controlCommand) {
    FlightState simulatedState = currentState;
    m_pdeConfig->simulateFiringSequence(simulatedState, firingSequence, controlCommand);
    
    // Calculate performance metrics based on simulated state and desired trajectory
    PerformanceMetrics metrics = m_performanceObjectives->calculateMetrics(simulatedState, desiredTrajectory, controlCommand);
    
    // Return a single objective value (lower is better)
    return metrics.trajectoryError + metrics.fuelConsumption + metrics.thermalLoad;
}

std::vector<int> PulseOptimizer::gradientDescentOptimization(const FlightState& currentState,
                                                             const Trajectory& desiredTrajectory,
                                                             const ControlCommand& controlCommand) {
    const size_t numTubes = m_pdeConfig->getNumberOfTubes();
    std::vector<int> bestSequence(numTubes, 0);
    double bestObjective = std::numeric_limits<double>::max();

    for (int iteration = 0; iteration < m_optimizationConfig.maxIterations; ++iteration) {
        for (size_t i = 0; i < numTubes; ++i) {
            std::vector<int> testSequence = bestSequence;
            testSequence[i] = 1 - testSequence[i];  // Flip the bit
            
            double objective = calculateObjectiveFunction(testSequence, currentState, desiredTrajectory, controlCommand);
            
            if (objective < bestObjective) {
                bestSequence = testSequence;
                bestObjective = objective;
            }
        }
    }

    m_currentOptimizedSequence = bestSequence;
    return bestSequence;
}

std::vector<int> PulseOptimizer::evolutionaryAlgorithmOptimization(const FlightState& currentState,
                                                                   const Trajectory& desiredTrajectory,
                                                                   const ControlCommand& controlCommand) {
    const size_t numTubes = m_pdeConfig->getNumberOfTubes();
    const int populationSize = 100;
    std::vector<std::vector<int>> population(populationSize, std::vector<int>(numTubes));
    std::vector<double> fitness(populationSize);

    // Initialize population
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 1);
    for (auto& individual : population) {
        for (int& gene : individual) {
            gene = dis(gen);
        }
    }

    for (int generation = 0; generation < m_optimizationConfig.maxIterations; ++generation) {
        // Evaluate fitness
        for (int i = 0; i < populationSize; ++i) {
            fitness[i] = -calculateObjectiveFunction(population[i], currentState, desiredTrajectory, controlCommand);
        }

        // Select best individual
        auto bestIt = std::max_element(fitness.begin(), fitness.end());
        int bestIndex = std::distance(fitness.begin(), bestIt);

        // Create new population
        std::vector<std::vector<int>> newPopulation;
        for (int i = 0; i < populationSize / 2; ++i) {
            int parent1 = selectParent(fitness);
            int parent2 = selectParent(fitness);
            auto [child1, child2] = crossover(population[parent1], population[parent2]);
            mutate(child1);
            mutate(child2);
            newPopulation.push_back(child1);
            newPopulation.push_back(child2);
        }

        // Elitism: keep the best individual
        newPopulation[0] = population[bestIndex];
        population = newPopulation;
    }

    // Evaluate fitness one last time
    for (int i = 0; i < populationSize; ++i) {
        fitness[i] = -calculateObjectiveFunction(population[i], currentState, desiredTrajectory, controlCommand);
    }

    // Find the best individual in the final population
    auto bestItFinal = std::max_element(fitness.begin(), fitness.end());
    int bestFinalIndex = std::distance(fitness.begin(), bestItFinal);

    m_currentOptimizedSequence = population[bestFinalIndex];
    return m_currentOptimizedSequence;
}

int PulseOptimizer::selectParent(const std::vector<double>& fitness) {
    // Implement parent selection (e.g., tournament selection)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, fitness.size() - 1);

    int best = dis(gen);
    for (int i = 1; i < 3; ++i) {
        int candidate = dis(gen);
        if (fitness[candidate] > fitness[best]) {
            best = candidate;
        }
    }
    return best;
}

std::pair<std::vector<int>, std::vector<int>> PulseOptimizer::crossover(const std::vector<int>& parent1, const std::vector<int>& parent2) {
    // Implement crossover (e.g., single-point crossover)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, parent1.size() - 1);

    int crossoverPoint = dis(gen);
    std::vector<int> child1 = parent1;
    std::vector<int> child2 = parent2;

    for (int i = crossoverPoint; i < parent1.size(); ++i) {
        std::swap(child1[i], child2[i]);
    }

    return {child1, child2};
}

void PulseOptimizer::mutate(std::vector<int>& individual) {
    // Implement mutation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    for (int& gene : individual) {
        if (dis(gen) < m_optimizationConfig.mutationRate) {
            gene = 1 - gene; // Flip the bit
        }
    }
}

} // namespace tvc