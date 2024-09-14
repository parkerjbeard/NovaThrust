#include "multi_tube_pde/multi_tube_configuration.hpp"
#include "tvc/performance_objectives.hpp"
#include "tvc/control_law.hpp"

namespace tvc {

MultiTubePDEConfiguration::MultiTubePDEConfiguration(size_t numTubes)
    : m_numTubes(numTubes),
      m_tubes(numTubes),
      m_manifoldSystem(std::make_unique<ManifoldSystem>(numTubes)),
      m_thrustVectoringSystem(std::make_unique<ThrustVectoringSystem>(numTubes)),
      m_tubeInteractionModel(std::make_unique<TubeInteractionModel>(numTubes)),
      m_valveController(std::make_unique<ValveController>(numTubes)) {
    for (size_t i = 0; i < numTubes; ++i) {
        m_tubes[i] = std::make_unique<PDETube>(1.0, 0.1); // Example dimensions
    }
}

void MultiTubePDEConfiguration::initialize() {
    for (auto& tube : m_tubes) {
        tube->initialize();
    }
    m_manifoldSystem->initialize();
    m_thrustVectoringSystem->initialize();
    m_tubeInteractionModel->initialize();
    m_valveController->initialize();
}

void MultiTubePDEConfiguration::updateConfiguration(double timeStep) {
    updateTubes(timeStep);
    updateSystems(timeStep);
}

std::vector<PDETubeState> MultiTubePDEConfiguration::getTubeStates() const {
    std::vector<PDETubeState> states;
    states.reserve(m_numTubes);
    for (const auto& tube : m_tubes) {
        states.push_back(tube->getState());
    }
    return states;
}

std::vector<int> MultiTubePDEConfiguration::optimizeFiringSequence(const ThrustVector& desiredThrust) {
    return m_thrustVectoringSystem->optimizeFiringSequence(m_tubes, desiredThrust);
}

void MultiTubePDEConfiguration::updateTubes(double timeStep) {
    for (auto& tube : m_tubes) {
        tube->updateState(timeStep);
    }
    m_tubeInteractionModel->updateInteractions(m_tubes, timeStep);
}

void MultiTubePDEConfiguration::updateSystems(double timeStep) {
    m_manifoldSystem->updateState(timeStep);
    m_valveController->updateValves(timeStep);

    // Update tube states based on manifold and valve states
    for (size_t i = 0; i < m_numTubes; ++i) {
        auto fuelValveState = m_valveController->getValveState(i, ValveType::FUEL);
        auto oxidizerValveState = m_valveController->getValveState(i, ValveType::OXIDIZER);
        auto manifoldState = m_manifoldSystem->getManifoldState();

        if (fuelValveState.isOpen && oxidizerValveState.isOpen) {
            double fuelMass = manifoldState.fuelFlowRate * timeStep;
            double oxidizerMass = manifoldState.oxidizerFlowRate * timeStep;
            m_tubes[i]->injectPropellants(fuelMass, oxidizerMass);
        }
    }
}

const TubeInteractionModel& MultiTubePDEConfiguration::getInteractionModel() const {
    return *m_tubeInteractionModel;
}

const ThrustVectoringSystem& MultiTubePDEConfiguration::getThrustVectoringSystem() const {
    return *m_thrustVectoringSystem;
}

// Implementation of the added methods

size_t MultiTubePDEConfiguration::getNumberOfTubes() const {
    return m_numTubes;
}

void MultiTubePDEConfiguration::simulateFiringSequence(FlightState& simulatedState,
                                                       const std::vector<int>& firingSequence,
                                                       const ControlCommand& controlCommand) {
    // Validate firing sequence length
    if (firingSequence.size() != m_numTubes) {
        throw std::invalid_argument("Firing sequence size does not match the number of tubes.");
    }

    // Apply control commands to update valve states
    for (const auto& [valveType, isOpen] : controlCommand.valveCommands) {
        // Assuming valveType applies to all tubes. Modify as needed.
        for (size_t i = 0; i < m_numTubes; ++i) {
            if (isOpen) {
                m_valveController->openValve(i, valveType);
            } else {
                m_valveController->closeValve(i, valveType);
            }
        }
    }

    // Iterate through each tube and apply firing sequence
    for (size_t i = 0; i < m_numTubes; ++i) {
        if (firingSequence[i] == 1) {
            // Fire the tube by injecting propellants
            double fuelMass = 1.0;      // Example fuel mass
            double oxidizerMass = 1.0; // Example oxidizer mass
            m_tubes[i]->injectPropellants(fuelMass, oxidizerMass);

            // Update the simulated state based on tube firing
            // Calculate thrust using ThrustVectoringSystem
            ThrustVector thrustVector = m_thrustVectoringSystem->calculateThrustVector(getTubeStates());
            simulatedState.velocity += (Eigen::Vector3d(thrustVector.x, thrustVector.y, thrustVector.z) * simulatedState.time_step);
        } else {
            // Ensure the tube is not firing by not injecting propellants
            // Additional idle logic can be implemented if necessary
        }

        // Update the tube state
        m_tubes[i]->updateState(simulatedState.time_step);
    }

    // Update interactions between tubes
    m_tubeInteractionModel->updateInteractions(m_tubes, simulatedState.time_step);

    // Update the simulated state based on manifold interactions
    auto manifoldState = m_manifoldSystem->getManifoldState();
    simulatedState.fuel_mass -= manifoldState.fuelFlowRate * simulatedState.time_step;
    simulatedState.oxidizer_mass -= manifoldState.oxidizerFlowRate * simulatedState.time_step;

    // Update overall system (valves, manifolds, etc.)
    updateConfiguration(simulatedState.time_step);
}

} // namespace tvc