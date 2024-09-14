#include "multi_tube_pde/tube_interaction_model.hpp"
#include <cmath>
#include <stdexcept>

TubeInteractionModel::TubeInteractionModel(size_t numTubes)
    : m_numTubes(numTubes), m_interactionMatrix(numTubes, std::vector<double>(numTubes, 0.0)) {
    if (numTubes == 0) {
        throw std::invalid_argument("Number of tubes must be positive");
    }
}

void TubeInteractionModel::initialize() {
    calculateInteractionMatrix();
}

void TubeInteractionModel::updateInteractions(const std::vector<std::unique_ptr<PDETube>>& tubes, double timeStep) {
    if (tubes.size() != m_numTubes) {
        throw std::invalid_argument("Number of tubes does not match the interaction model");
    }

    calculateCrossTalk(tubes, timeStep);
    propagatePressureWaves(tubes, timeStep);
}

double TubeInteractionModel::getInteractionStrength(size_t tube1, size_t tube2) const {
    if (tube1 >= m_numTubes || tube2 >= m_numTubes) {
        throw std::out_of_range("Tube index out of range");
    }
    return m_interactionMatrix[tube1][tube2];
}

void TubeInteractionModel::calculateInteractionMatrix() {
    // This is a simplified model. In a real implementation, you would need to consider
    // the geometric arrangement of the tubes and other factors affecting interaction strength.
    for (size_t i = 0; i < m_numTubes; ++i) {
        for (size_t j = 0; j < m_numTubes; ++j) {
            if (i == j) {
                m_interactionMatrix[i][j] = 0.0;
            } else {
                // Assume interaction strength decreases with "distance" between tubes
                double distance = std::abs(static_cast<int>(i) - static_cast<int>(j));
                m_interactionMatrix[i][j] = 1.0 / (1.0 + distance);
            }
        }
    }
}

void TubeInteractionModel::calculateCrossTalk(const std::vector<std::unique_ptr<PDETube>>& tubes, double timeStep) {
    // This method implements a simplified algorithm for calculating inter-tube effects.
    for (size_t i = 0; i < m_numTubes; ++i) {
        for (size_t j = 0; j < m_numTubes; ++j) {
            if (i != j) {
                PDETubeState state_i = tubes[i]->getState();
                PDETubeState state_j = tubes[j]->getState();
                
                double pressureDifference = state_i.pressure - state_j.pressure;
                double interactionStrength = m_interactionMatrix[i][j];
                
                // Apply a simplified pressure equalization effect
                double pressureTransfer = pressureDifference * interactionStrength * timeStep;
                
                // Update pressures
                tubes[i]->updatePressure(state_i.pressure - pressureTransfer);
                tubes[j]->updatePressure(state_j.pressure + pressureTransfer);
            }
        }
    }
}

void TubeInteractionModel::propagatePressureWaves(const std::vector<std::unique_ptr<PDETube>>& tubes, double timeStep) {
    std::vector<double> pressureEffects(m_numTubes, 0.0);

    for (size_t i = 0; i < m_numTubes; ++i) {
        for (size_t j = 0; j < m_numTubes; ++j) {
            if (i != j) {
                double effect = calculatePressureEffect(*tubes[i], *tubes[j]);
                pressureEffects[j] += effect * m_interactionMatrix[i][j];
            }
        }
    }

    // Apply pressure effects to each tube
    for (size_t i = 0; i < m_numTubes; ++i) {
        PDETubeState currentState = tubes[i]->getState();
        double newPressure = currentState.pressure + pressureEffects[i] * timeStep;
        tubes[i]->updatePressure(newPressure);
    }
}

double TubeInteractionModel::calculatePressureEffect(const PDETube& sourceTube, const PDETube& targetTube) const {
    // This is a simplified model. In a real implementation, you would need to consider
    // the phase of each tube, the pressure difference, and other factors.
    PDETubeState sourceState = sourceTube.getState();
    PDETubeState targetState = targetTube.getState();

    double pressureDifference = sourceState.pressure - targetState.pressure;
    
    // Only propagate pressure waves if the source tube is in the FIRE phase
    if (sourceState.phase == DetonationPhase::FIRE) {
        return pressureDifference * 0.1; // Arbitrary scaling factor
    } else {
        return 0.0;
    }
}