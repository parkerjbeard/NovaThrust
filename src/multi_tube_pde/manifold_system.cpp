#include "multi_tube_pde/manifold_system.hpp"
#include <stdexcept>
#include <algorithm>
#include <numeric>  // Add this line to include std::accumulate

ManifoldSystem::ManifoldSystem(size_t numTubes)
    : m_numTubes(numTubes),
      m_tubeFuelInjectionRates(numTubes, 0.0),
      m_tubeOxidizerInjectionRates(numTubes, 0.0) {
    if (numTubes == 0) {
        throw std::invalid_argument("Number of tubes must be positive");
    }
}

void ManifoldSystem::initialize() {
    m_state = {
        .fuelPressure = 1000000.0,  // 10 bar
        .oxidizerPressure = 1000000.0,  // 10 bar
        .fuelFlowRate = 0.0,
        .oxidizerFlowRate = 0.0
    };
}

void ManifoldSystem::updateState(double timeStep) {
    calculateInjectionRates();
    updateManifoldPressures(timeStep);
}

void ManifoldSystem::injectFuel(size_t tubeIndex, double amount) {
    if (tubeIndex >= m_numTubes) {
        throw std::out_of_range("Tube index out of range");
    }
    m_tubeFuelInjectionRates[tubeIndex] = amount;
}

void ManifoldSystem::injectOxidizer(size_t tubeIndex, double amount) {
    if (tubeIndex >= m_numTubes) {
        throw std::out_of_range("Tube index out of range");
    }
    m_tubeOxidizerInjectionRates[tubeIndex] = amount;
}

ManifoldState ManifoldSystem::getManifoldState() const {
    return m_state;
}

void ManifoldSystem::updateManifoldPressures(double timeStep) {
    // Simple model: pressure decreases as flow rate increases
    double totalFuelFlow = m_state.fuelFlowRate * timeStep;
    double totalOxidizerFlow = m_state.oxidizerFlowRate * timeStep;

    m_state.fuelPressure -= totalFuelFlow * 1000.0;  // Arbitrary scaling factor
    m_state.oxidizerPressure -= totalOxidizerFlow * 1000.0;  // Arbitrary scaling factor

    // Ensure pressures don't go below a minimum value
    m_state.fuelPressure = std::max(m_state.fuelPressure, 100000.0);  // 1 bar minimum
    m_state.oxidizerPressure = std::max(m_state.oxidizerPressure, 100000.0);  // 1 bar minimum
}

void ManifoldSystem::calculateInjectionRates() {
    m_state.fuelFlowRate = std::accumulate(m_tubeFuelInjectionRates.begin(), m_tubeFuelInjectionRates.end(), 0.0);
    m_state.oxidizerFlowRate = std::accumulate(m_tubeOxidizerInjectionRates.begin(), m_tubeOxidizerInjectionRates.end(), 0.0);
}