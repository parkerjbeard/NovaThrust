#include "multi_tube_pde/valve_controller.hpp"
#include <stdexcept>

ValveController::ValveController(size_t numTubes)
    : m_numTubes(numTubes), m_valveStates(numTubes), m_currentTime(0.0) {
    if (numTubes == 0) {
        throw std::invalid_argument("Number of tubes must be positive");
    }
}

void ValveController::initialize() {
    for (auto& tubeValves : m_valveStates) {
        for (auto& valve : tubeValves) {
            valve = {false, 0.0, 0.0};
        }
    }
}

void ValveController::updateValves(double currentTime) {
    m_currentTime = currentTime;
    for (size_t i = 0; i < m_numTubes; ++i) {
        for (int j = 0; j < 2; ++j) {
            auto& valve = m_valveStates[i][j];
            if (!valve.isOpen && m_currentTime >= valve.openTime) {
                openValve(i, static_cast<ValveType>(j));
            } else if (valve.isOpen && m_currentTime >= valve.closeTime) {
                closeValve(i, static_cast<ValveType>(j));
            }
        }
    }
}

void ValveController::openValve(size_t tubeIndex, ValveType type) {
    validateTubeIndex(tubeIndex);
    m_valveStates[tubeIndex][static_cast<int>(type)].isOpen = true;
}

void ValveController::closeValve(size_t tubeIndex, ValveType type) {
    validateTubeIndex(tubeIndex);
    m_valveStates[tubeIndex][static_cast<int>(type)].isOpen = false;
}

ValveState ValveController::getValveState(size_t tubeIndex, ValveType type) const {
    validateTubeIndex(tubeIndex);
    return m_valveStates[tubeIndex][static_cast<int>(type)];
}

void ValveController::setValveTiming(size_t tubeIndex, ValveType type, double openTime, double closeTime) {
    validateTubeIndex(tubeIndex);
    if (openTime >= closeTime) {
        throw std::invalid_argument("Open time must be less than close time");
    }
    auto& valve = m_valveStates[tubeIndex][static_cast<int>(type)];
    valve.openTime = openTime;
    valve.closeTime = closeTime;
}

void ValveController::validateTubeIndex(size_t tubeIndex) const {
    if (tubeIndex >= m_numTubes) {
        throw std::out_of_range("Tube index out of range");
    }
}