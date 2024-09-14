#include "multi_tube_pde/thrust_vectoring_system.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <stdexcept>

constexpr double PI = 3.14159265358979323846;
constexpr double R = 8.314; // Universal gas constant, J/(mol·K)
constexpr double g0 = 9.80665; // Standard gravity, m/s^2

ThrustVectoringSystem::ThrustVectoringSystem(size_t numTubes)
    : m_numTubes(numTubes), m_tubeOrientations(numTubes) {
    if (numTubes == 0) {
        throw std::invalid_argument("Number of tubes must be positive");
    }
}

void ThrustVectoringSystem::initialize() {
    // Initialize tube orientations (example: evenly distributed around a circle)
    for (size_t i = 0; i < m_numTubes; ++i) {
        double angle = 2 * PI * i / m_numTubes;
        m_tubeOrientations[i] = {std::cos(angle), std::sin(angle), 0.0};
    }

    m_currentFlightConditions = {0.0, 0.0, 0.0, 0.0}; // Initialize to sea level, stationary
}

double ThrustVectoringSystem::calculateTubeThrustMagnitude(const PDETubeState& tubeState) const {
    double fillFraction = calculateFillFraction(tubeState);
    double Isp = calculateSpecificImpulse(tubeState);
    double throatArea = calculateThroatArea(tubeState);
    double expansionRatio = calculateExpansionRatio(m_currentFlightConditions.altitude);
    double ambientPressure = calculateAmbientPressure(m_currentFlightConditions.altitude);

    // Thrust equation based on Endo et al. (2004)
    double chamberPressure = tubeState.pressure;
    double exitPressure = chamberPressure / std::pow(expansionRatio, 1.4); // Assuming γ = 1.4 for simplicity
    double thrustCoefficient = std::sqrt(2 * 1.4 * 1.4 / (1.4 - 1) * (1 - std::pow(exitPressure / chamberPressure, (1.4 - 1) / 1.4)))
                               + expansionRatio * (exitPressure - ambientPressure) / chamberPressure;

    return fillFraction * throatArea * chamberPressure * thrustCoefficient;
}

ThrustVector ThrustVectoringSystem::calculateThrustVector(const std::vector<PDETubeState>& tubeStates) const {
    ThrustVector totalThrust = {0.0, 0.0, 0.0};

    for (size_t i = 0; i < m_numTubes; ++i) {
        double thrustMagnitude = calculateTubeThrustMagnitude(tubeStates[i]);
        totalThrust.x += thrustMagnitude * m_tubeOrientations[i][0];
        totalThrust.y += thrustMagnitude * m_tubeOrientations[i][1];
        totalThrust.z += thrustMagnitude * m_tubeOrientations[i][2];
    }

    // Adjust for flight conditions
    return rotateThrust(totalThrust, m_currentFlightConditions.angle_of_attack, m_currentFlightConditions.sideslip_angle);
}

std::vector<int> ThrustVectoringSystem::optimizeFiringSequence(const std::vector<std::unique_ptr<PDETube>>& tubes, const ThrustVector& desiredThrust) {
    std::vector<int> sequence;
    ThrustVector currentThrust = {0.0, 0.0, 0.0};
    std::vector<bool> usedTubes(m_numTubes, false);

    while (sequence.size() < m_numTubes) {
        int bestTube = -1;
        double bestImprovement = -1.0;

        for (size_t i = 0; i < m_numTubes; ++i) {
            if (usedTubes[i]) continue;

            double thrustMagnitude = calculateTubeThrustMagnitude(tubes[i]->getState());
            ThrustVector tubeThrust = {
                thrustMagnitude * m_tubeOrientations[i][0],
                thrustMagnitude * m_tubeOrientations[i][1],
                thrustMagnitude * m_tubeOrientations[i][2]
            };

            ThrustVector newThrust = {
                currentThrust.x + tubeThrust.x,
                currentThrust.y + tubeThrust.y,
                currentThrust.z + tubeThrust.z
            };

            double currentError = std::hypot(desiredThrust.x - currentThrust.x,
                                             desiredThrust.y - currentThrust.y,
                                             desiredThrust.z - currentThrust.z);
            double newError = std::hypot(desiredThrust.x - newThrust.x,
                                         desiredThrust.y - newThrust.y,
                                         desiredThrust.z - newThrust.z);
            double improvement = currentError - newError;

            if (improvement > bestImprovement) {
                bestImprovement = improvement;
                bestTube = i;
            }
        }

        if (bestTube == -1) break;

        sequence.push_back(bestTube);
        usedTubes[bestTube] = true;
        double thrustMagnitude = calculateTubeThrustMagnitude(tubes[bestTube]->getState());
        currentThrust.x += thrustMagnitude * m_tubeOrientations[bestTube][0];
        currentThrust.y += thrustMagnitude * m_tubeOrientations[bestTube][1];
        currentThrust.z += thrustMagnitude * m_tubeOrientations[bestTube][2];
    }

    return sequence;
}

std::vector<ThrustVector> ThrustVectoringSystem::predictThrustProfile(const std::vector<int>& firingSequence, double duration) {
    std::vector<ThrustVector> profile;
    const double timeStep = 0.001; // 1 ms time step
    int numSteps = static_cast<int>(duration / timeStep);

    for (int step = 0; step < numSteps; ++step) {
        double time = step * timeStep;
        ThrustVector thrust = {0.0, 0.0, 0.0};

        for (size_t i = 0; i < firingSequence.size(); ++i) {
            int tubeIndex = firingSequence[i];
            double tubeTime = std::fmod(time - i * 0.01, 0.1); // Assume 10 Hz firing frequency, 0.01s delay between tubes

            if (tubeTime >= 0 && tubeTime < 0.005) { // Assume 5 ms thrust duration
                double thrustMagnitude = std::sin(PI * tubeTime / 0.005); // Simplified thrust profile
                thrust.x += thrustMagnitude * m_tubeOrientations[tubeIndex][0];
                thrust.y += thrustMagnitude * m_tubeOrientations[tubeIndex][1];
                thrust.z += thrustMagnitude * m_tubeOrientations[tubeIndex][2];
            }
        }

        profile.push_back(thrust);
    }

    return profile;
}

void ThrustVectoringSystem::adjustForFlightConditions(const FlightConditions& conditions) {
    m_currentFlightConditions = conditions;
}

double ThrustVectoringSystem::calculateFillFraction(const PDETubeState& tubeState) const {
    // Simplified fill fraction calculation
    return tubeState.fuelMassFraction + tubeState.oxidiserMassFraction;
}

double ThrustVectoringSystem::calculateSpecificImpulse(const PDETubeState& tubeState) const {
    // Simplified Isp calculation based on Endo et al. (2004)
    double chamberTemperature = tubeState.temperature;
    double chamberPressure = tubeState.pressure;
    double expansionRatio = calculateExpansionRatio(m_currentFlightConditions.altitude);
    
    return std::sqrt(2 * 1.4 * R * chamberTemperature / (1.4 - 1) * (1 - std::pow(1 / expansionRatio, (1.4 - 1) / 1.4))) / g0;
}

double ThrustVectoringSystem::calculateThroatArea(const PDETubeState& tubeState) const {
    // Simplified throat area calculation (assuming a constant fraction of tube cross-sectional area)
    // Note: You might need to add a diameter field to PDETubeState or pass it separately
    double diameter = 0.1; // Placeholder value, replace with actual diameter
    return 0.2 * PI * std::pow(diameter / 2, 2);
}

double ThrustVectoringSystem::calculateExpansionRatio(double altitude) const {
    // Simplified expansion ratio calculation based on altitude
    return 1.0 + 0.1 * altitude / 1000.0; // Increase by 10% per km of altitude
}

double ThrustVectoringSystem::calculateAmbientPressure(double altitude) const {
    // Simplified atmospheric pressure calculation
    return 101325.0 * std::exp(-0.0001 * altitude);
}

ThrustVector ThrustVectoringSystem::rotateThrust(const ThrustVector& thrust, double pitch, double yaw) const {
    // Rotate thrust vector based on angle of attack (pitch) and sideslip angle (yaw)
    double cosPitch = std::cos(pitch);
    double sinPitch = std::sin(pitch);
    double cosYaw = std::cos(yaw);
    double sinYaw = std::sin(yaw);

    return {
        thrust.x * cosYaw * cosPitch - thrust.y * sinYaw + thrust.z * cosYaw * sinPitch,
        thrust.x * sinYaw * cosPitch + thrust.y * cosYaw + thrust.z * sinYaw * sinPitch,
        -thrust.x * sinPitch + thrust.z * cosPitch
    };
}