#include "performance_metrics/thrust_calculator.hpp"
#include <stdexcept>

ThrustCalculator::ThrustCalculator()
    : m_exitArea(0.0), m_ambientPressure(0.0), m_massFlowRate(0.0), m_exhaustVelocity(0.0),
      m_detonationFrequency(0.0), m_chamberPressure(0.0), m_exitPressure(0.0), m_nozzleExpansionRatio(0.0)
{}

ThrustCalculator::~ThrustCalculator() = default;

double ThrustCalculator::calculateInstantaneousThrust(double time) {
    if (m_exitArea <= 0 || m_ambientPressure <= 0 || m_massFlowRate <= 0 || m_exhaustVelocity <= 0 ||
        m_detonationFrequency <= 0 || m_chamberPressure <= 0 || m_exitPressure <= 0 || m_nozzleExpansionRatio <= 1) {
        throw std::runtime_error("Invalid input parameters for thrust calculation");
    }

    return computeThrustProfile(time);
}

double ThrustCalculator::calculateAverageThrust(double startTime, double endTime) {
    if (startTime >= endTime) {
        throw std::runtime_error("Invalid time range for average thrust calculation");
    }

    const int numSamples = 1000;
    double totalThrust = 0.0;
    double timeStep = (endTime - startTime) / numSamples;

    for (int i = 0; i < numSamples; ++i) {
        double time = startTime + i * timeStep;
        totalThrust += calculateInstantaneousThrust(time);
    }

    return totalThrust / numSamples;
}

void ThrustCalculator::setExitArea(double area) {
    m_exitArea = area;
}

void ThrustCalculator::setAmbientPressure(double pressure) {
    m_ambientPressure = pressure;
}

void ThrustCalculator::setMassFlowRate(double rate) {
    m_massFlowRate = rate;
}

void ThrustCalculator::setExhaustVelocity(double velocity) {
    m_exhaustVelocity = velocity;
}

void ThrustCalculator::setDetonationFrequency(double frequency) {
    m_detonationFrequency = frequency;
}

void ThrustCalculator::setChamberPressure(double pressure) {
    m_chamberPressure = pressure;
}

void ThrustCalculator::setExitPressure(double pressure) {
    m_exitPressure = pressure;
}

void ThrustCalculator::setNozzleExpansionRatio(double ratio) {
    m_nozzleExpansionRatio = ratio;
}

double ThrustCalculator::computePressureForce() const {
    return m_exitArea * (m_exitPressure - m_ambientPressure);
}

double ThrustCalculator::computeMomentumFlux() const {
    return m_massFlowRate * m_exhaustVelocity;
}

double ThrustCalculator::computeThrustCoefficient() const {
    double pressureRatio = m_exitPressure / m_chamberPressure;
    double gamma = 1.4; // Assuming air as the working fluid. Adjust if needed.

    double term1 = (2 * gamma * gamma) / (gamma - 1);
    double term2 = (2 / (gamma + 1)) * std::pow((gamma + 1) / 2, gamma / (gamma - 1));
    double term3 = 1 - std::pow(pressureRatio, (gamma - 1) / gamma);
    
    return std::sqrt(term1 * term2 * term3) + 
           (m_nozzleExpansionRatio * (pressureRatio - m_ambientPressure / m_chamberPressure));
}

double ThrustCalculator::computePulseDuration() const {
    return 1.0 / m_detonationFrequency;
}

double ThrustCalculator::computePeakThrust() const {
    return computePressureForce() + computeMomentumFlux();
}

double ThrustCalculator::computeThrustProfile(double time) const {
    double pulseDuration = computePulseDuration();
    double peakThrust = computePeakThrust();
    double normalizedTime = std::fmod(time, pulseDuration) / pulseDuration;

    // Simplified thrust profile model: rapid rise, exponential decay
    if (normalizedTime < 0.1) {
        // Rapid rise (linear approximation)
        return peakThrust * (normalizedTime / 0.1);
    } else {
        // Exponential decay
        return peakThrust * std::exp(-5 * (normalizedTime - 0.1));
    }
}

// Implement the calculate() method
double ThrustCalculator::calculate() {
    // Example implementation: Calculate peak thrust
    return computePeakThrust();
}