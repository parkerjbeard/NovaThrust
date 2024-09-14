#include "performance_metrics/specific_impulse_calculator.hpp"
#include <stdexcept>

SpecificImpulseCalculator::SpecificImpulseCalculator()
    : m_detonationFrequency(0.0), m_chamberPressure(0.0), m_exitPressure(0.0),
      m_fuelMassFlowRate(0.0), m_oxidizerMassFlowRate(0.0), m_nozzleExpansionRatio(0.0),
      m_detonationTemperature(0.0), m_specificHeatRatio(0.0)
{}

SpecificImpulseCalculator::~SpecificImpulseCalculator() = default;

double SpecificImpulseCalculator::calculate() {
    if (m_detonationFrequency <= 0 || m_chamberPressure <= 0 || m_exitPressure <= 0 ||
        m_fuelMassFlowRate <= 0 || m_oxidizerMassFlowRate <= 0 || m_nozzleExpansionRatio <= 1 ||
        m_detonationTemperature <= 0 || m_specificHeatRatio <= 1) {
        throw std::runtime_error("Invalid input parameters for Isp calculation");
    }

    return calculateTimeAveragedIsp();
}

void SpecificImpulseCalculator::setDetonationFrequency(double frequency) {
    m_detonationFrequency = frequency;
}

void SpecificImpulseCalculator::setChamberPressure(double pressure) {
    m_chamberPressure = pressure;
}

void SpecificImpulseCalculator::setExitPressure(double pressure) {
    m_exitPressure = pressure;
}

void SpecificImpulseCalculator::setFuelMassFlowRate(double rate) {
    m_fuelMassFlowRate = rate;
}

void SpecificImpulseCalculator::setOxidizerMassFlowRate(double rate) {
    m_oxidizerMassFlowRate = rate;
}

void SpecificImpulseCalculator::setNozzleExpansionRatio(double ratio) {
    m_nozzleExpansionRatio = ratio;
}

void SpecificImpulseCalculator::setDetonationTemperature(double temperature) {
    m_detonationTemperature = temperature;
}

void SpecificImpulseCalculator::setSpecificHeatRatio(double ratio) {
    m_specificHeatRatio = ratio;
}

double SpecificImpulseCalculator::computeMassFlowRate() const {
    return m_fuelMassFlowRate + m_oxidizerMassFlowRate;
}

double SpecificImpulseCalculator::computeExhaustVelocity() const {
    double characteristicVelocity = computeCharacteristicVelocity();
    double thrustCoefficient = computeThrustCoefficient();
    return characteristicVelocity * thrustCoefficient;
}

double SpecificImpulseCalculator::calculateTimeAveragedIsp() const {
    double exhaustVelocity = computeExhaustVelocity();
    return exhaustVelocity / GRAVITATIONAL_ACCELERATION;
}

double SpecificImpulseCalculator::calculatePulseIsp(double pulseThrust, double pulseDuration) const {
    double totalImpulse = pulseThrust * pulseDuration;
    double propellantMassPerPulse = computeMassFlowRate() / m_detonationFrequency;
    return totalImpulse / (propellantMassPerPulse * GRAVITATIONAL_ACCELERATION);
}

double SpecificImpulseCalculator::computeCharacteristicVelocity() const {
    double R = 8314.46 / 29.0; // Assuming air as oxidizer, adjust if needed
    return std::sqrt((m_specificHeatRatio * R * m_detonationTemperature) /
                     (m_specificHeatRatio - 1));
}

double SpecificImpulseCalculator::computeThrustCoefficient() const {
    double pressureRatio = m_exitPressure / m_chamberPressure;
    double term1 = (2 * m_specificHeatRatio * m_specificHeatRatio) / (m_specificHeatRatio - 1);
    double term2 = 2 / (m_specificHeatRatio + 1);
    double term3 = std::pow(term2, (m_specificHeatRatio + 1) / (m_specificHeatRatio - 1));
    double term4 = 1 - std::pow(pressureRatio, (m_specificHeatRatio - 1) / m_specificHeatRatio);
    
    return std::sqrt(term1 * term3 * term4) + 
           (m_nozzleExpansionRatio * (pressureRatio - m_exitPressure / m_chamberPressure));
}