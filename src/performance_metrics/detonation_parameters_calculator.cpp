#include "performance_metrics/detonation_parameters_calculator.hpp"
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <unordered_map>

DetonationParametersCalculator::DetonationParametersCalculator()
    : m_initialPressure(0.0), m_initialTemperature(0.0), m_equivalenceRatio(0.0),
      m_fuelType(""), m_oxidizer(""), m_tubeLength(0.0), m_tubeDiameter(0.0),
      m_waveSpeed(0.0), m_peakPressure(0.0), m_cellSize(0.0), m_detonationFrequency(0.0)
{}

DetonationParametersCalculator::~DetonationParametersCalculator() = default;

void DetonationParametersCalculator::calculate() {
    if (m_initialPressure <= 0 || m_initialTemperature <= 0 || m_equivalenceRatio <= 0 ||
        m_fuelType.empty() || m_oxidizer.empty() || m_tubeLength <= 0 || m_tubeDiameter <= 0) {
        throw std::runtime_error("Invalid input parameters for detonation calculation");
    }

    m_waveSpeed = calculateWaveSpeed();
    m_peakPressure = calculatePeakPressure();
    m_cellSize = calculateCellSize();
    m_detonationFrequency = calculateDetonationFrequency();
}

double DetonationParametersCalculator::getWaveSpeed() const { return m_waveSpeed; }
double DetonationParametersCalculator::getPeakPressure() const { return m_peakPressure; }
double DetonationParametersCalculator::getCellSize() const { return m_cellSize; }
double DetonationParametersCalculator::getDetonationFrequency() const { return m_detonationFrequency; }

void DetonationParametersCalculator::setInitialPressure(double pressure) { m_initialPressure = pressure; }
void DetonationParametersCalculator::setInitialTemperature(double temperature) { m_initialTemperature = temperature; }
void DetonationParametersCalculator::setEquivalenceRatio(double ratio) { m_equivalenceRatio = ratio; }
void DetonationParametersCalculator::setFuelType(const std::string& fuel) { m_fuelType = fuel; }
void DetonationParametersCalculator::setOxidizerType(const std::string& oxidizer) { m_oxidizer = oxidizer; }
void DetonationParametersCalculator::setTubeLength(double length) { m_tubeLength = length; }
void DetonationParametersCalculator::setTubeDiameter(double diameter) { m_tubeDiameter = diameter; }

std::string DetonationParametersCalculator::getReport() const {
    std::ostringstream report;
    report << std::fixed << std::setprecision(2);
    report << "Detonation Wave Speed: " << m_waveSpeed << " m/s\n";
    report << "Peak Pressure: " << m_peakPressure / 1e6 << " MPa\n";
    report << "Detonation Cell Size: " << m_cellSize * 1000 << " mm\n";
    report << "Detonation Frequency: " << m_detonationFrequency << " Hz\n";
    return report.str();
}

double DetonationParametersCalculator::calculateWaveSpeed() {
    double gamma = calculateSpecificHeatRatio();
    double Q = calculateHeatOfCombustion();
    double R = 8314.46 / 29.0; // Assuming air as oxidizer, adjust if needed

    // Using the ZND detonation model
    double MCJ = calculateChapmanJouguetMachNumber();
    return MCJ * std::sqrt(gamma * R * m_initialTemperature);
}

double DetonationParametersCalculator::calculatePeakPressure() {
    double gamma = calculateSpecificHeatRatio();
    double MCJ = calculateChapmanJouguetMachNumber();

    // Using the ZND detonation model
    return m_initialPressure * (1 + gamma * (MCJ * MCJ - 1));
}

double DetonationParametersCalculator::calculateCellSize() {
    // This is a simplified model. For accurate results, experimental data or detailed simulations are needed.
    double lambda = 50.0; // Characteristic length scale, depends on fuel-oxidizer mixture

    if (m_fuelType == "hydrogen" && m_oxidizer == "air") {
        lambda = 15.0;
    } else if (m_fuelType == "methane" && m_oxidizer == "air") {
        lambda = 30.0;
    }

    return lambda * m_initialPressure / 101325.0; // Scaling with initial pressure
}

double DetonationParametersCalculator::calculateDetonationFrequency() {
    // Simplified model assuming one detonation cycle per tube length
    return m_waveSpeed / (2 * m_tubeLength);
}

double DetonationParametersCalculator::calculateSpecificHeatRatio() {
    // This is a simplified model. For accurate results, use a thermodynamic library or detailed calculations.
    std::unordered_map<std::string, double> fuelGammas = {
        {"hydrogen", 1.41},
        {"methane", 1.32},
        {"propane", 1.13}
    };

    return fuelGammas.count(m_fuelType) ? fuelGammas[m_fuelType] : 1.4; // Default to air if fuel not found
}

double DetonationParametersCalculator::calculateHeatOfCombustion() {
    // Heat of combustion in J/kg. This is a simplified model.
    std::unordered_map<std::string, double> fuelHeats = {
        {"hydrogen", 120e6},
        {"methane", 50e6},
        {"propane", 46.35e6}
    };

    return fuelHeats.count(m_fuelType) ? fuelHeats[m_fuelType] : 0.0;
}

double DetonationParametersCalculator::calculateChapmanJouguetMachNumber() {
    double gamma = calculateSpecificHeatRatio();
    double Q = calculateHeatOfCombustion();
    double R = 8314.46 / 29.0; // Assuming air as oxidizer, adjust if needed

    // Using the CJ condition equation
    double term = 1 + (gamma * gamma - 1) * Q / (2 * gamma * R * m_initialTemperature);
    return std::sqrt(term + std::sqrt(term * term - 1));
}