#include "performance_metrics/performance_metrics_calculator.hpp"
#include "performance_metrics/specific_impulse_calculator.hpp"
#include "performance_metrics/thrust_calculator.hpp"
#include "performance_metrics/detonation_parameters_calculator.hpp"
#include "performance_metrics/thermal_load_estimator.hpp"
#include "performance_metrics/structural_stress_estimator.hpp"
#include <sstream>
#include <iomanip>

PerformanceMetricsCalculator::PerformanceMetricsCalculator()
    : m_specificImpulseCalculator(std::make_unique<SpecificImpulseCalculator>()),
      m_thrustCalculator(std::make_unique<ThrustCalculator>()),
      m_detonationParametersCalculator(std::make_unique<DetonationParametersCalculator>()),
      m_thermalLoadEstimator(std::make_unique<ThermalLoadEstimator>()),
      m_structuralStressEstimator(std::make_unique<StructuralStressEstimator>())
{}

PerformanceMetricsCalculator::~PerformanceMetricsCalculator() = default;

double PerformanceMetricsCalculator::calculateSpecificImpulse() {
    return m_specificImpulseCalculator->calculate();
}

double PerformanceMetricsCalculator::calculateThrust() {
    return m_thrustCalculator->calculate();
}

void PerformanceMetricsCalculator::calculateDetonationParameters() {
    m_detonationParametersCalculator->calculate();
}

void PerformanceMetricsCalculator::estimateThermalLoads() {
    m_thermalLoadEstimator->estimate();
}

void PerformanceMetricsCalculator::estimateStructuralStresses() {
    m_structuralStressEstimator->estimate();
}

std::string PerformanceMetricsCalculator::generatePerformanceReport() {
    std::ostringstream report;
    report << std::fixed << std::setprecision(2);

    report << "PDE Performance Report\n";
    report << "======================\n\n";

    report << "Specific Impulse: " << calculateSpecificImpulse() << " s\n";
    report << "Thrust: " << calculateThrust() << " N\n\n";

    calculateDetonationParameters();
    report << "Detonation Parameters:\n";
    report << m_detonationParametersCalculator->getReport() << "\n";

    estimateThermalLoads();
    report << "Thermal Loads:\n";
    report << m_thermalLoadEstimator->getReport() << "\n";

    estimateStructuralStresses();
    report << "Structural Stresses:\n";
    report << m_structuralStressEstimator->getReport() << "\n";

    return report.str();
}

void PerformanceMetricsCalculator::setSpecificImpulseParameters(double frequency, double chamberPressure,
    double exitPressure, double fuelMassFlowRate, double oxidizerMassFlowRate,
    double nozzleExpansionRatio, double detonationTemperature, double specificHeatRatio) {
    m_specificImpulseCalculator->setDetonationFrequency(frequency);
    m_specificImpulseCalculator->setChamberPressure(chamberPressure);
    m_specificImpulseCalculator->setExitPressure(exitPressure);
    m_specificImpulseCalculator->setFuelMassFlowRate(fuelMassFlowRate);
    m_specificImpulseCalculator->setOxidizerMassFlowRate(oxidizerMassFlowRate);
    m_specificImpulseCalculator->setNozzleExpansionRatio(nozzleExpansionRatio);
    m_specificImpulseCalculator->setDetonationTemperature(detonationTemperature);
    m_specificImpulseCalculator->setSpecificHeatRatio(specificHeatRatio);
}

void PerformanceMetricsCalculator::setThrustParameters(double exitArea, double ambientPressure,
    double massFlowRate, double exhaustVelocity, double detonationFrequency,
    double chamberPressure, double exitPressure, double nozzleExpansionRatio) {
    m_thrustCalculator->setExitArea(exitArea);
    m_thrustCalculator->setAmbientPressure(ambientPressure);
    m_thrustCalculator->setMassFlowRate(massFlowRate);
    m_thrustCalculator->setExhaustVelocity(exhaustVelocity);
    m_thrustCalculator->setDetonationFrequency(detonationFrequency);
    m_thrustCalculator->setChamberPressure(chamberPressure);
    m_thrustCalculator->setExitPressure(exitPressure);
    m_thrustCalculator->setNozzleExpansionRatio(nozzleExpansionRatio);
}

void PerformanceMetricsCalculator::setDetonationParameters(double initialPressure, double initialTemperature,
    double equivalenceRatio, const std::string& fuelType, const std::string& oxidizerType,
    double tubeLength, double tubeDiameter) {
    m_detonationParametersCalculator->setInitialPressure(initialPressure);
    m_detonationParametersCalculator->setInitialTemperature(initialTemperature);
    m_detonationParametersCalculator->setEquivalenceRatio(equivalenceRatio);
    m_detonationParametersCalculator->setFuelType(fuelType);
    m_detonationParametersCalculator->setOxidizerType(oxidizerType);
    m_detonationParametersCalculator->setTubeLength(tubeLength);
    m_detonationParametersCalculator->setTubeDiameter(tubeDiameter);
}

void PerformanceMetricsCalculator::setThermalLoadParameters(double detonationTemperature, double ambientTemperature,
    double tubeLength, double tubeDiameter, double tubeWallThickness, const std::string& tubeWallMaterial,
    double detonationFrequency, double coolantFlowRate, const std::string& coolantType) {
    m_thermalLoadEstimator->setDetonationTemperature(detonationTemperature);
    m_thermalLoadEstimator->setAmbientTemperature(ambientTemperature);
    m_thermalLoadEstimator->setTubeLength(tubeLength);
    m_thermalLoadEstimator->setTubeDiameter(tubeDiameter);
    m_thermalLoadEstimator->setTubeWallThickness(tubeWallThickness);
    m_thermalLoadEstimator->setTubeWallMaterial(tubeWallMaterial);
    m_thermalLoadEstimator->setDetonationFrequency(detonationFrequency);
    m_thermalLoadEstimator->setCoolantFlowRate(coolantFlowRate);
    m_thermalLoadEstimator->setCoolantType(coolantType);
}

void PerformanceMetricsCalculator::setStructuralStressParameters(double peakDetonationPressure, double ambientPressure,
    double tubeLength, double tubeInnerDiameter, double tubeWallThickness, const std::string& tubeWallMaterial,
    double detonationFrequency, double averageTemperature, double temperatureGradient) {
    m_structuralStressEstimator->setPeakDetonationPressure(peakDetonationPressure);
    m_structuralStressEstimator->setAmbientPressure(ambientPressure);
    m_structuralStressEstimator->setTubeLength(tubeLength);
    m_structuralStressEstimator->setTubeInnerDiameter(tubeInnerDiameter);
    m_structuralStressEstimator->setTubeWallThickness(tubeWallThickness);
    m_structuralStressEstimator->setTubeWallMaterial(tubeWallMaterial);
    m_structuralStressEstimator->setDetonationFrequency(detonationFrequency);
    m_structuralStressEstimator->setAverageTemperature(averageTemperature);
    m_structuralStressEstimator->setTemperatureGradient(temperatureGradient);
}