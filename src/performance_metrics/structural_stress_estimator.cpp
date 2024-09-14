#include "performance_metrics/structural_stress_estimator.hpp"
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <unordered_map>

StructuralStressEstimator::StructuralStressEstimator()
    : m_peakDetonationPressure(0.0), m_ambientPressure(0.0), m_tubeLength(0.0),
      m_tubeInnerDiameter(0.0), m_tubeWallThickness(0.0), m_tubeWallMaterial(""),
      m_detonationFrequency(0.0), m_averageTemperature(0.0), m_temperatureGradient(0.0),
      m_maxHoopStress(0.0), m_maxAxialStress(0.0), m_maxThermalStress(0.0),
      m_maxVonMisesStress(0.0), m_estimatedFatigueLife(0.0)
{}

StructuralStressEstimator::~StructuralStressEstimator() = default;

void StructuralStressEstimator::estimate() {
    if (m_peakDetonationPressure <= 0 || m_ambientPressure <= 0 || m_tubeLength <= 0 ||
        m_tubeInnerDiameter <= 0 || m_tubeWallThickness <= 0 || m_tubeWallMaterial.empty() ||
        m_detonationFrequency <= 0 || m_averageTemperature <= 0) {
        throw std::runtime_error("Invalid input parameters for structural stress estimation");
    }

    m_maxHoopStress = computePressureStresses();
    m_maxAxialStress = m_maxHoopStress / 2.0; // Simplified assumption for thin-walled cylinders
    m_maxThermalStress = computeThermalStresses();
    m_maxVonMisesStress = std::sqrt(m_maxHoopStress*m_maxHoopStress - 
                                    m_maxHoopStress*m_maxAxialStress + 
                                    m_maxAxialStress*m_maxAxialStress + 
                                    3*m_maxThermalStress*m_maxThermalStress);
    m_estimatedFatigueLife = estimateFatigueLife();
    m_highStressRegions = identifyHighStressRegions();
}

void StructuralStressEstimator::setPeakDetonationPressure(double pressure) { m_peakDetonationPressure = pressure; }
void StructuralStressEstimator::setAmbientPressure(double pressure) { m_ambientPressure = pressure; }
void StructuralStressEstimator::setTubeLength(double length) { m_tubeLength = length; }
void StructuralStressEstimator::setTubeInnerDiameter(double diameter) { m_tubeInnerDiameter = diameter; }
void StructuralStressEstimator::setTubeWallThickness(double thickness) { m_tubeWallThickness = thickness; }
void StructuralStressEstimator::setTubeWallMaterial(const std::string& material) { m_tubeWallMaterial = material; }
void StructuralStressEstimator::setDetonationFrequency(double frequency) { m_detonationFrequency = frequency; }
void StructuralStressEstimator::setAverageTemperature(double temperature) { m_averageTemperature = temperature; }
void StructuralStressEstimator::setTemperatureGradient(double gradient) { m_temperatureGradient = gradient; }

double StructuralStressEstimator::getMaxHoopStress() const { return m_maxHoopStress; }
double StructuralStressEstimator::getMaxAxialStress() const { return m_maxAxialStress; }
double StructuralStressEstimator::getMaxThermalStress() const { return m_maxThermalStress; }
double StructuralStressEstimator::getMaxVonMisesStress() const { return m_maxVonMisesStress; }
double StructuralStressEstimator::getEstimatedFatigueLife() const { return m_estimatedFatigueLife; }
std::vector<StressPoint> StructuralStressEstimator::getHighStressRegions() const { return m_highStressRegions; }

std::string StructuralStressEstimator::getReport() const {
    std::ostringstream report;
    report << std::fixed << std::setprecision(2);
    report << "Structural Stress Estimation Report:\n";
    report << "Max Hoop Stress: " << m_maxHoopStress / 1e6 << " MPa\n";
    report << "Max Axial Stress: " << m_maxAxialStress / 1e6 << " MPa\n";
    report << "Max Thermal Stress: " << m_maxThermalStress / 1e6 << " MPa\n";
    report << "Max Von Mises Stress: " << m_maxVonMisesStress / 1e6 << " MPa\n";
    report << "Estimated Fatigue Life: " << m_estimatedFatigueLife << " hours\n";
    report << "High Stress Regions:\n";
    for (const auto& point : m_highStressRegions) {
        report << "  Position: (" << point.axialPosition << "m, " << point.circumferentialPosition 
               << "°), Von Mises Stress: " << point.vonMisesStress / 1e6 << " MPa\n";
    }
    return report.str();
}

double StructuralStressEstimator::computePressureStresses() {
    // Using thin-walled cylinder approximation for hoop stress
    return (m_peakDetonationPressure - m_ambientPressure) * m_tubeInnerDiameter / (2 * m_tubeWallThickness);
}

double StructuralStressEstimator::computeThermalStresses() {
    double E = calculateYoungsModulus();
    double alpha = calculateThermalExpansionCoefficient();
    double deltaT = m_temperatureGradient * m_tubeWallThickness;
    
    // Simplified thermal stress calculation
    return E * alpha * deltaT / (2 * (1 - 0.3)); // 0.3 is an approximation for Poisson's ratio
}

double StructuralStressEstimator::estimateFatigueLife() {
    double UTS = calculateUltimateTensileStrength();
    double stressAmplitude = m_maxVonMisesStress / 2; // Assuming fully reversed loading
    
    // Simplified S-N curve approximation
    double N = std::pow(10, 6) * std::pow(UTS / stressAmplitude, 3);
    
    return N / (m_detonationFrequency * 3600); // Convert cycles to hours
}

std::vector<StressPoint> StructuralStressEstimator::identifyHighStressRegions() {
    std::vector<StressPoint> highStressRegions;
    
    // Simplified model: assuming high stress points at the beginning, middle, and end of the tube
    highStressRegions.push_back({0.1 * m_tubeLength, 0, m_maxVonMisesStress * 0.9});
    highStressRegions.push_back({0.5 * m_tubeLength, 180, m_maxVonMisesStress * 0.95});
    highStressRegions.push_back({0.9 * m_tubeLength, 90, m_maxVonMisesStress});
    
    return highStressRegions;
}

double StructuralStressEstimator::calculateYoungsModulus() {
    // Simplified Young's modulus model (GPa)
    std::unordered_map<std::string, double> materialYoungsModulus = {
        {"steel", 200},
        {"inconel", 214},
        {"titanium", 114}
    };
    
    return materialYoungsModulus.count(m_tubeWallMaterial) ? 
           materialYoungsModulus[m_tubeWallMaterial] * 1e9 : 200e9; // Default to steel
}

double StructuralStressEstimator::calculateThermalExpansionCoefficient() {
    // Simplified thermal expansion coefficient model (1/K)
    std::unordered_map<std::string, double> materialThermalExpansion = {
        {"steel", 13e-6},
        {"inconel", 11.5e-6},
        {"titanium", 8.6e-6}
    };
    
    return materialThermalExpansion.count(m_tubeWallMaterial) ? 
           materialThermalExpansion[m_tubeWallMaterial] : 13e-6; // Default to steel
}

double StructuralStressEstimator::calculateYieldStrength() {
    // Simplified yield strength model (MPa)
    std::unordered_map<std::string, double> materialYieldStrength = {
        {"steel", 250},
        {"inconel", 460},
        {"titanium", 880}
    };
    
    return materialYieldStrength.count(m_tubeWallMaterial) ? 
           materialYieldStrength[m_tubeWallMaterial] * 1e6 : 250e6; // Default to steel
}

double StructuralStressEstimator::calculateUltimateTensileStrength() {
    // Simplified ultimate tensile strength model (MPa)
    std::unordered_map<std::string, double> materialUTS = {
        {"steel", 400},
        {"inconel", 785},
        {"titanium", 950}
    };
    
    return materialUTS.count(m_tubeWallMaterial) ? 
           materialUTS[m_tubeWallMaterial] * 1e6 : 400e6; // Default to steel
}