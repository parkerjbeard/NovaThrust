#include "performance_metrics/thermal_load_estimator.hpp"
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <unordered_map>

ThermalLoadEstimator::ThermalLoadEstimator()
    : m_detonationTemperature(0.0), m_ambientTemperature(0.0), m_tubeLength(0.0),
      m_tubeDiameter(0.0), m_tubeWallThickness(0.0), m_tubeWallMaterial(""),
      m_detonationFrequency(0.0), m_coolantFlowRate(0.0), m_coolantType(""),
      m_averageHeatFlux(0.0), m_peakHeatFlux(0.0), m_coolingRequirement(0.0),
      m_thermalCyclesPerHour(0)
{}

ThermalLoadEstimator::~ThermalLoadEstimator() = default;

void ThermalLoadEstimator::estimate() {
    if (m_detonationTemperature <= 0 || m_ambientTemperature <= 0 || m_tubeLength <= 0 ||
        m_tubeDiameter <= 0 || m_tubeWallThickness <= 0 || m_tubeWallMaterial.empty() ||
        m_detonationFrequency <= 0 || m_coolantFlowRate <= 0 || m_coolantType.empty()) {
        throw std::runtime_error("Invalid input parameters for thermal load estimation");
    }

    m_averageHeatFlux = computeHeatFlux();
    m_peakHeatFlux = m_averageHeatFlux * 2.5; // Assuming peak is 2.5 times the average
    m_coolingRequirement = estimateCoolingRequirements();
    m_thermalCyclesPerHour = calculateThermalCycleEffects();
    m_hotSpots = predictHotSpots();
}

void ThermalLoadEstimator::setDetonationTemperature(double temperature) { m_detonationTemperature = temperature; }
void ThermalLoadEstimator::setAmbientTemperature(double temperature) { m_ambientTemperature = temperature; }
void ThermalLoadEstimator::setTubeLength(double length) { m_tubeLength = length; }
void ThermalLoadEstimator::setTubeDiameter(double diameter) { m_tubeDiameter = diameter; }
void ThermalLoadEstimator::setTubeWallThickness(double thickness) { m_tubeWallThickness = thickness; }
void ThermalLoadEstimator::setTubeWallMaterial(const std::string& material) { m_tubeWallMaterial = material; }
void ThermalLoadEstimator::setDetonationFrequency(double frequency) { m_detonationFrequency = frequency; }
void ThermalLoadEstimator::setCoolantFlowRate(double flowRate) { m_coolantFlowRate = flowRate; }
void ThermalLoadEstimator::setCoolantType(const std::string& coolant) { m_coolantType = coolant; }

double ThermalLoadEstimator::getAverageHeatFlux() const { return m_averageHeatFlux; }
double ThermalLoadEstimator::getPeakHeatFlux() const { return m_peakHeatFlux; }
double ThermalLoadEstimator::getCoolingRequirement() const { return m_coolingRequirement; }
int ThermalLoadEstimator::getThermalCyclesPerHour() const { return m_thermalCyclesPerHour; }
std::vector<HotSpot> ThermalLoadEstimator::getHotSpots() const { return m_hotSpots; }

std::string ThermalLoadEstimator::getReport() const {
    std::ostringstream report;
    report << std::fixed << std::setprecision(2);
    report << "Thermal Load Estimation Report:\n";
    report << "Average Heat Flux: " << m_averageHeatFlux << " W/m^2\n";
    report << "Peak Heat Flux: " << m_peakHeatFlux << " W/m^2\n";
    report << "Cooling Requirement: " << m_coolingRequirement << " W\n";
    report << "Thermal Cycles per Hour: " << m_thermalCyclesPerHour << "\n";
    report << "Hot Spots:\n";
    for (const auto& spot : m_hotSpots) {
        report << "  Position: (" << spot.axialPosition << "m, " << spot.circumferentialPosition 
               << "°), Temperature: " << spot.temperature << " K\n";
    }
    return report.str();
}

double ThermalLoadEstimator::computeHeatFlux() {
    double k = calculateThermalConductivity();
    double h = calculateConvectionCoefficient();
    double A = M_PI * m_tubeDiameter * m_tubeLength;
    
    // Simplified heat transfer model
    double q = (m_detonationTemperature - m_ambientTemperature) / 
               (1 / (h * A) + m_tubeWallThickness / (k * A));
    
    return q / A;
}

double ThermalLoadEstimator::estimateCoolingRequirements() {
    // Assuming all heat needs to be removed by the cooling system
    return m_averageHeatFlux * (M_PI * m_tubeDiameter * m_tubeLength);
}

int ThermalLoadEstimator::calculateThermalCycleEffects() {
    // Simplified model: each detonation causes one thermal cycle
    return static_cast<int>(m_detonationFrequency * 3600);
}

std::vector<HotSpot> ThermalLoadEstimator::predictHotSpots() {
    std::vector<HotSpot> hotSpots;
    
    // Simplified model: assuming hot spots at the beginning, middle, and end of the tube
    hotSpots.push_back({0.1 * m_tubeLength, 0, m_detonationTemperature * 0.9});
    hotSpots.push_back({0.5 * m_tubeLength, 180, m_detonationTemperature * 0.85});
    hotSpots.push_back({0.9 * m_tubeLength, 90, m_detonationTemperature * 0.8});
    
    return hotSpots;
}

double ThermalLoadEstimator::calculateThermalConductivity() {
    // Simplified thermal conductivity model (W/m-K)
    std::unordered_map<std::string, double> materialConductivity = {
        {"steel", 50.2},
        {"inconel", 11.4},
        {"titanium", 21.9}
    };
    
    return materialConductivity.count(m_tubeWallMaterial) ? 
           materialConductivity[m_tubeWallMaterial] : 50.2; // Default to steel
}

double ThermalLoadEstimator::calculateConvectionCoefficient() {
    // Simplified convection coefficient model (W/m^2-K)
    std::unordered_map<std::string, double> coolantCoefficient = {
        {"water", 10000},
        {"oil", 1000},
        {"air", 100}
    };
    
    return coolantCoefficient.count(m_coolantType) ? 
           coolantCoefficient[m_coolantType] : 100; // Default to air cooling
}

double ThermalLoadEstimator::calculateRadiationHeatTransfer(double surfaceTemp) {
    const double STEFAN_BOLTZMANN = 5.67e-8; // W/m^2-K^4
    double emissivity = 0.8; // Assuming a high emissivity for most materials
    
    return emissivity * STEFAN_BOLTZMANN * 
           (std::pow(surfaceTemp, 4) - std::pow(m_ambientTemperature, 4));
}