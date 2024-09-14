#ifndef THERMAL_LOAD_ESTIMATOR_HPP
#define THERMAL_LOAD_ESTIMATOR_HPP

#include <vector>
#include <string>
#include <cmath>

struct HotSpot {
    double axialPosition;
    double circumferentialPosition;
    double temperature;
};

class ThermalLoadEstimator {
public:
    ThermalLoadEstimator();
    ~ThermalLoadEstimator();

    // Main estimation method
    void estimate();

    // Setter methods for input parameters
    void setDetonationTemperature(double temperature);
    void setAmbientTemperature(double temperature);
    void setTubeLength(double length);
    void setTubeDiameter(double diameter);
    void setTubeWallThickness(double thickness);
    void setTubeWallMaterial(const std::string& material);
    void setDetonationFrequency(double frequency);
    void setCoolantFlowRate(double flowRate);
    void setCoolantType(const std::string& coolant);

    // Getter methods for results
    double getAverageHeatFlux() const;
    double getPeakHeatFlux() const;
    double getCoolingRequirement() const;
    int getThermalCyclesPerHour() const;
    std::vector<HotSpot> getHotSpots() const;

    // Method to get a report of the estimated thermal loads
    std::string getReport() const;

private:
    // Input parameters
    double m_detonationTemperature; // K
    double m_ambientTemperature;    // K
    double m_tubeLength;            // m
    double m_tubeDiameter;          // m
    double m_tubeWallThickness;     // m
    std::string m_tubeWallMaterial;
    double m_detonationFrequency;   // Hz
    double m_coolantFlowRate;       // kg/s
    std::string m_coolantType;

    // Calculated parameters
    double m_averageHeatFlux;       // W/m^2
    double m_peakHeatFlux;          // W/m^2
    double m_coolingRequirement;    // W
    int m_thermalCyclesPerHour;
    std::vector<HotSpot> m_hotSpots;

    // Helper methods
    double computeHeatFlux();
    double estimateCoolingRequirements();
    int calculateThermalCycleEffects();
    std::vector<HotSpot> predictHotSpots();

    // Additional helper methods
    double calculateThermalConductivity();
    double calculateConvectionCoefficient();
    double calculateRadiationHeatTransfer(double surfaceTemp);
};

#endif // THERMAL_LOAD_ESTIMATOR_HPP