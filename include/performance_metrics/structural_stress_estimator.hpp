#ifndef STRUCTURAL_STRESS_ESTIMATOR_HPP
#define STRUCTURAL_STRESS_ESTIMATOR_HPP

#include <vector>
#include <string>
#include <cmath>

struct StressPoint {
    double axialPosition;
    double circumferentialPosition;
    double vonMisesStress;
};

class StructuralStressEstimator {
public:
    StructuralStressEstimator();
    ~StructuralStressEstimator();

    // Main estimation method
    void estimate();

    // Setter methods for input parameters
    void setPeakDetonationPressure(double pressure);
    void setAmbientPressure(double pressure);
    void setTubeLength(double length);
    void setTubeInnerDiameter(double diameter);
    void setTubeWallThickness(double thickness);
    void setTubeWallMaterial(const std::string& material);
    void setDetonationFrequency(double frequency);
    void setAverageTemperature(double temperature);
    void setTemperatureGradient(double gradient);

    // Getter methods for results
    double getMaxHoopStress() const;
    double getMaxAxialStress() const;
    double getMaxThermalStress() const;
    double getMaxVonMisesStress() const;
    double getEstimatedFatigueLife() const;
    std::vector<StressPoint> getHighStressRegions() const;

    // Method to get a report of the estimated structural stresses
    std::string getReport() const;

private:
    // Input parameters
    double m_peakDetonationPressure; // Pa
    double m_ambientPressure;        // Pa
    double m_tubeLength;             // m
    double m_tubeInnerDiameter;      // m
    double m_tubeWallThickness;      // m
    std::string m_tubeWallMaterial;
    double m_detonationFrequency;    // Hz
    double m_averageTemperature;     // K
    double m_temperatureGradient;    // K/m

    // Calculated parameters
    double m_maxHoopStress;          // Pa
    double m_maxAxialStress;         // Pa
    double m_maxThermalStress;       // Pa
    double m_maxVonMisesStress;      // Pa
    double m_estimatedFatigueLife;   // hours
    std::vector<StressPoint> m_highStressRegions;

    // Helper methods
    double computePressureStresses();
    double computeThermalStresses();
    double estimateFatigueLife();
    std::vector<StressPoint> identifyHighStressRegions();

    // Additional helper methods
    double calculateYoungsModulus();
    double calculateThermalExpansionCoefficient();
    double calculateYieldStrength();
    double calculateUltimateTensileStrength();
};

#endif // STRUCTURAL_STRESS_ESTIMATOR_HPP