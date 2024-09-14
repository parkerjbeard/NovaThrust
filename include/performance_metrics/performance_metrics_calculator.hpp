#ifndef PERFORMANCE_METRICS_CALCULATOR_HPP
#define PERFORMANCE_METRICS_CALCULATOR_HPP

#include <memory>
#include <string>

// Forward declarations
class SpecificImpulseCalculator;
class ThrustCalculator;
class DetonationParametersCalculator;
class ThermalLoadEstimator;
class StructuralStressEstimator;

class PerformanceMetricsCalculator {
public:
    PerformanceMetricsCalculator();
    ~PerformanceMetricsCalculator();

    double calculateSpecificImpulse();
    double calculateThrust();
    double getDetonationFrequency() const;
    double getPeakHeatFlux() const;
    double getMaxVonMisesStress() const;
    void calculateDetonationParameters();
    void estimateThermalLoads();
    void estimateStructuralStresses();
    std::string generatePerformanceReport();

    void setSpecificImpulseParameters(double frequency, double chamberPressure, double exitPressure,
                                      double fuelMassFlowRate, double oxidizerMassFlowRate,
                                      double nozzleExpansionRatio, double detonationTemperature,
                                      double specificHeatRatio);
    void setThrustParameters(double total_exit_area,
                             double ambient_pressure,
                             double total_mass_flow_rate,
                             double exhaust_velocity,
                             double detonation_frequency,
                             double chamber_pressure,
                             double exit_pressure,
                             double nozzle_expansion_ratio);
    void setDetonationParameters(double initialPressure, double initialTemperature,
                                 double equivalenceRatio, const std::string& fuelType,
                                 const std::string& oxidizerType, double tubeLength,
                                 double tubeDiameter);
    void setThermalLoadParameters(double detonationTemperature, double ambientTemperature,
                                  double tubeLength, double tubeDiameter, double tubeWallThickness,
                                  const std::string& tubeWallMaterial, double detonationFrequency,
                                  double coolantFlowRate, const std::string& coolantType);
    void setStructuralStressParameters(double peakDetonationPressure, double ambientPressure,
                                       double tubeLength, double tubeInnerDiameter,
                                       double tubeWallThickness, const std::string& tubeWallMaterial,
                                       double detonationFrequency, double averageTemperature,
                                       double temperatureGradient);

private:
    std::unique_ptr<SpecificImpulseCalculator> m_specificImpulseCalculator;
    std::unique_ptr<ThrustCalculator> m_thrustCalculator;
    std::unique_ptr<DetonationParametersCalculator> m_detonationParametersCalculator;
    std::unique_ptr<ThermalLoadEstimator> m_thermalLoadEstimator;
    std::unique_ptr<StructuralStressEstimator> m_structuralStressEstimator;

    double m_total_exit_area;
    double m_ambient_pressure;
    double m_total_mass_flow_rate;
    double m_exhaust_velocity;
    double m_detonation_frequency;
    double m_chamber_pressure;
    double m_exit_pressure;
    double m_nozzle_expansion_ratio;
};

#endif // PERFORMANCE_METRICS_CALCULATOR_HPP
