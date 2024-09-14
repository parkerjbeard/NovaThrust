#ifndef SPECIFIC_IMPULSE_CALCULATOR_HPP
#define SPECIFIC_IMPULSE_CALCULATOR_HPP

#include <vector>
#include <cmath>

class SpecificImpulseCalculator {
public:
    SpecificImpulseCalculator();
    ~SpecificImpulseCalculator();

    // Main calculation method
    double calculate();

    // Setter methods for input parameters
    void setDetonationFrequency(double frequency);
    void setChamberPressure(double pressure);
    void setExitPressure(double pressure);
    void setFuelMassFlowRate(double rate);
    void setOxidizerMassFlowRate(double rate);
    void setNozzleExpansionRatio(double ratio);
    void setDetonationTemperature(double temperature);
    void setSpecificHeatRatio(double ratio);

private:
    // Input parameters
    double m_detonationFrequency;  // Hz
    double m_chamberPressure;      // Pa
    double m_exitPressure;         // Pa
    double m_fuelMassFlowRate;     // kg/s
    double m_oxidizerMassFlowRate; // kg/s
    double m_nozzleExpansionRatio;
    double m_detonationTemperature; // K
    double m_specificHeatRatio;

    // Constants
    const double GRAVITATIONAL_ACCELERATION = 9.80665; // m/s^2

    // Helper methods
    double computeMassFlowRate() const;
    double computeExhaustVelocity() const;
    double calculateTimeAveragedIsp() const;
    double calculatePulseIsp(double pulseThrust, double pulseDuration) const;
    double computeCharacteristicVelocity() const;
    double computeThrustCoefficient() const;
};

#endif // SPECIFIC_IMPULSE_CALCULATOR_HPP