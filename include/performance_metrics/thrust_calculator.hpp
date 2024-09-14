#ifndef THRUST_CALCULATOR_HPP
#define THRUST_CALCULATOR_HPP

#include <vector>
#include <cmath>

class ThrustCalculator {
public:
    ThrustCalculator();
    virtual ~ThrustCalculator();

    double calculate();

    // Main calculation methods
    double calculateInstantaneousThrust(double time);
    double calculateAverageThrust(double startTime, double endTime);

    // Setter methods for input parameters
    void setExitArea(double area);
    void setAmbientPressure(double pressure);
    void setMassFlowRate(double rate);
    void setExhaustVelocity(double velocity);
    void setDetonationFrequency(double frequency);
    void setChamberPressure(double pressure);
    void setExitPressure(double pressure);
    void setNozzleExpansionRatio(double ratio);

protected:
    // Helper methods (moved to protected)
    virtual double computePressureForce() const;
    virtual double computeMomentumFlux() const;
    virtual double computeThrustCoefficient() const;
    virtual double computePulseDuration() const;
    virtual double computePeakThrust() const;
    virtual double computeThrustProfile(double time) const;

private:
    // Input parameters
    double m_exitArea;             // m^2
    double m_ambientPressure;      // Pa
    double m_massFlowRate;         // kg/s
    double m_exhaustVelocity;      // m/s
    double m_detonationFrequency;  // Hz
    double m_chamberPressure;      // Pa
    double m_exitPressure;         // Pa
    double m_nozzleExpansionRatio;
};

#endif // THRUST_CALCULATOR_HPP