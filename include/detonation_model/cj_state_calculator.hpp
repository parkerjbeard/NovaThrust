#ifndef CJ_STATE_CALCULATOR_HPP
#define CJ_STATE_CALCULATOR_HPP

#include <vector>
#include <functional>

class CJStateCalculator {
public:
    CJStateCalculator();
    ~CJStateCalculator();

    // Main calculation methods
    void calculateCJState(double initialPressure, double initialTemperature, double initialDensity);
    
    // Setter methods
    void setEquationOfState(const std::function<double(double, double)>& eos);
    void setSpecificHeatRatio(double gamma);
    void setHeatOfReaction(double q);

    // Getter methods for CJ state properties
    double getCJPressure() const;
    double getCJTemperature() const;
    double getCJDensity() const;
    double getCJVelocity() const;

    // Additional utility methods
    double calculateDetonationMachNumber() const;
    double calculateVonNeumannPressure() const;

private:
    // Private member variables
    double m_initialPressure;
    double m_initialTemperature;
    double m_initialDensity;
    double m_gamma;
    double m_q;  // Heat of reaction

    double m_cjPressure;
    double m_cjTemperature;
    double m_cjDensity;
    double m_cjVelocity;

    std::function<double(double, double)> m_equationOfState;

    // Private helper methods
    double findCJPoint();
    double calculateCJPressureRatio(double pressureRatio) const;
    double calculateCJTemperature(double pressureRatio) const;
    double calculateCJDensity(double pressureRatio) const;
    double calculateCJVelocity(double pressureRatio) const;
};

// Inline method implementations with formulas

inline double CJStateCalculator::calculateCJPressureRatio(double pressureRatio) const {
    // CJ condition: u^2 = (dp/dρ)_s (Rayleigh line tangent to Hugoniot curve)
    // For ideal gas EoS, this leads to the following equation:
    return (pressureRatio - 1) * (1 - m_gamma + 2 * m_gamma * pressureRatio) 
           - m_gamma * m_q / (m_initialTemperature * (m_gamma * m_gamma - 1));
}

inline double CJStateCalculator::calculateCJTemperature(double pressureRatio) const {
    // CJ temperature from Rankine-Hugoniot relations
    return m_initialTemperature * pressureRatio 
           * (2 * m_gamma + (m_gamma - 1) * pressureRatio) 
           / ((m_gamma + 1) * (m_gamma + 1));
}

inline double CJStateCalculator::calculateCJDensity(double pressureRatio) const {
    // CJ density from Rankine-Hugoniot relations
    return m_initialDensity * (m_gamma + 1) * pressureRatio 
           / (2 + (m_gamma - 1) * pressureRatio);
}

inline double CJStateCalculator::calculateCJVelocity(double pressureRatio) const {
    // CJ velocity (detonation velocity) from Rankine-Hugoniot relations
    double soundSpeed = std::sqrt(m_gamma * m_initialPressure / m_initialDensity);
    return soundSpeed * std::sqrt((m_gamma + 1) * pressureRatio / (2 * m_gamma) 
           + (m_gamma - 1) / (2 * m_gamma));
}

inline double CJStateCalculator::calculateDetonationMachNumber() const {
    // Detonation Mach number: M = D / c0
    double initialSoundSpeed = std::sqrt(m_gamma * m_initialPressure / m_initialDensity);
    return m_cjVelocity / initialSoundSpeed;
}

inline double CJStateCalculator::calculateVonNeumannPressure() const {
    // von Neumann pressure: pressure immediately behind the shock front
    // For strong shocks: p_vN ≈ (γ + 1) / 2 * ρ0 * D^2
    return 0.5 * (m_gamma + 1) * m_initialDensity * m_cjVelocity * m_cjVelocity;
}

#endif // CJ_STATE_CALCULATOR_HPP
