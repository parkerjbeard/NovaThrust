#ifndef EOS_CALCULATOR_HPP
#define EOS_CALCULATOR_HPP

#include <vector>
#include <cmath>

class EOSCalculator {
public:
    enum class EOSType {
        REDLICH_KWONG,
        PENG_ROBINSON
    };

    EOSCalculator(EOSType eosType = EOSType::PENG_ROBINSON);
    ~EOSCalculator();

    // Setter methods
    void setEOSType(EOSType eosType);
    void setCriticalProperties(double Tc, double Pc, double omega);

    // Main calculation methods
    double calculatePressure(double T, double V, double n) const;
    double calculateVolume(double T, double P, double n) const;
    double calculateCompressibilityFactor(double T, double P) const;
    double calculateFugacityCoefficient(double T, double P) const;

    // Thermodynamic property calculations
    double calculateEnthalpy(double T, double P, double n) const;
    double calculateEntropy(double T, double P, double n) const;
    double calculateGibbsFreeEnergy(double T, double P, double n) const;

private:
    EOSType m_eosType;
    double m_Tc;  // Critical temperature
    double m_Pc;  // Critical pressure
    double m_omega;  // Acentric factor

    // EOS parameters
    double m_a;
    double m_b;

    // Helper methods
    void calculateEOSParameters();
    double calculateAlpha(double T) const;
    double solveForVolume(double T, double P, double n) const;
};

// Inline method implementations with formulas

inline double EOSCalculator::calculatePressure(double T, double V, double n) const {
    double R = 8.314;  // Gas constant in J/(mol·K)
    double alpha = calculateAlpha(T);

    if (m_eosType == EOSType::REDLICH_KWONG) {
        // Redlich-Kwong EOS: P = RT/(V-b) - a/(T^0.5 * V(V+b))
        return R * T / (V - n * m_b) - m_a * alpha / (std::sqrt(T) * V * (V + n * m_b));
    } else {  // Peng-Robinson
        // Peng-Robinson EOS: P = RT/(V-b) - a*alpha / (V^2 + 2bV - b^2)
        return R * T / (V - n * m_b) - m_a * alpha / (V * V + 2 * n * m_b * V - n * n * m_b * m_b);
    }
}

inline double EOSCalculator::calculateAlpha(double T) const {
    double Tr = T / m_Tc;
    double kappa;

    if (m_eosType == EOSType::REDLICH_KWONG) {
        return 1 / std::sqrt(Tr);
    } else {  // Peng-Robinson
        kappa = 0.37464 + 1.54226 * m_omega - 0.26992 * m_omega * m_omega;
        return std::pow(1 + kappa * (1 - std::sqrt(Tr)), 2);
    }
}

inline void EOSCalculator::calculateEOSParameters() {
    double R = 8.314;  // Gas constant in J/(mol·K)

    if (m_eosType == EOSType::REDLICH_KWONG) {
        m_a = 0.42748 * R * R * m_Tc * m_Tc * std::sqrt(m_Tc) / m_Pc;
        m_b = 0.08664 * R * m_Tc / m_Pc;
    } else {  // Peng-Robinson
        m_a = 0.45724 * R * R * m_Tc * m_Tc / m_Pc;
        m_b = 0.07780 * R * m_Tc / m_Pc;
    }
}

inline double EOSCalculator::calculateCompressibilityFactor(double T, double P) const {
    double R = 8.314;  // Gas constant in J/(mol·K)
    double V = solveForVolume(T, P, 1.0);  // Solve for molar volume
    return P * V / (R * T);
}

// Note: The following methods are declared but not fully implemented inline
// due to their complexity. They would be implemented in the .cpp file.

inline double EOSCalculator::calculateVolume(double T, double P, double n) const {
    return solveForVolume(T, P, n);
}

inline double EOSCalculator::calculateFugacityCoefficient(double T, double P) const {
    // Implementation would involve complex calculations based on the chosen EOS
    // This would typically be implemented in the .cpp file
    return 0.0;
}

inline double EOSCalculator::calculateEnthalpy(double T, double P, double n) const {
    // Implementation would involve partial derivatives of the EOS
    // This would typically be implemented in the .cpp file
    return 0.0;
}

inline double EOSCalculator::calculateEntropy(double T, double P, double n) const {
    // Implementation would involve partial derivatives of the EOS
    // This would typically be implemented in the .cpp file
    return 0.0;
}

inline double EOSCalculator::calculateGibbsFreeEnergy(double T, double P, double n) const {
    // G = H - TS
    return calculateEnthalpy(T, P, n) - T * calculateEntropy(T, P, n);
}

#endif // EOS_CALCULATOR_HPP
