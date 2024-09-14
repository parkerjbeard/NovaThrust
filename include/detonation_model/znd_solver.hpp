#ifndef ZND_SOLVER_HPP
#define ZND_SOLVER_HPP

#include <vector>
#include <functional>

class ZNDSolver {
public:
    ZNDSolver();
    ~ZNDSolver();

    // Main solving methods
    void solve(const std::vector<double>& initialConditions);
    void analyzeDetonationStructure();

    // Setter methods for model parameters
    void setReactionRate(const std::function<double(double, double)>& rate);
    void setEquationOfState(const std::function<double(double, double)>& eos);
    void setSpecificHeatRatio(double gamma);

    // Getter methods for results
    std::vector<double> getTemperatureProfile() const;
    std::vector<double> getPressureProfile() const;
    std::vector<double> getDensityProfile() const;
    std::vector<double> getVelocityProfile() const;
    std::vector<double> getReactionProgressProfile() const;

    // Utility methods
    double calculateDetonationVelocity() const;
    double calculateReactionZoneLength() const;

private:
    // Private member variables
    std::vector<double> m_temperatureProfile;
    std::vector<double> m_pressureProfile;
    std::vector<double> m_densityProfile;
    std::vector<double> m_velocityProfile;
    std::vector<double> m_reactionProgressProfile;
    
    std::function<double(double, double)> m_reactionRate;
    std::function<double(double, double)> m_equationOfState;
    double m_gamma;

    // Private helper methods
    void integrateZNDEquations();
    void calculateProfiles();
    bool checkConvergence();
};

#endif // ZND_SOLVER_HPP
