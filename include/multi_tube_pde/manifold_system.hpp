#ifndef MANIFOLD_SYSTEM_HPP
#define MANIFOLD_SYSTEM_HPP

#include <vector>
#include "pde_tube.hpp"

struct ManifoldState {
    double fuelPressure;
    double oxidizerPressure;
    double fuelFlowRate;
    double oxidizerFlowRate;
};

class ManifoldSystem {
public:
    explicit ManifoldSystem(size_t numTubes);
    ~ManifoldSystem() = default;

    void initialize();
    void updateState(double timeStep);
    void injectFuel(size_t tubeIndex, double amount);
    void injectOxidizer(size_t tubeIndex, double amount);
    ManifoldState getManifoldState() const;

private:
    size_t m_numTubes;
    ManifoldState m_state;
    std::vector<double> m_tubeFuelInjectionRates;
    std::vector<double> m_tubeOxidizerInjectionRates;

    void updateManifoldPressures(double timeStep);
    void calculateInjectionRates();
};

#endif // MANIFOLD_SYSTEM_HPP