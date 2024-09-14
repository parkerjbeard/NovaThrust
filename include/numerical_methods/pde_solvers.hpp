#ifndef PDE_SOLVERS_HPP
#define PDE_SOLVERS_HPP

#include <vector>
#include <functional>
#include <complex>

namespace PDESolvers {

// Base class for PDE solvers
class PDESolver {
public:
    virtual ~PDESolver() = default;
    virtual void solve() = 0;
    virtual std::vector<double> getSolution() const = 0;
};

// Finite Difference Method Solver
class FiniteDifferenceSolver : public PDESolver {
public:
    enum class BoundaryCondition { Dirichlet, Neumann, Periodic };

    FiniteDifferenceSolver(
        std::function<double(double, double)> initialCondition,
        std::function<double(double, double, double)> pdeFunction,
        double xMin, double xMax, double tMax,
        int numSpatialPoints, int numTimeSteps,
        BoundaryCondition leftBC, BoundaryCondition rightBC
    );

    void solve() override;
    std::vector<double> getSolution() const override;

private:
    std::function<double(double, double)> m_initialCondition;
    std::function<double(double, double, double)> m_pdeFunction;
    double m_xMin, m_xMax, m_tMax;
    int m_numSpatialPoints, m_numTimeSteps;
    BoundaryCondition m_leftBC, m_rightBC;
    std::vector<std::vector<double>> m_solution;

    void applyBoundaryConditions(std::vector<double>& u, double t);
    std::vector<double> finiteDifferenceStep(const std::vector<double>& u, double t, double dt, double dx);
};

// Finite Volume Method Solver
class FiniteVolumeSolver : public PDESolver {
public:
    FiniteVolumeSolver(
        std::function<double(double)> initialCondition,
        std::function<double(double)> fluxFunction,
        double xMin, double xMax, double tMax,
        int numCells, int numTimeSteps
    );

    void solve() override;
    std::vector<double> getSolution() const override;

private:
    std::function<double(double)> m_initialCondition;
    std::function<double(double)> m_fluxFunction;
    double m_xMin, m_xMax, m_tMax;
    int m_numCells, m_numTimeSteps;
    std::vector<std::vector<double>> m_solution;

    double numericalFlux(double uLeft, double uRight) const;
    std::vector<double> finiteVolumeStep(const std::vector<double>& u, double dt, double dx);
};

// Spectral Method Solver
class SpectralSolver : public PDESolver {
public:
    SpectralSolver(
        std::function<double(double)> initialCondition,
        std::function<double(double, int)> spectralOperator,
        double xMin, double xMax, double tMax,
        int numModes, int numTimeSteps
    );

    void solve() override;
    std::vector<double> getSolution() const override;

private:
    std::function<double(double)> m_initialCondition;
    std::function<double(double, int)> m_spectralOperator;
    double m_xMin, m_xMax, m_tMax;
    int m_numModes, m_numTimeSteps;
    std::vector<std::complex<double>> m_solution;

    std::vector<std::complex<double>> forwardFFT(const std::vector<double>& u) const;
    std::vector<double> inverseFFT(const std::vector<std::complex<double>>& u) const;
    std::vector<std::complex<double>> spectralStep(const std::vector<std::complex<double>>& u, double dt) const;
};

} // namespace PDESolvers

#endif // PDE_SOLVERS_HPP