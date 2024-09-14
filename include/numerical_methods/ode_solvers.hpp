#ifndef ODE_SOLVERS_HPP
#define ODE_SOLVERS_HPP

#include <vector>
#include <functional>
#include <memory>

namespace ODESolvers {

// Type alias for the ODE function: dy/dt = f(t, y)
using ODEFunction = std::function<std::vector<double>(double, const std::vector<double>&)>;

// Base class for ODE solvers
class ODESolver {
public:
    virtual ~ODESolver() = default;
    virtual std::vector<double> step(double t, const std::vector<double>& y, double dt) = 0;
    virtual void reset() {}
};

// 4th order Runge-Kutta method
class RungeKutta4 : public ODESolver {
public:
    explicit RungeKutta4(ODEFunction f);
    std::vector<double> step(double t, const std::vector<double>& y, double dt) override;

private:
    ODEFunction m_f;
};

// Adaptive step size control using embedded Runge-Kutta methods
class AdaptiveRungeKutta : public ODESolver {
public:
    AdaptiveRungeKutta(ODEFunction f, double tolerance = 1e-6, double minStep = 1e-10, double maxStep = 1.0);
    std::vector<double> step(double t, const std::vector<double>& y, double dt) override;

private:
    ODEFunction m_f;
    double m_tolerance;
    double m_minStep;
    double m_maxStep;

    std::tuple<std::vector<double>, double> dormandPrince54Step(double t, const std::vector<double>& y, double h);
};

// Implicit solver for stiff systems (Backward Differentiation Formula)
class BDFSolver : public ODESolver {
public:
    BDFSolver(ODEFunction f, int order = 2);
    std::vector<double> step(double t, const std::vector<double>& y, double dt) override;
    void reset() override;

private:
    ODEFunction m_f;
    int m_order;
    std::vector<std::pair<double, std::vector<double>>> m_history;
    std::unique_ptr<RungeKutta4> m_starter;
    std::vector<std::vector<double>> m_alpha;

    void initializeAlphaCoefficients();
    std::vector<std::vector<double>> approximateJacobian(double t, const std::vector<double>& y, double dt);
    std::vector<double> solveLinearSystem(const std::vector<std::vector<double>>& A, const std::vector<double>& b);
};

// Utility functions
std::vector<double> add(const std::vector<double>& a, const std::vector<double>& b);
std::vector<double> subtract(const std::vector<double>& a, const std::vector<double>& b);
std::vector<double> multiply(const std::vector<double>& a, double scalar);
double vectorNorm(const std::vector<double>& v);

} // namespace ODESolvers

#endif // ODE_SOLVERS_HPP