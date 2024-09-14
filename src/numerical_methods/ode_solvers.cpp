#include "numerical_methods/ode_solvers.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <numeric>

namespace ODESolvers {

// Utility functions implementation
std::vector<double> add(const std::vector<double>& a, const std::vector<double>& b) {
    std::vector<double> result(a.size());
    std::transform(a.begin(), a.end(), b.begin(), result.begin(), std::plus<>());
    return result;
}

std::vector<double> subtract(const std::vector<double>& a, const std::vector<double>& b) {
    std::vector<double> result(a.size());
    std::transform(a.begin(), a.end(), b.begin(), result.begin(), std::minus<>());
    return result;
}

std::vector<double> multiply(const std::vector<double>& a, double scalar) {
    std::vector<double> result(a.size());
    std::transform(a.begin(), a.end(), result.begin(), [scalar](double x) { return x * scalar; });
    return result;
}

double vectorNorm(const std::vector<double>& v) {
    return std::sqrt(std::inner_product(v.begin(), v.end(), v.begin(), 0.0));
}

// RungeKutta4 implementation
RungeKutta4::RungeKutta4(ODEFunction f) : m_f(std::move(f)) {}

std::vector<double> RungeKutta4::step(double t, const std::vector<double>& y, double dt) {
    std::vector<double> k1 = m_f(t, y);
    std::vector<double> k2 = m_f(t + 0.5*dt, add(y, multiply(k1, 0.5*dt)));
    std::vector<double> k3 = m_f(t + 0.5*dt, add(y, multiply(k2, 0.5*dt)));
    std::vector<double> k4 = m_f(t + dt, add(y, multiply(k3, dt)));

    std::vector<double> k_sum = add(k1, add(multiply(k2, 2.0), add(multiply(k3, 2.0), k4)));
    return add(y, multiply(k_sum, dt/6.0));
}

// AdaptiveRungeKutta implementation
AdaptiveRungeKutta::AdaptiveRungeKutta(ODEFunction f, double tolerance, double minStep, double maxStep)
    : m_f(std::move(f)), m_tolerance(tolerance), m_minStep(minStep), m_maxStep(maxStep) {}

std::vector<double> AdaptiveRungeKutta::step(double t, const std::vector<double>& y, double dt) {
    double h = dt;
    std::vector<double> y_new;
    double error;

    do {
        std::tie(y_new, error) = dormandPrince54Step(t, y, h);

        if (error > m_tolerance) {
            h = std::max(m_minStep, 0.9 * h * std::pow(m_tolerance / error, 0.2));
        } else if (error < 0.1 * m_tolerance) {
            h = std::min(m_maxStep, 1.1 * h * std::pow(m_tolerance / error, 0.2));
        }
    } while (error > m_tolerance);

    return y_new;
}

std::tuple<std::vector<double>, double> AdaptiveRungeKutta::dormandPrince54Step(double t, const std::vector<double>& y, double h) {
    // Coefficients for Dormand-Prince 5(4) method
    static const std::vector<double> a2 = {1.0/5.0};
    static const std::vector<double> a3 = {3.0/40.0, 9.0/40.0};
    static const std::vector<double> a4 = {44.0/45.0, -56.0/15.0, 32.0/9.0};
    static const std::vector<double> a5 = {19372.0/6561.0, -25360.0/2187.0, 64448.0/6561.0, -212.0/729.0};
    static const std::vector<double> a6 = {9017.0/3168.0, -355.0/33.0, 46732.0/5247.0, 49.0/176.0, -5103.0/18656.0};
    static const std::vector<double> b = {35.0/384.0, 0.0, 500.0/1113.0, 125.0/192.0, -2187.0/6784.0, 11.0/84.0};
    static const std::vector<double> bstar = {5179.0/57600.0, 0.0, 7571.0/16695.0, 393.0/640.0, -92097.0/339200.0, 187.0/2100.0, 1.0/40.0};

    std::vector<double> k1 = m_f(t, y);
    std::vector<double> k2 = m_f(t + a2[0]*h, add(y, multiply(k1, h*a2[0])));
    std::vector<double> k3 = m_f(t + h*(a3[0]+a3[1]), add(y, add(multiply(k1, h*a3[0]), multiply(k2, h*a3[1]))));
    std::vector<double> k4 = m_f(t + h*(a4[0]+a4[1]+a4[2]), add(y, add(multiply(k1, h*a4[0]), add(multiply(k2, h*a4[1]), multiply(k3, h*a4[2])))));
    std::vector<double> k5 = m_f(t + h*(a5[0]+a5[1]+a5[2]+a5[3]), add(y, add(multiply(k1, h*a5[0]), add(multiply(k2, h*a5[1]), add(multiply(k3, h*a5[2]), multiply(k4, h*a5[3]))))));
    std::vector<double> k6 = m_f(t + h*(a6[0]+a6[1]+a6[2]+a6[3]+a6[4]), add(y, add(multiply(k1, h*a6[0]), add(multiply(k2, h*a6[1]), add(multiply(k3, h*a6[2]), add(multiply(k4, h*a6[3]), multiply(k5, h*a6[4])))))));
    std::vector<double> k7 = m_f(t + h, add(y, add(multiply(k1, h*b[0]), add(multiply(k3, h*b[2]), add(multiply(k4, h*b[3]), add(multiply(k5, h*b[4]), multiply(k6, h*b[5])))))));

    std::vector<double> y_new = add(y, add(multiply(k1, h*b[0]), add(multiply(k3, h*b[2]), add(multiply(k4, h*b[3]), add(multiply(k5, h*b[4]), multiply(k6, h*b[5]))))));
    std::vector<double> y_error = multiply(subtract(multiply(k1, bstar[0]-b[0]), add(multiply(k3, bstar[2]-b[2]), add(multiply(k4, bstar[3]-b[3]), add(multiply(k5, bstar[4]-b[4]), add(multiply(k6, bstar[5]-b[5]), multiply(k7, bstar[6])))))), h);

    double error = vectorNorm(y_error);

    return {y_new, error};
}

// BDFSolver implementation
BDFSolver::BDFSolver(ODEFunction f, int order)
    : m_f(std::move(f)), m_order(order), m_history(order + 1), m_starter(std::make_unique<RungeKutta4>(m_f)) {
    if (order < 1 || order > 6) {
        throw std::invalid_argument("BDF order must be between 1 and 6");
    }
    initializeAlphaCoefficients();
}

void BDFSolver::initializeAlphaCoefficients() {
    m_alpha = {
        {1},
        {4.0/3.0, -1.0/3.0},
        {18.0/11.0, -9.0/11.0, 2.0/11.0},
        {48.0/25.0, -36.0/25.0, 16.0/25.0, -3.0/25.0},
        {300.0/137.0, -300.0/137.0, 200.0/137.0, -75.0/137.0, 12.0/137.0},
        {360.0/147.0, -450.0/147.0, 400.0/147.0, -225.0/147.0, 72.0/147.0, -10.0/147.0}
    };
}

std::vector<double> BDFSolver::step(double t, const std::vector<double>& y, double dt) {
    if (m_history.size() < m_order + 1) {
        // Use RK4 to build up history for BDF
        m_history.push_back({t, y});
        while (m_history.size() < m_order + 1) {
            auto [t_prev, y_prev] = m_history.back();
            auto y_next = m_starter->step(t_prev, y_prev, dt);
            m_history.push_back({t_prev + dt, y_next});
        }
    }

    std::vector<double> y_pred(y.size());
    for (int i = 0; i <= m_order; ++i) {
        for (size_t j = 0; j < y.size(); ++j) {
            y_pred[j] += m_alpha[m_order - 1][i] * m_history[m_history.size() - 1 - i].second[j];
        }
    }

    // Newton iteration to solve the implicit equation
    std::vector<double> y_new = y_pred;
    const int max_iterations = 10;
    const double tolerance = 1e-8;

    for (int iter = 0; iter < max_iterations; ++iter) {
        std::vector<double> f_new = m_f(t + dt, y_new);
        std::vector<double> residual = subtract(y_new, add(y_pred, multiply(f_new, dt * m_alpha[m_order - 1][0])));

        if (vectorNorm(residual) < tolerance) {
            break;
        }

        std::vector<std::vector<double>> jacobian = approximateJacobian(t + dt, y_new, dt);
        std::vector<double> delta = solveLinearSystem(jacobian, residual);
        y_new = subtract(y_new, delta);
    }

    // Update history
    m_history.push_back({t + dt, y_new});
    m_history.erase(m_history.begin());

    return y_new;
}

void BDFSolver::reset() {
    m_history.clear();
}

std::vector<std::vector<double>> BDFSolver::approximateJacobian(double t, const std::vector<double>& y, double dt) {
    const double eps = 1e-8;
    std::vector<std::vector<double>> jacobian(y.size(), std::vector<double>(y.size()));
    std::vector<double> f_base = m_f(t, y);

    for (size_t i = 0; i < y.size(); ++i) {
        std::vector<double> y_perturbed = y;
        y_perturbed[i] += eps;
        std::vector<double> f_perturbed = m_f(t, y_perturbed);

        for (size_t j = 0; j < y.size(); ++j) {
            jacobian[j][i] = (f_perturbed[j] - f_base[j]) / eps;
        }
        jacobian[i][i] += 1.0 / (dt * m_alpha[m_order - 1][0]);
    }

    return jacobian;
}

std::vector<double> BDFSolver::solveLinearSystem(const std::vector<std::vector<double>>& A, const std::vector<double>& b) {
    // Implement a linear system solver (e.g., LU decomposition or iterative method)
    // For simplicity, we'll use Gaussian elimination with partial pivoting
    int n = b.size();
    std::vector<std::vector<double>> augmented_matrix = A;
    for (int i = 0; i < n; ++i) {
        augmented_matrix[i].push_back(b[i]);
    }

    for (int i = 0; i < n; ++i) {
        // Partial pivoting
        int max_row = i;
        for (int j = i + 1; j < n; ++j) {
            if (std::abs(augmented_matrix[j][i]) > std::abs(augmented_matrix[max_row][i])) {
                max_row = j;
            }
        }
        std::swap(augmented_matrix[i], augmented_matrix[max_row]);

        // Elimination
        for (int j = i + 1; j < n; ++j) {
            double factor = augmented_matrix[j][i] / augmented_matrix[i][i];
            for (int k = i; k <= n; ++k) {
                augmented_matrix[j][k] -= factor * augmented_matrix[i][k];
            }
        }
    }

    // Back-substitution
    std::vector<double> solution(n);
    for (int i = n - 1; i >= 0; --i) {
        solution[i] = augmented_matrix[i][n];
        for (int j = i + 1; j < n; ++j) {
            solution[i] -= augmented_matrix[i][j] * solution[j];
        }
        solution[i] /= augmented_matrix[i][i];
    }

    return solution;
}

} // namespace ODESolvers