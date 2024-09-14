#include "numerical_methods/pde_solvers.hpp"
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <fftw3.h>

namespace PDESolvers {

// Implementation of FiniteDifferenceSolver methods

FiniteDifferenceSolver::FiniteDifferenceSolver(
    std::function<double(double, double)> initialCondition,
    std::function<double(double, double, double)> pdeFunction,
    double xMin, double xMax, double tMax,
    int numSpatialPoints, int numTimeSteps,
    BoundaryCondition leftBC, BoundaryCondition rightBC
) : m_initialCondition(std::move(initialCondition)),
    m_pdeFunction(std::move(pdeFunction)),
    m_xMin(xMin), m_xMax(xMax), m_tMax(tMax),
    m_numSpatialPoints(numSpatialPoints), m_numTimeSteps(numTimeSteps),
    m_leftBC(leftBC), m_rightBC(rightBC) {
    m_solution.resize(m_numTimeSteps + 1, std::vector<double>(m_numSpatialPoints));
}

void FiniteDifferenceSolver::solve() {
    double dx = (m_xMax - m_xMin) / (m_numSpatialPoints - 1);
    double dt = m_tMax / m_numTimeSteps;

    // Initialize solution with initial condition
    for (int i = 0; i < m_numSpatialPoints; ++i) {
        double x = m_xMin + i * dx;
        m_solution[0][i] = m_initialCondition(x, 0);
    }

    // Time-stepping loop
    for (int n = 0; n < m_numTimeSteps; ++n) {
        double t = n * dt;
        m_solution[n + 1] = finiteDifferenceStep(m_solution[n], t, dt, dx);
        applyBoundaryConditions(m_solution[n + 1], t + dt);
    }
}

std::vector<double> FiniteDifferenceSolver::getSolution() const {
    return m_solution.back();
}

void FiniteDifferenceSolver::applyBoundaryConditions(std::vector<double>& u, double t) {
    double dx = (m_xMax - m_xMin) / (m_numSpatialPoints - 1);

    if (m_leftBC == BoundaryCondition::Dirichlet) {
        u[0] = m_initialCondition(m_xMin, t);
    } else if (m_leftBC == BoundaryCondition::Neumann) {
        u[0] = u[1] - dx * m_initialCondition(m_xMin, t);
    } else if (m_leftBC == BoundaryCondition::Periodic) {
        u[0] = u[m_numSpatialPoints - 2];
    }

    if (m_rightBC == BoundaryCondition::Dirichlet) {
        u[m_numSpatialPoints - 1] = m_initialCondition(m_xMax, t);
    } else if (m_rightBC == BoundaryCondition::Neumann) {
        u[m_numSpatialPoints - 1] = u[m_numSpatialPoints - 2] + dx * m_initialCondition(m_xMax, t);
    } else if (m_rightBC == BoundaryCondition::Periodic) {
        u[m_numSpatialPoints - 1] = u[1];
    }
}

std::vector<double> FiniteDifferenceSolver::finiteDifferenceStep(const std::vector<double>& u, double t, double dt, double dx) {
    std::vector<double> u_new(u.size());

    for (size_t i = 1; i < u.size() - 1; ++i) {
        double x = m_xMin + i * dx;
        u_new[i] = u[i] + dt * m_pdeFunction(x, t, (u[i+1] - 2*u[i] + u[i-1]) / (dx*dx));
    }

    return u_new;
}

// Implementation of FiniteVolumeSolver methods

FiniteVolumeSolver::FiniteVolumeSolver(
    std::function<double(double)> initialCondition,
    std::function<double(double)> fluxFunction,
    double xMin, double xMax, double tMax,
    int numCells, int numTimeSteps
) : m_initialCondition(std::move(initialCondition)),
    m_fluxFunction(std::move(fluxFunction)),
    m_xMin(xMin), m_xMax(xMax), m_tMax(tMax),
    m_numCells(numCells), m_numTimeSteps(numTimeSteps) {
    m_solution.resize(m_numTimeSteps + 1, std::vector<double>(m_numCells));
}

void FiniteVolumeSolver::solve() {
    double dx = (m_xMax - m_xMin) / m_numCells;
    double dt = m_tMax / m_numTimeSteps;

    // Initialize solution with initial condition
    for (int i = 0; i < m_numCells; ++i) {
        double x = m_xMin + (i + 0.5) * dx;
        m_solution[0][i] = m_initialCondition(x);
    }

    // Time-stepping loop
    for (int n = 0; n < m_numTimeSteps; ++n) {
        m_solution[n + 1] = finiteVolumeStep(m_solution[n], dt, dx);
    }
}

std::vector<double> FiniteVolumeSolver::getSolution() const {
    return m_solution.back();
}

double FiniteVolumeSolver::numericalFlux(double uLeft, double uRight) const {
    // Lax-Friedrichs flux
    double a = std::max(std::abs(m_fluxFunction(uLeft)), std::abs(m_fluxFunction(uRight)));
    return 0.5 * (m_fluxFunction(uLeft) + m_fluxFunction(uRight) - a * (uRight - uLeft));
}

std::vector<double> FiniteVolumeSolver::finiteVolumeStep(const std::vector<double>& u, double dt, double dx) {
    std::vector<double> u_new(u.size());
    std::vector<double> fluxes(u.size() + 1);

    // Compute fluxes
    for (size_t i = 0; i <= u.size(); ++i) {
        double uLeft = (i == 0) ? u.back() : u[i-1];
        double uRight = (i == u.size()) ? u.front() : u[i];
        fluxes[i] = numericalFlux(uLeft, uRight);
    }

    // Update solution
    for (size_t i = 0; i < u.size(); ++i) {
        u_new[i] = u[i] - (dt / dx) * (fluxes[i+1] - fluxes[i]);
    }

    return u_new;
}

// Implementation of SpectralSolver methods

SpectralSolver::SpectralSolver(
    std::function<double(double)> initialCondition,
    std::function<double(double, int)> spectralOperator,
    double xMin, double xMax, double tMax,
    int numModes, int numTimeSteps
) : m_initialCondition(std::move(initialCondition)),
    m_spectralOperator(std::move(spectralOperator)),
    m_xMin(xMin), m_xMax(xMax), m_tMax(tMax),
    m_numModes(numModes), m_numTimeSteps(numTimeSteps) {
    m_solution.resize(m_numModes);
}

void SpectralSolver::solve() {
    double dt = m_tMax / m_numTimeSteps;

    // Initialize solution with initial condition
    std::vector<double> u_initial(m_numModes);
    for (int i = 0; i < m_numModes; ++i) {
        double x = m_xMin + (m_xMax - m_xMin) * i / m_numModes;
        u_initial[i] = m_initialCondition(x);
    }
    m_solution = forwardFFT(u_initial);

    // Time-stepping loop
    for (int n = 0; n < m_numTimeSteps; ++n) {
        m_solution = spectralStep(m_solution, dt);
    }
}

std::vector<double> SpectralSolver::getSolution() const {
    return inverseFFT(m_solution);
}

std::vector<std::complex<double>> SpectralSolver::forwardFFT(const std::vector<double>& u) const {
    std::vector<std::complex<double>> result(m_numModes);
    fftw_plan plan = fftw_plan_dft_r2c_1d(m_numModes, const_cast<double*>(u.data()),
                                          reinterpret_cast<fftw_complex*>(result.data()),
                                          FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);
    return result;
}

std::vector<double> SpectralSolver::inverseFFT(const std::vector<std::complex<double>>& u) const {
    std::vector<double> result(m_numModes);
    fftw_plan plan = fftw_plan_dft_c2r_1d(m_numModes, const_cast<fftw_complex*>(reinterpret_cast<const fftw_complex*>(u.data())),
                                          result.data(), FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);
    for (double& val : result) {
        val /= m_numModes;
    }
    return result;
}

std::vector<std::complex<double>> SpectralSolver::spectralStep(const std::vector<std::complex<double>>& u, double dt) const {
    std::vector<std::complex<double>> u_new(m_numModes);
    for (int k = 0; k < m_numModes; ++k) {
        double lambda = m_spectralOperator(2 * M_PI * k / (m_xMax - m_xMin), k);
        u_new[k] = u[k] * std::exp(std::complex<double>(0, 1) * lambda * dt);
    }
    return u_new;
}

} // namespace PDESolvers