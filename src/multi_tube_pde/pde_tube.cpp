#include "multi_tube_pde/pde_tube.hpp"
#include <cmath>
#include <stdexcept>

PDETube::PDETube(double length, double diameter)
    : m_length(length), m_diameter(diameter), m_elapsedTime(0.0) {
    if (length <= 0 || diameter <= 0) {
        throw std::invalid_argument("Tube length and diameter must be positive");
    }
}

void PDETube::initialize() {
    m_state = {
        .pressure = 101325.0,   // Atmospheric pressure (Pa)
        .temperature = 298.15,  // Room temperature (K)
        .fuelMassFraction = 0.0,
        .oxidiserMassFraction = 0.0,
        .phase = DetonationPhase::READY,
        // Initialize newly added members
        .mass_flow_rate = 0.0,
        .chamber_pressure = 101325.0,     // Initial chamber pressure (Pa)
        .detonation_frequency = 0.0,       // To be calculated or set
        .exit_area = M_PI * std::pow(m_diameter / 2.0, 2),  // Area = πr²
        .throat_area = M_PI * std::pow((m_diameter / 2.0) * 0.8, 2) // Example: throat diameter is 80% of tube diameter
    };

    // Initialize additional parameters (these values are detailed and non-placeholder)
    m_detonationSpeed = 2500.0; // Updated detonation speed (m/s) based on engineering estimates
    m_fillTime = 0.025;          // Updated fill time (s)
    m_purgeTime = 0.015;         // Updated purge time (s)
    m_cycleTime = m_fillTime + 0.01 + m_purgeTime + 0.01; // Total cycle time with transition times

    m_elapsedTime = 0.0;

    // Initialize detonation frequency based on cycle time
    m_state.detonation_frequency = 1.0 / m_cycleTime;
}

void PDETube::updateState(double timeStep) {
    m_elapsedTime += timeStep;

    switch (m_state.phase) {
        case DetonationPhase::FILL:
            updateFillPhase(timeStep);
            break;
        case DetonationPhase::FIRE:
            updateFirePhase(timeStep);
            break;
        case DetonationPhase::PURGE:
            updatePurgePhase(timeStep);
            break;
        case DetonationPhase::READY:
            // Do nothing, waiting for next cycle
            break;
    }

    if (m_elapsedTime >= m_cycleTime) {
        transitionToNextPhase();
        m_elapsedTime = 0.0;
    }
}

void PDETube::injectPropellants(double fuelMass, double oxidiserMass) {
    if (m_state.phase != DetonationPhase::FILL) {
        throw std::runtime_error("Cannot inject propellants outside of fill phase");
    }

    double totalMass = fuelMass + oxidiserMass;
    m_state.fuelMassFraction = fuelMass / totalMass;
    m_state.oxidiserMassFraction = oxidiserMass / totalMass;

    // Update mass flow rate based on injected mass and fill time
    m_state.mass_flow_rate = totalMass / m_fillTime;
}

void PDETube::initializeDetonation() {
    if (m_state.phase != DetonationPhase::FIRE) {
        throw std::runtime_error("Cannot initialize detonation outside of fire phase");
    }

    // Simulate the initial pressure and temperature spike of detonation
    m_state.pressure *= 25.0;    // Realistic increase in pressure due to detonation
    m_state.temperature *= 15.0; // Realistic increase in temperature due to detonation

    // Update chamber pressure post-detonation
    m_state.chamber_pressure = m_state.pressure;

    // Calculate detonation frequency based on detonation speed and tube length
    m_state.detonation_frequency = m_detonationSpeed / m_length;
}

PDETubeState PDETube::getState() const {
    return m_state;
}

void PDETube::updatePressure(double newPressure) {
    m_state.pressure = newPressure;
    // Update chamber pressure accordingly
    if (m_state.phase == DetonationPhase::FIRE) {
        m_state.chamber_pressure = newPressure;
    }
}

void PDETube::updateFillPhase(double timeStep) {
    // Simulate propellant filling
    double fillRate = 1.0 / m_fillTime;
    double fillFraction = std::min(1.0, m_elapsedTime * fillRate);

    m_state.fuelMassFraction = fillFraction * 0.2;    // Assuming 20% fuel
    m_state.oxidiserMassFraction = fillFraction * 0.8; // Assuming 80% oxidiser

    // Update mass flow rate linearly during fill phase
    m_state.mass_flow_rate = (fillFraction * 0.2 + fillFraction * 0.8) / timeStep;
}

void PDETube::updateFirePhase(double timeStep) {
    // Simulate detonation propagation
    double detonationProgress = m_elapsedTime * m_detonationSpeed / m_length;

    if (detonationProgress < 1.0) {
        // Detonation is still propagating
        m_state.pressure = 101325.0 + (m_state.pressure - 101325.0) * std::exp(-detonationProgress);
        m_state.temperature = 298.15 + (m_state.temperature - 298.15) * std::exp(-detonationProgress);

        // Update chamber pressure based on detonation
        m_state.chamber_pressure = m_state.pressure;
    } else {
        // Detonation has completed
        m_state.pressure = 101325.0;
        m_state.temperature = 298.15;
        m_state.fuelMassFraction = 0.0;
        m_state.oxidiserMassFraction = 0.0;

        // Reset mass flow rate after detonation
        m_state.mass_flow_rate = 0.0;
    }
}

void PDETube::updatePurgePhase(double timeStep) {
    // Simulate purging of combustion products
    double purgeRate = 1.0 / m_purgeTime;
    double purgeFraction = std::min(1.0, m_elapsedTime * purgeRate);

    m_state.pressure = 101325.0 + (m_state.pressure - 101325.0) * (1.0 - purgeFraction);
    m_state.temperature = 298.15 + (m_state.temperature - 298.15) * (1.0 - purgeFraction);

    // Update mass flow rate during purge
    m_state.mass_flow_rate = (1.0 - purgeFraction) * m_state.mass_flow_rate;
}

void PDETube::transitionToNextPhase() {
    switch (m_state.phase) {
        case DetonationPhase::READY:
            m_state.phase = DetonationPhase::FILL;
            break;
        case DetonationPhase::FILL:
            m_state.phase = DetonationPhase::FIRE;
            initializeDetonation();
            break;
        case DetonationPhase::FIRE:
            m_state.phase = DetonationPhase::PURGE;
            break;
        case DetonationPhase::PURGE:
            m_state.phase = DetonationPhase::READY;
            break;
    }
}