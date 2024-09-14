#ifndef PDE_TUBE_HPP
#define PDE_TUBE_HPP

#include <vector>

enum class DetonationPhase {
    FILL,
    FIRE,
    PURGE,
    READY
};

struct PDETubeState {
    double pressure;
    double temperature;
    double fuelMassFraction;
    double oxidiserMassFraction;
    DetonationPhase phase;

    // Newly added members
    double mass_flow_rate;
    double chamber_pressure;
    double detonation_frequency;
    double exit_area;
    double throat_area;
};

class PDETube {
public:
    PDETube(double length = 1.0, double diameter = 0.1);
    ~PDETube() = default;

    void initialize();
    void updateState(double timeStep);
    void injectPropellants(double fuelMass, double oxidiserMass);
    void initializeDetonation();

    PDETubeState getState() const;
    void updatePressure(double newPressure);

    // Getter methods for tube parameters
    double getLength() const { return m_length; }
    double getDiameter() const { return m_diameter; }

private:
    double m_length;
    double m_diameter;
    PDETubeState m_state;

    // Additional parameters
    double m_detonationSpeed;
    double m_fillTime;
    double m_purgeTime;
    double m_cycleTime;

    double m_elapsedTime;

    void updateFillPhase(double timeStep);
    void updateFirePhase(double timeStep);
    void updatePurgePhase(double timeStep);
    void transitionToNextPhase();
};

#endif // PDE_TUBE_HPP