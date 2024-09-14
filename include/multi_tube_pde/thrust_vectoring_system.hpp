#ifndef THRUST_VECTORING_SYSTEM_HPP
#define THRUST_VECTORING_SYSTEM_HPP

#include <vector>
#include <memory>
#include "pde_tube.hpp"

struct ThrustVector {
    double x, y, z;
};

struct FlightConditions {
    double altitude;
    double velocity;
    double angle_of_attack;
    double sideslip_angle;
};

class ThrustVectoringSystem {
public:
    ThrustVectoringSystem(size_t numTubes);
    void initialize();
    ThrustVector calculateThrustVector(const std::vector<PDETubeState>& tubeStates) const;
    std::vector<int> optimizeFiringSequence(const std::vector<std::unique_ptr<PDETube>>& tubes, const ThrustVector& desiredThrust);
    std::vector<ThrustVector> predictThrustProfile(const std::vector<int>& firingSequence, double duration);
    void adjustForFlightConditions(const FlightConditions& conditions);

private:
    size_t m_numTubes;
    std::vector<std::array<double, 3>> m_tubeOrientations;
    FlightConditions m_currentFlightConditions;

    double calculateTubeThrustMagnitude(const PDETubeState& tubeState) const;
    double calculateFillFraction(const PDETubeState& tubeState) const;
    double calculateSpecificImpulse(const PDETubeState& tubeState) const;
    double calculateThroatArea(const PDETubeState& tubeState) const;
    double calculateExpansionRatio(double altitude) const;
    double calculateAmbientPressure(double altitude) const;
    ThrustVector rotateThrust(const ThrustVector& thrust, double pitch, double yaw) const;
};

#endif // THRUST_VECTORING_SYSTEM_HPP