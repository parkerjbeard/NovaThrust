#ifndef MULTI_TUBE_CONFIGURATION_HPP
#define MULTI_TUBE_CONFIGURATION_HPP

#include <vector>
#include <memory>
#include <stdexcept>
#include <Eigen/Dense>
#include "pde_tube.hpp"
#include "manifold_system.hpp"
#include "thrust_vectoring_system.hpp"
#include "tube_interaction_model.hpp"
#include "valve_controller.hpp"
#include "tvc/performance_objectives.hpp"
#include "tvc/control_law.hpp"

namespace tvc {

// Remove the FlightState and ControlCommand structs as they are already defined

class MultiTubePDEConfiguration {
public:
    MultiTubePDEConfiguration(size_t numTubes);
    void initialize();
    void updateConfiguration(double timeStep);
    std::vector<PDETubeState> getTubeStates() const;
    std::vector<int> optimizeFiringSequence(const ThrustVector& desiredThrust);
    const TubeInteractionModel& getInteractionModel() const;
    const ThrustVectoringSystem& getThrustVectoringSystem() const;

    size_t getNumberOfTubes() const;
    void simulateFiringSequence(FlightState& simulatedState,
                                const std::vector<int>& firingSequence,
                                const ControlCommand& controlCommand);

private:
    size_t m_numTubes;
    std::vector<std::unique_ptr<PDETube>> m_tubes;
    std::unique_ptr<ManifoldSystem> m_manifoldSystem;
    std::unique_ptr<ThrustVectoringSystem> m_thrustVectoringSystem;
    std::unique_ptr<TubeInteractionModel> m_tubeInteractionModel;
    std::unique_ptr<ValveController> m_valveController;

    void updateTubes(double timeStep);
    void updateSystems(double timeStep);
};

} // namespace tvc

#endif // MULTI_TUBE_CONFIGURATION_HPP