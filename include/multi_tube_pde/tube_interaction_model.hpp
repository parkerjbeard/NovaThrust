#ifndef TUBE_INTERACTION_MODEL_HPP
#define TUBE_INTERACTION_MODEL_HPP

#include <vector>
#include <memory>
#include "pde_tube.hpp"

class TubeInteractionModel {
public:
    TubeInteractionModel(size_t numTubes);
    ~TubeInteractionModel() = default;

    void initialize();
    void updateInteractions(const std::vector<std::unique_ptr<PDETube>>& tubes, double timeStep);
    double getInteractionStrength(size_t tube1, size_t tube2) const;

private:
    size_t m_numTubes;
    std::vector<std::vector<double>> m_interactionMatrix;

    void calculateInteractionMatrix();
    void calculateCrossTalk(const std::vector<std::unique_ptr<PDETube>>& tubes, double timeStep);
    void propagatePressureWaves(const std::vector<std::unique_ptr<PDETube>>& tubes, double timeStep);
    double calculatePressureEffect(const PDETube& sourceTube, const PDETube& targetTube) const;
};

#endif // TUBE_INTERACTION_MODEL_HPP