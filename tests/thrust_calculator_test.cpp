#include "../include/performance_metrics/thrust_calculator.hpp"
#include <gtest/gtest.h>

class TestThrustCalculator : public ThrustCalculator {
public:
    using ThrustCalculator::computePressureForce;
    using ThrustCalculator::computeMomentumFlux;
    using ThrustCalculator::computeThrustCoefficient;
    using ThrustCalculator::computePulseDuration;
    using ThrustCalculator::computePeakThrust;
    using ThrustCalculator::computeThrustProfile;
};

class ThrustCalculatorTest : public ::testing::Test {
protected:
    TestThrustCalculator calculator;

    void SetUp() override {
        // Initialize with valid parameters
        calculator.setExitArea(1.0);                  // m^2
        calculator.setAmbientPressure(101325.0);      // Pa
        calculator.setMassFlowRate(10.0);             // kg/s
        calculator.setExhaustVelocity(3000.0);        // m/s
        calculator.setDetonationFrequency(50.0);      // Hz
        calculator.setChamberPressure(500000.0);      // Pa
        calculator.setExitPressure(150000.0);         // Pa (changed from 100000.0)
        calculator.setNozzleExpansionRatio(2.0);
    }
};

TEST_F(ThrustCalculatorTest, ComputePressureForceReturnsValidValue) {
    double pressureForce = calculator.computePressureForce();
    EXPECT_GT(pressureForce, 0.0);
}

TEST_F(ThrustCalculatorTest, ComputeMomentumFluxReturnsValidValue) {
    double momentumFlux = calculator.computeMomentumFlux();
    EXPECT_GT(momentumFlux, 0.0);
}

TEST_F(ThrustCalculatorTest, ComputeThrustCoefficientReturnsValidValue) {
    double thrustCoefficient = calculator.computeThrustCoefficient();
    EXPECT_GT(thrustCoefficient, 0.0);
}

TEST_F(ThrustCalculatorTest, ComputePulseDurationReturnsValidValue) {
    double pulseDuration = calculator.computePulseDuration();
    EXPECT_GT(pulseDuration, 0.0);
}

TEST_F(ThrustCalculatorTest, ComputePeakThrustReturnsValidValue) {
    double peakThrust = calculator.computePeakThrust();
    EXPECT_GT(peakThrust, 0.0);
}

TEST_F(ThrustCalculatorTest, ComputeThrustProfileReturnsValidValues) {
    // Test at different normalized times
    double times[] = {0.05, 0.15, 0.5, 0.95};
    for (double t : times) {
        double profile = calculator.computeThrustProfile(t);
        EXPECT_GE(profile, 0.0);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}