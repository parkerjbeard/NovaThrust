#include "../include/performance_metrics/thrust_calculator.hpp"
#include <iostream>
#include <iomanip>

void printThrust(const std::string& label, double thrust) {
    std::cout << std::setw(25) << std::left << label 
              << std::setw(10) << std::right << std::fixed << std::setprecision(2) 
              << thrust << " N" << std::endl;
}

int main() {
    ThrustCalculator calculator;

    // Set parameters for a medium-sized liquid rocket engine (similar to SpaceX Merlin 1D)
    calculator.setExitArea(1.55);              // m^2
    calculator.setAmbientPressure(101325.0);   // Pa (sea level)
    calculator.setMassFlowRate(236.0);         // kg/s
    calculator.setExhaustVelocity(3050.0);     // m/s
    calculator.setDetonationFrequency(0.0);    // Hz (not applicable for this engine type)
    calculator.setChamberPressure(9700000.0);  // Pa
    calculator.setExitPressure(101325.0);      // Pa (optimized for sea level)
    calculator.setNozzleExpansionRatio(16.0);

    std::cout << "Thrust Calculations for Medium-sized Liquid Rocket Engine" << std::endl;
    std::cout << "=========================================================" << std::endl;

    // Calculate and print peak thrust
    double peakThrust = calculator.calculate();
    printThrust("Peak Thrust:", peakThrust);

    // Calculate and print instantaneous thrust at different times
    double times[] = {0.0, 0.5, 1.0, 2.0, 5.0};
    for (double time : times) {
        double instantThrust = calculator.calculateInstantaneousThrust(time);
        printThrust("Thrust at t=" + std::to_string(time) + "s:", instantThrust);
    }

    // Calculate and print average thrust over different time intervals
    std::pair<double, double> intervals[] = {{0.0, 1.0}, {0.0, 5.0}, {1.0, 5.0}};
    for (const auto& interval : intervals) {
        double avgThrust = calculator.calculateAverageThrust(interval.first, interval.second);
        printThrust("Avg Thrust t=" + std::to_string(interval.first) + 
                    " to " + std::to_string(interval.second) + "s:", avgThrust);
    }

    return 0;
}
