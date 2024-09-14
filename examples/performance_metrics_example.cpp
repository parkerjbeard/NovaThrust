#include "performance_metrics/performance_metrics_calculator.hpp"
#include <iostream>

int main() {
    PerformanceMetricsCalculator calculator;

    // Calculate specific impulse
    double specificImpulse = calculator.calculateSpecificImpulse();
    std::cout << "Specific Impulse: " << specificImpulse << " s\n";

    // Calculate thrust
    double thrust = calculator.calculateThrust();
    std::cout << "Thrust: " << thrust << " N\n";

    // Generate a comprehensive performance report
    std::string report = calculator.generatePerformanceReport();
    std::cout << "\nFull Performance Report:\n" << report << std::endl;

    return 0;
}