#include <iostream>
#include "performance_metrics/thrust_calculator.hpp"

int main() {
    std::cout << "Welcome to NovaThrust!" << std::endl;
    
    ThrustCalculator calculator;
    double thrust = calculator.calculate();
    
    std::cout << "Calculated Thrust: " << thrust << std::endl;
    
    return 0;
}