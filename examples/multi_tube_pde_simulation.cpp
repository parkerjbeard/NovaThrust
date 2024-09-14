#include "multi_tube_pde/multi_tube_configuration.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <cmath>

// Simulation parameters
constexpr int NUM_TUBES = 6;
constexpr double SIMULATION_DURATION = 1.0; // seconds
constexpr double TIME_STEP = 0.001; // seconds
constexpr double DESIRED_THRUST_MAGNITUDE = 10000.0; // N

// Helper function to write simulation results to a CSV file
void writeResultsToCSV(const std::vector<std::vector<double>>& results, const std::string& filename) {
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
        return;
    }

    // Write header
    outFile << "Time,ThrustX,ThrustY,ThrustZ,TotalThrust" << std::endl;

    // Write data
    for (const auto& row : results) {
        for (size_t i = 0; i < row.size(); ++i) {
            outFile << row[i];
            if (i < row.size() - 1) {
                outFile << ",";
            }
        }
        outFile << std::endl;
    }

    outFile.close();
}

int main() {
    // Initialize multi-tube configuration
    tvc::MultiTubePDEConfiguration config(NUM_TUBES);
    config.initialize();

    // Set up desired thrust vector
    ThrustVector desiredThrust = {
        DESIRED_THRUST_MAGNITUDE * std::cos(M_PI / 4),
        DESIRED_THRUST_MAGNITUDE * std::sin(M_PI / 4),
        0.0
    };

    // Optimize firing sequence
    std::vector<int> firingSequence = config.optimizeFiringSequence(desiredThrust);

    std::cout << "Optimized firing sequence: ";
    for (int tubeIndex : firingSequence) {
        std::cout << tubeIndex << " ";
    }
    std::cout << std::endl;

    // Set up simulation
    std::vector<std::vector<double>> results;
    double currentTime = 0.0;

    // Main simulation loop
    while (currentTime < SIMULATION_DURATION) {
        // Update configuration
        config.updateConfiguration(TIME_STEP);

        // Get current thrust vector
        ThrustVector currentThrust = config.getThrustVectoringSystem().calculateThrustVector(config.getTubeStates());

        // Calculate total thrust magnitude
        double totalThrust = std::sqrt(currentThrust.x * currentThrust.x +
                                       currentThrust.y * currentThrust.y +
                                       currentThrust.z * currentThrust.z);

        // Store results
        results.push_back({currentTime, currentThrust.x, currentThrust.y, currentThrust.z, totalThrust});

        // Update time
        currentTime += TIME_STEP;
    }

    // Write results to CSV file
    writeResultsToCSV(results, "multi_tube_pde_simulation_results.csv");

    std::cout << "Simulation completed. Results written to multi_tube_pde_simulation_results.csv" << std::endl;

    // Calculate and display performance metrics
    double averageThrust = 0.0;
    double maxThrust = 0.0;
    double minThrust = std::numeric_limits<double>::max();

    for (const auto& row : results) {
        double thrust = row[4]; // Total thrust is in the 5th column
        averageThrust += thrust;
        maxThrust = std::max(maxThrust, thrust);
        minThrust = std::min(minThrust, thrust);
    }

    averageThrust /= results.size();

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Performance Metrics:" << std::endl;
    std::cout << "Average Thrust: " << averageThrust << " N" << std::endl;
    std::cout << "Maximum Thrust: " << maxThrust << " N" << std::endl;
    std::cout << "Minimum Thrust: " << minThrust << " N" << std::endl;

    // Calculate thrust vectoring efficiency
    double totalDesiredThrust = std::sqrt(desiredThrust.x * desiredThrust.x +
                                          desiredThrust.y * desiredThrust.y +
                                          desiredThrust.z * desiredThrust.z);
    double averageActualThrust = averageThrust;
    double thrustVectoringEfficiency = averageActualThrust / totalDesiredThrust * 100.0;

    std::cout << "Thrust Vectoring Efficiency: " << thrustVectoringEfficiency << "%" << std::endl;

    // Display tube interaction summary
    const TubeInteractionModel& interactionModel = config.getInteractionModel();
    std::cout << "\nTube Interaction Summary:" << std::endl;
    for (int i = 0; i < NUM_TUBES; ++i) {
        for (int j = i + 1; j < NUM_TUBES; ++j) {
            double interactionStrength = interactionModel.getInteractionStrength(i, j);
            std::cout << "Interaction between Tube " << i << " and Tube " << j
                      << ": " << interactionStrength << std::endl;
        }
    }

    return 0;
}