#ifndef DETONATION_PARAMETERS_CALCULATOR_HPP
#define DETONATION_PARAMETERS_CALCULATOR_HPP

#include <string>
#include <cmath>

class DetonationParametersCalculator {
public:
    DetonationParametersCalculator();
    ~DetonationParametersCalculator();

    // Main calculation method
    void calculate();

    // Getter methods for calculated parameters
    double getWaveSpeed() const;
    double getPeakPressure() const;
    double getCellSize() const;
    double getDetonationFrequency() const;

    // Setter methods for input parameters
    void setInitialPressure(double pressure);
    void setInitialTemperature(double temperature);
    void setEquivalenceRatio(double ratio);
    void setFuelType(const std::string& fuel);
    void setOxidizerType(const std::string& oxidizer);
    void setTubeLength(double length);
    void setTubeDiameter(double diameter);

    // Method to get a report of the calculated parameters
    std::string getReport() const;

private:
    // Input parameters
    double m_initialPressure;    // Pa
    double m_initialTemperature; // K
    double m_equivalenceRatio;
    std::string m_fuelType;
    std::string m_oxidizer;
    double m_tubeLength;         // m
    double m_tubeDiameter;       // m

    // Calculated parameters
    double m_waveSpeed;          // m/s
    double m_peakPressure;       // Pa
    double m_cellSize;           // m
    double m_detonationFrequency;// Hz

    // Helper methods
    double calculateWaveSpeed();
    double calculatePeakPressure();
    double calculateCellSize();
    double calculateDetonationFrequency();

    // Additional helper methods
    double calculateSpecificHeatRatio();
    double calculateHeatOfCombustion();
    double calculateChapmanJouguetMachNumber();
};

#endif // DETONATION_PARAMETERS_CALCULATOR_HPP