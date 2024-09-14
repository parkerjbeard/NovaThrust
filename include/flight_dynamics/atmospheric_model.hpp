#ifndef ATMOSPHERIC_MODEL_HPP
#define ATMOSPHERIC_MODEL_HPP

#include <array>
#include <cmath>

class AtmosphericModel {
public:
    struct AtmosphericProperties {
        double density;      // kg/m^3
        double temperature;  // K
        double pressure;     // Pa
    };

    struct WindVector {
        double velocity;     // m/s
        double direction;    // radians
    };

    AtmosphericModel();
    ~AtmosphericModel() = default;

    AtmosphericProperties getAtmosphericProperties(double altitude) const;
    WindVector computeWindEffects(double altitude, double time) const;
    double getSpeedOfSound(double temperature) const;

private:
    static constexpr double R = 287.05;  // Specific gas constant for air (J/(kg*K))
    static constexpr double g = 9.80665; // Gravitational acceleration (m/s^2)
    static constexpr double L = 0.0065;  // Temperature lapse rate (K/m)
    static constexpr double P0 = 101325; // Sea level standard atmospheric pressure (Pa)
    static constexpr double T0 = 288.15; // Sea level standard temperature (K)

    double calculateDensity(double pressure, double temperature) const;
    double calculatePressure(double altitude) const;
    double calculateTemperature(double altitude) const;
    WindVector calculateWindModel(double altitude, double time) const;
    WindVector calculateGustModel(double time) const;
};

#endif // ATMOSPHERIC_MODEL_HPP