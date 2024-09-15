#include "flight_dynamics/atmospheric_model.hpp"
#include <random>
#include <stdexcept>

AtmosphericModel::AtmosphericModel()
    : generator(std::random_device{}()) {}  // Initialize the random number generator once

AtmosphericModel::AtmosphericProperties AtmosphericModel::getAtmosphericProperties(double altitude) const {
    if (altitude < 0.0 || altitude > 100000.0) {  // Validate altitude (0 to 100 km)
        throw std::out_of_range("Altitude must be between 0 and 100,000 meters.");
    }

    AtmosphericProperties props;

    // Determine the atmospheric layer based on altitude
    if (altitude <= 11000.0) {  // Troposphere
        props.temperature = calculateTemperatureTroposphere(altitude);
        props.pressure = calculatePressureTroposphere(altitude, props.temperature);
    }
    else if (altitude <= 25000.0) {  // Lower Stratosphere
        props.temperature = calculateTemperatureStratosphere(altitude);
        props.pressure = calculatePressureStratosphere(altitude, props.temperature);
    }
    else {  // Higher layers can be added similarly
        props.temperature = calculateTemperatureMesosphere(altitude);
        props.pressure = calculatePressureMesosphere(altitude, props.temperature);
    }

    props.density = calculateDensity(props.pressure, props.temperature);
    return props;
}

AtmosphericModel::WindVector AtmosphericModel::computeWindEffects(double altitude, double time) const {
    if (altitude < 0.0 || altitude > 100000.0) {  // Validate altitude
        throw std::out_of_range("Altitude must be between 0 and 100,000 meters.");
    }

    // Integrate a more sophisticated wind profile model
    WindVector wind = calculateAdvancedWindModel(altitude, time);
    WindVector gust = calculateGustModel(time);
    
    // Combine wind and gust effects
    double vx = wind.velocity * std::cos(wind.direction) + gust.velocity * std::cos(gust.direction);
    double vy = wind.velocity * std::sin(wind.direction) + gust.velocity * std::sin(gust.direction);
    
    WindVector result;
    result.velocity = std::sqrt(vx * vx + vy * vy);
    result.direction = std::atan2(vy, vx);
    return result;
}

double AtmosphericModel::getSpeedOfSound(double temperature) const {
    return std::sqrt(1.4 * R * temperature);  // 1.4 is the adiabatic index for air
}

double AtmosphericModel::calculateDensity(double pressure, double temperature) const {
    return pressure / (R * temperature);
}

double AtmosphericModel::calculatePressureTroposphere(double altitude, double temperature) const {
    return P0 * std::pow(temperature / T0, -g / (R * L));
}

double AtmosphericModel::calculateTemperatureTroposphere(double altitude) const {
    return T0 - L * altitude;
}

double AtmosphericModel::calculatePressureStratosphere(double altitude, double temperature) const {
    // Example calculation for stratosphere (simplified)
    return P0 * std::exp(-g * (altitude - 11000.0) / (R * temperature));
}

double AtmosphericModel::calculateTemperatureStratosphere(double altitude) const {
    // Example: Temperature inversion in stratosphere
    return 216.65;  // Constant temperature in lower stratosphere
}

double AtmosphericModel::calculatePressureMesosphere(double altitude, double temperature) const {
    // Example calculation for mesosphere (simplified)
    return P0 * std::exp(-g * (altitude - 25000.0) / (R * temperature));
}

double AtmosphericModel::calculateTemperatureMesosphere(double altitude) const {
    // Example: Decreasing temperature in mesosphere
    return 216.65 - 0.001 * (altitude - 25000.0);
}

AtmosphericModel::WindVector AtmosphericModel::calculateAdvancedWindModel(double altitude, double time) const {
    // Placeholder for a more sophisticated wind model
    // This can be replaced with real atmospheric data or a complex mathematical model
    WindVector wind;
    wind.velocity = 10.0 + 5.0 * std::sin(0.0001 * altitude) + 3.0 * std::cos(0.05 * time);
    wind.direction = M_PI * std::sin(0.0002 * altitude) + 0.3 * std::cos(0.02 * time);
    return wind;
}

AtmosphericModel::WindVector AtmosphericModel::calculateGustModel(double time) const {
    std::normal_distribution<double> velocity_distribution(0.0, 2.0);
    std::uniform_real_distribution<double> direction_distribution(0.0, 2.0 * M_PI);

    WindVector gust;
    gust.velocity = std::abs(velocity_distribution(generator));
    gust.direction = direction_distribution(generator);
    return gust;
}