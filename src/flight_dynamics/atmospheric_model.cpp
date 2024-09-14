#include "flight_dynamics/atmospheric_model.hpp"
#include <random>

AtmosphericModel::AtmosphericModel() {}

AtmosphericModel::AtmosphericProperties AtmosphericModel::getAtmosphericProperties(double altitude) const {
    AtmosphericProperties props;
    props.temperature = calculateTemperature(altitude);
    props.pressure = calculatePressure(altitude);
    props.density = calculateDensity(props.pressure, props.temperature);
    return props;
}

AtmosphericModel::WindVector AtmosphericModel::computeWindEffects(double altitude, double time) const {
    WindVector wind = calculateWindModel(altitude, time);
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

double AtmosphericModel::calculatePressure(double altitude) const {
    double temperature = calculateTemperature(altitude);
    return P0 * std::pow(temperature / T0, -g / (R * L));
}

double AtmosphericModel::calculateTemperature(double altitude) const {
    return T0 - L * altitude;
}

AtmosphericModel::WindVector AtmosphericModel::calculateWindModel(double altitude, double time) const {
    // Implement a simple wind model that varies with altitude and time
    WindVector wind;
    wind.velocity = 5.0 + 2.0 * std::sin(0.001 * altitude) + 1.5 * std::cos(0.1 * time);
    wind.direction = 0.5 * M_PI * std::sin(0.0005 * altitude) + 0.2 * std::cos(0.05 * time);
    return wind;
}

AtmosphericModel::WindVector AtmosphericModel::calculateGustModel(double time) const {
    // Implement a simple gust model using random number generation
    static std::default_random_engine generator(static_cast<unsigned>(time));
    std::normal_distribution<double> velocity_distribution(0.0, 2.0);
    std::uniform_real_distribution<double> direction_distribution(0.0, 2.0 * M_PI);

    WindVector gust;
    gust.velocity = std::abs(velocity_distribution(generator));
    gust.direction = direction_distribution(generator);
    return gust;
}