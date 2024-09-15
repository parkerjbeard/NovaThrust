#ifndef ATMOSPHERIC_MODEL_HPP
#define ATMOSPHERIC_MODEL_HPP

#include <array>
#include <cmath>
#include <random>

/**
 * @class AtmosphericModel
 * @brief Models atmospheric conditions and wind effects based on altitude and time.
 *
 * The AtmosphericModel class provides methods to calculate atmospheric properties
 * such as density, temperature, and pressure at various altitudes. It also computes
 * wind effects, including both steady winds and gusts, using advanced wind models.
 */
class AtmosphericModel {
public:
    /**
     * @struct AtmosphericProperties
     * @brief Represents the basic atmospheric properties at a given altitude.
     *
     * This structure holds the density, temperature, and pressure of the atmosphere
     * at a specific altitude.
     */
    struct AtmosphericProperties {
        double density;      ///< Air density in kilograms per cubic meter (kg/m³)
        double temperature;  ///< Temperature in Kelvin (K)
        double pressure;     ///< Pressure in Pascals (Pa)
    };

    /**
     * @struct WindVector
     * @brief Represents wind velocity and direction.
     *
     * This structure holds the wind velocity and its direction in radians.
     */
    struct WindVector {
        double velocity;     ///< Wind speed in meters per second (m/s)
        double direction;    ///< Wind direction in radians
    };

    /**
     * @brief Constructs an AtmosphericModel instance.
     *
     * Initializes the random number generator used for simulating wind gusts.
     */
    AtmosphericModel();

    /**
     * @brief Default destructor for AtmosphericModel.
     *
     * Since no dynamic memory is allocated, the default destructor is sufficient.
     */
    ~AtmosphericModel() = default;

    /**
     * @brief Retrieves atmospheric properties at a specified altitude.
     *
     * Calculates the density, temperature, and pressure of the atmosphere at the given altitude.
     *
     * @param altitude Altitude in meters where atmospheric properties are to be calculated.
     * @return AtmosphericProperties structure containing density, temperature, and pressure.
     * @throws std::out_of_range if altitude is outside the valid range (0 to 100,000 meters).
     */
    AtmosphericProperties getAtmosphericProperties(double altitude) const;

    /**
     * @brief Computes wind effects at a specified altitude and time.
     *
     * Combines steady wind profiles with gust models to provide a comprehensive wind vector.
     *
     * @param altitude Altitude in meters where wind effects are to be calculated.
     * @param time Current time in seconds, used for temporal variations in wind.
     * @return WindVector structure containing the resultant wind velocity and direction.
     * @throws std::out_of_range if altitude is outside the valid range (0 to 100,000 meters).
     */
    WindVector computeWindEffects(double altitude, double time) const;

    /**
     * @brief Calculates the speed of sound based on temperature.
     *
     * Uses the adiabatic index for air to determine the speed of sound at a given temperature.
     *
     * @param temperature Temperature in Kelvin.
     * @return Speed of sound in meters per second (m/s).
     */
    double getSpeedOfSound(double temperature) const;

private:
    // Constants
    static constexpr double R = 287.05;   ///< Specific gas constant for air (J/(kg·K))
    static constexpr double g = 9.80665;  ///< Gravitational acceleration (m/s²)
    static constexpr double L = 0.0065;   ///< Temperature lapse rate (K/m)
    static constexpr double P0 = 101325;  ///< Sea level standard atmospheric pressure (Pa)
    static constexpr double T0 = 288.15;  ///< Sea level standard temperature (K)

    /**
     * @brief Calculates air density based on pressure and temperature.
     *
     * Applies the ideal gas law to determine density.
     *
     * @param pressure Pressure in Pascals (Pa).
     * @param temperature Temperature in Kelvin (K).
     * @return Air density in kilograms per cubic meter (kg/m³).
     */
    double calculateDensity(double pressure, double temperature) const;

    // Troposphere calculations

    /**
     * @brief Calculates pressure in the troposphere based on altitude and temperature.
     *
     * Uses the barometric formula tailored for the tropospheric layer.
     *
     * @param altitude Altitude in meters.
     * @param temperature Temperature in Kelvin.
     * @return Atmospheric pressure in Pascals (Pa).
     */
    double calculatePressureTroposphere(double altitude, double temperature) const;

    /**
     * @brief Calculates temperature in the troposphere based on altitude.
     *
     * Applies a linear temperature lapse rate to determine temperature changes with altitude.
     *
     * @param altitude Altitude in meters.
     * @return Temperature in Kelvin (K).
     */
    double calculateTemperatureTroposphere(double altitude) const;

    // Stratosphere calculations

    /**
     * @brief Calculates pressure in the stratosphere based on altitude and temperature.
     *
     * Uses an exponential decay model suitable for the stratospheric layer.
     *
     * @param altitude Altitude in meters.
     * @param temperature Temperature in Kelvin.
     * @return Atmospheric pressure in Pascals (Pa).
     */
    double calculatePressureStratosphere(double altitude, double temperature) const;

    /**
     * @brief Calculates temperature in the stratosphere based on altitude.
     *
     * Represents the temperature inversion characteristic of the lower stratosphere.
     *
     * @param altitude Altitude in meters.
     * @return Temperature in Kelvin (K).
     */
    double calculateTemperatureStratosphere(double altitude) const;

    // Mesosphere calculations

    /**
     * @brief Calculates pressure in the mesosphere based on altitude and temperature.
     *
     * Uses an exponential decay model tailored for the mesosphere.
     *
     * @param altitude Altitude in meters.
     * @param temperature Temperature in Kelvin.
     * @return Atmospheric pressure in Pascals (Pa).
     */
    double calculatePressureMesosphere(double altitude, double temperature) const;

    /**
     * @brief Calculates temperature in the mesosphere based on altitude.
     *
     * Models the decreasing temperature trend observed in the mesosphere.
     *
     * @param altitude Altitude in meters.
     * @return Temperature in Kelvin (K).
     */
    double calculateTemperatureMesosphere(double altitude) const;

    /**
     * @brief Calculates a sophisticated wind model based on altitude and time.
     *
     * Placeholder for integrating real atmospheric data or complex mathematical models.
     *
     * @param altitude Altitude in meters.
     * @param time Current time in seconds.
     * @return WindVector structure containing wind velocity and direction.
     */
    WindVector calculateAdvancedWindModel(double altitude, double time) const;

    /**
     * @brief Generates a gust model based on the current time.
     *
     * Utilizes random distributions to simulate gust velocity and direction.
     *
     * @param time Current time in seconds.
     * @return WindVector structure containing gust velocity and direction.
     */
    WindVector calculateGustModel(double time) const;

    /**
     * @brief Random number generator for simulating wind gusts.
     *
     * Mutable to allow modification within const methods.
     */
    mutable std::default_random_engine generator;
};

#endif // ATMOSPHERIC_MODEL_HPP