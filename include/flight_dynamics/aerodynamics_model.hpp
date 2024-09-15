#ifndef AERODYNAMICS_MODEL_HPP
#define AERODYNAMICS_MODEL_HPP

#include <array>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "atmospheric_model.hpp"

/**
 * @class AerodynamicsModel
 * @brief Computes aerodynamic forces and moments on a vehicle based on its state and control surfaces.
 * 
 * This class models the aerodynamic behavior of a vehicle across different flow regimes
 * (subsonic, supersonic, hypersonic) by interpolating aerodynamic coefficients and
 * calculating the resulting forces and moments.
 */
class AerodynamicsModel {
public:
    /**
     * @struct VehicleState
     * @brief Represents the state of the vehicle.
     * 
     * Contains position, velocity, attitude, mass, and reference dimensions of the vehicle.
     */
    struct VehicleState {
        Eigen::Vector3d position;      /**< Position in Earth-centered Earth-fixed (ECEF) frame (meters). */
        Eigen::Vector3d velocity;      /**< Velocity in ECEF frame (meters per second). */
        Eigen::Vector3d attitude;      /**< Attitude angles: roll, pitch, yaw (radians). */
        double mass;                    /**< Mass of the vehicle (kilograms). */
        double reference_area;          /**< Reference area for aerodynamic calculations (square meters). */
        double reference_length;        /**< Reference length for aerodynamic calculations (meters). */
    };

    /**
     * @struct ControlSurfaces
     * @brief Represents the deflections of the vehicle's control surfaces.
     * 
     * Contains deflection angles for elevator, aileron, and rudder.
     */
    struct ControlSurfaces {
        double elevator;   /**< Elevator deflection angle (radians). */
        double aileron;    /**< Aileron deflection angle (radians). */
        double rudder;     /**< Rudder deflection angle (radians). */
    };

    /**
     * @struct AeroForces
     * @brief Represents aerodynamic forces and moments acting on the vehicle.
     * 
     * Contains force vectors (lift, drag, side force) and moment vectors (pitch, roll, yaw).
     */
    struct AeroForces {
        Eigen::Vector3d force;    /**< Aerodynamic forces: Lift, Drag, Side force (Newtons). */
        Eigen::Vector3d moment;   /**< Aerodynamic moments: Pitch, Roll, Yaw moment (Newton-meters). */
    };

    /**
     * @brief Constructs an AerodynamicsModel with a reference to an AtmosphericModel.
     * 
     * @param atm_model Reference to an AtmosphericModel providing atmospheric properties.
     */
    AerodynamicsModel(const AtmosphericModel& atm_model);

    /**
     * @brief Default destructor.
     */
    ~AerodynamicsModel() = default;

    /**
     * @brief Computes the aerodynamic forces acting on the vehicle.
     * 
     * Calculates lift, drag, and side forces based on the vehicle's state and control surface deflections.
     * The computation varies depending on the Mach number (subsonic, supersonic, hypersonic).
     * 
     * @param state Current state of the vehicle.
     * @param controls Current deflections of control surfaces.
     * @return AeroForces Struct containing the computed aerodynamic forces.
     */
    AeroForces computeAeroForces(const VehicleState& state, const ControlSurfaces& controls) const;

    /**
     * @brief Computes the aerodynamic moments acting on the vehicle.
     * 
     * Calculates pitch, roll, and yaw moments based on the vehicle's state and control surface deflections.
     * The computation varies depending on the Mach number (subsonic, supersonic, hypersonic).
     * 
     * @param state Current state of the vehicle.
     * @param controls Current deflections of control surfaces.
     * @return AeroForces Struct containing the computed aerodynamic moments.
     */
    AeroForces computeAeroMoments(const VehicleState& state, const ControlSurfaces& controls) const;

    /**
     * @brief Updates the aerodynamic forces and moments based on control surface deflections.
     * 
     * Applies the effects of control surface deflections (elevator, aileron, rudder) to the existing aerodynamic forces and moments.
     * Ensures control surface deflections are within realistic limits to handle potential nonlinearities.
     * 
     * @param forces Reference to AeroForces struct to be updated.
     * @param state Current state of the vehicle.
     * @param controls Current deflections of control surfaces.
     */
    void updateControlSurfaceEffects(AeroForces& forces, const VehicleState& state, const ControlSurfaces& controls) const;

private:
    const AtmosphericModel& atm_model; /**< Reference to an AtmosphericModel for obtaining atmospheric properties. */

    // Aerodynamic coefficients (these would typically be loaded from a database or file)
    
    // Subsonic flow coefficients
    std::vector<double> cl_alpha;                /**< Lift coefficient vs angle of attack (Subsonic). */
    std::vector<double> cd_alpha;                /**< Drag coefficient vs angle of attack (Subsonic). */
    std::vector<double> cm_alpha;                /**< Pitch moment coefficient vs angle of attack (Subsonic). */
    
    // Supersonic flow coefficients
    std::vector<double> cl_alpha_supersonic;     /**< Lift coefficient vs angle of attack (Supersonic). */
    std::vector<double> cd_alpha_supersonic;     /**< Drag coefficient vs angle of attack (Supersonic). */
    std::vector<double> cm_alpha_supersonic;     /**< Pitch moment coefficient vs angle of attack (Supersonic). */
    
    // Hypersonic flow coefficients
    std::vector<double> cl_alpha_hypersonic;     /**< Lift coefficient vs angle of attack (Hypersonic). */
    std::vector<double> cd_alpha_hypersonic;     /**< Drag coefficient vs angle of attack (Hypersonic). */
    std::vector<double> cm_alpha_hypersonic;     /**< Pitch moment coefficient vs angle of attack (Hypersonic). */
    
    double cy_beta;                /**< Side force coefficient vs sideslip angle. */
    double cn_beta;                /**< Yaw moment coefficient vs sideslip angle. */
    double cl_beta;                /**< Roll moment coefficient vs sideslip angle. */

    // Control surface effectiveness coefficients
    double cl_elevator; /**< Lift coefficient per radian of elevator deflection. */
    double cm_elevator; /**< Pitch moment coefficient per radian of elevator deflection. */
    double cl_aileron;  /**< Roll moment coefficient per radian of aileron deflection. */
    double cn_rudder;   /**< Yaw moment coefficient per radian of rudder deflection. */

    /**
     * @brief Calculates the Mach number based on the vehicle's state.
     * 
     * Computes the speed of the vehicle relative to the local speed of sound.
     * 
     * @param state Current state of the vehicle.
     * @return Mach number (dimensionless).
     */
    double calculateMachNumber(const VehicleState& state) const;

    /**
     * @brief Calculates the angle of attack and sideslip angle.
     * 
     * Transforms the velocity vector from the ECEF frame to the body frame and computes the angles.
     * 
     * @param state Current state of the vehicle.
     * @return std::array containing angle of attack and sideslip angle (radians).
     */
    std::array<double, 2> calculateAngleOfAttackAndSideslip(const VehicleState& state) const;
    
    /**
     * @brief Interpolates aerodynamic coefficients based on angle of attack and Mach number.
     * 
     * Uses cubic Hermite spline interpolation for higher precision.
     * Clamps or extrapolates values if the angle of attack is outside the predefined range.
     * 
     * @param coeff_table Vector containing aerodynamic coefficients at discrete angles of attack.
     * @param alpha Angle of attack (radians).
     * @param mach Mach number (dimensionless).
     * @return Interpolated aerodynamic coefficient.
     */
    double interpolateCoefficient(const std::vector<double>& coeff_table, double alpha, double mach) const;
    
    /**
     * @brief Computes aerodynamic forces for subsonic flow conditions.
     * 
     * Uses subsonic-specific aerodynamic coefficients to calculate lift, drag, and side forces.
     * 
     * @param state Current state of the vehicle.
     * @param alpha Angle of attack (radians).
     * @param beta Sideslip angle (radians).
     * @return AeroForces Struct containing the computed aerodynamic forces.
     */
    AeroForces computeSubsonicAero(const VehicleState& state, double alpha, double beta) const;

    /**
     * @brief Computes aerodynamic forces for supersonic flow conditions.
     * 
     * Uses supersonic-specific aerodynamic coefficients to calculate lift, drag, and side forces.
     * 
     * @param state Current state of the vehicle.
     * @param alpha Angle of attack (radians).
     * @param beta Sideslip angle (radians).
     * @return AeroForces Struct containing the computed aerodynamic forces.
     */
    AeroForces computeSupersonicAero(const VehicleState& state, double alpha, double beta) const;

    /**
     * @brief Computes aerodynamic forces for hypersonic flow conditions.
     * 
     * Uses hypersonic-specific aerodynamic coefficients to calculate lift, drag, and side forces.
     * Accounts for phenomena like shock-boundary layer interactions.
     * 
     * @param state Current state of the vehicle.
     * @param alpha Angle of attack (radians).
     * @param beta Sideslip angle (radians).
     * @return AeroForces Struct containing the computed aerodynamic forces.
     */
    AeroForces computeHypersonicAero(const VehicleState& state, double alpha, double beta) const;
    
    /**
     * @brief Calculates the geodetic altitude based on ECEF position.
     * 
     * Uses the WGS84 ellipsoid model for accurate altitude determination.
     * 
     * @param position_ecef Position vector in ECEF frame (meters).
     * @return Geodetic altitude above the WGS84 ellipsoid (meters).
     */
    double calculateGeodeticAltitude(const Eigen::Vector3d& position_ecef) const;
};

#endif // AERODYNAMICS_MODEL_HPP
