/**
 * @file EKF.h
 * @author Daniel Pearson (djpearson@wpi.edu)
 * @brief This program provides an Extended Kalman Filter for predicting vehicle orientation as a quaternion.  The filter successfully fuses data from the Accelerometer, Gyroscope and Magnetometer.  For more information contact Dan, Colette or Nikhil.
 * @version 1.0
 * @date 2023-11-23
 * 
 * @copyright Copyright (c) Worcester Polytechnic Institute High Power Rocketry Club 2023
 */

#pragma once

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <SensorBoardLibraries/SensorBoard.hpp>

class VehicleStateEstimator {
public:
    /**
     * @brief Construct a new Quat State Estimator:: Quat State Estimator object
     * 
     * @param x0 Initial State
     * @param dt Timestep
     */
    VehicleStateEstimator(BLA::Matrix<10> initialState, float dt);

    /**
     * @brief Called every loop to update vehicle state
     * 
     * @param sensorPacket Sensorboard frame to pass new data to EKF
     * @return BLA::Matrix<10> Corrected state after sensor fusion
     */
    BLA::Matrix<10> onLoop(SensorFrame sensorPacket);
    
    BLA::Matrix<3,3> quat2rotm(BLA::Matrix<4> q);

private:

    /**
     * @brief Uses gyro and accel dynamic model of vehicle to predict current state
     * 
     * @param sensorPacket Sensorboard frame to pass new data to EKF 
     * @return BLA::Matrix<4> Integrated predicated state
     */
    BLA::Matrix<10> measurementFunction(SensorFrame sensorPacket);

    /**
     * @brief Performs jacobian of current prediction to obtain the covariance of the change
     * 
     * @param u Gyroscope data vector 
     * @return BLA::Matrix<4,4> Covariance of dynamic model
     */
    BLA::Matrix<10,10> measurementJacobian(SensorFrame sensorPacket);

    /**
     * @brief Correction function applied to predicted orientation to "correct" or "verify" integrated readings weighting the validity of the sensors in the process
     * 
     * @return BLA::Matrix<float, 6,1> Unit vector of expected state in NED (North-East-Down) model
     */
    BLA::Matrix<6> updateFunction(SensorFrame sensorPacket);

    /**
     * @brief Correction function covariance for weighting readings
     * 
     * @return Matrix<float, 6,4> Covariance of correction model
     */
    BLA::Matrix<6,10> updateJacobian(SensorFrame sensorPacket);

    /**
     * @brief Function to return the predicted error of the dynamic model for updating the Dynamic Model Error - Q
     * 
     * @return Matrix<float, 4,3> Quaternion Error Covariance
     */
    BLA::Matrix<10,6> updateModelCovariance(SensorFrame sensorPacket);

    const float gyroVariance = 0.00489/sqrt(40); // [Rad/s]
    const float magVariance = 0.008; // [T]
    const float  accelVariance = 0.00069/sqrt(40); // [m/s/s]

    float dt = 0.025;

    /* Magnetometer Calibration Values - Calculated using MATLAB */
    BLA::Matrix<3,3> softIronCal = {
        3.5838, 0.8795, -1.1902,
        0.8795, 0.7596, -0.7211,
        -1.1902, -0.7211, 1.2511
    };

    BLA::Matrix<3,1> hardIronCal = {
        pow(1,7)*1.3071, pow(1,7)*1.3130, pow(1,7)*1.3042   
    };

    BLA::Matrix<10,10> P = {
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    }; // Process Error Covariance

    BLA::Matrix<10,10> P_min = {
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    }; // Process Error Covariance

    const BLA::Matrix<6,6> R = {
        pow(accelVariance,2), 0, 0, 0, 0, 0,
        0, pow(accelVariance, 2), 0, 0, 0, 0,
        0, 0, pow(accelVariance, 2), 0, 0, 0,
        0, 0, 0, pow(magVariance, 2), 0, 0,
        0, 0, 0, 0, pow(magVariance, 2), 0,
        0, 0, 0, 0, 0, pow(magVariance, 2)
    }; // Sensor Noise Covariance - Accel and Mag

    const BLA::Matrix<6,6> gyroAccelVar = {
        pow(gyroVariance, 2), 0, 0, 0, 0, 0,
        0, pow(gyroVariance, 2), 0, 0, 0, 0,
        0, 0, pow(gyroVariance, 2), 0, 0, 0,
        0, 0, 0, pow(accelVariance, 2), 0, 0,
        0, 0, 0, 0, pow(accelVariance, 2), 0,
        0, 0, 0, 0, 0, pow(accelVariance, 2)
    };
    
    const BLA::Matrix<10,10> eye10 = {
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    }; // 10 Element Identity Matrix

    BLA::Matrix<10> x;
    BLA::Matrix<10> x_min;

    /* Earth Constants*/
    float inclination = 66.546 * (PI/180); // [Rad]
    constexpr static float g0 = 9.80665; // [m/s/s] Gravitational Acceleration
    constexpr static float rho_inf = 1.22; // [kg/m^3] Air Density at Sea Level

};