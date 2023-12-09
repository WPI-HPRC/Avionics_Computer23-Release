#pragma once

#include "Arduino.h"
#include <BasicLinearAlgebra.h>
#include <SensorBoardLibraries/SensorBoard.hpp>

/**
 * @author @frostydev99 - Daniel Pearson
 * @brief Quaternion State Estimator performing gyroscope and magnetometer sensor fusion to predict vehicle orientation
 */
class QuatStateEstimator {
    public:

    QuatStateEstimator(BLA::Matrix<4> initialOrientation, float dt);

    BLA::Matrix<4> onLoop(SensorFrame sensorPacket);

    BLA::Matrix<3,3> quat2rotm(BLA::Matrix<4> q);
    
    private:
    constexpr static int initialLoopIters = 1000;

    // Input variance from sensor data sheet
    const float gyroVariance = 0.00489/sqrt(40); // [Rad/s]
    const float magVariance = 0.008; // [T]
    const float accelVariance = 0.00069/sqrt(40); // [m/s/s]

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

    /* Magnetic Field Constants for Earth at WPI */
    // float inclination = 66.546;
    float inclination = 66.546 * (PI/180); // [Rad]

    BLA::Matrix<4,4> P = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    }; // Process Error Covariance

    BLA::Matrix<4,4> P_min = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    }; // Process Error Covariance

    const BLA::Matrix<6,6> R = {
        pow(accelVariance,2), 0, 0, 0, 0, 0,
        0, pow(accelVariance, 2), 0, 0, 0, 0,
        0, 0, pow(accelVariance, 2), 0, 0, 0,
        0, 0, 0, pow(magVariance, 2), 0, 0,
        0, 0, 0, 0, pow(magVariance, 2), 0,
        0, 0, 0, 0, 0, pow(magVariance, 2)
    }; // Sensor Noise Covariance - Accel and Mag

    // const BLA::Matrix<3,3> R = {
    //     pow(magVariance, 2), 0.0, 0.0,
    //     0.0, pow(magVariance, 2), 0.0,
    //     0.0, 0.0, pow(magVariance, 2)
    // }; // Sensor Noise Covariance - Magnetometer

    const BLA::Matrix<3,3> gyroVar = {
        pow(gyroVariance, 2), 0, 0,
        0, pow(gyroVariance, 2), 0,
        0, 0, pow(gyroVariance, 2)
    };

    BLA::Matrix<4,4> Q = {
        0.01, 0, 0, 0,
        0, 0.01, 0, 0,
        0, 0, 0.01, 0,
        0, 0, 0, 0.01
    }; // Dynamic Model Covariance 
    
    const BLA::Matrix<4,4> eye4 = {
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1,
    }; // 4 Element Identity Matrix

    BLA::Matrix<4> measurementFunction(SensorFrame sensorPacket);
    BLA::Matrix<4,4> measurementJacobian(SensorFrame sensorPacket);

    BLA::Matrix<6> updateFunction(SensorFrame sensorPacket);
    BLA::Matrix<6,4> updateJacobian(SensorFrame sensorPacket);

    BLA::Matrix<4,3> updateModelCovariance(SensorFrame sensorPacket);

    BLA::Matrix<4> x;
    BLA::Matrix<4> x_min;

    float G = 9.81;

    BLA::Matrix<4> quaternionMultiplication(BLA::Matrix<4> q1, BLA::Matrix<4> q2);
};