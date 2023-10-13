#pragma once

#include "Arduino.h"
#include <BasicLinearAlgebra.h>
#include <SensorBoardLibraries/SensorBoard.hpp>

class QuatStateEstimator {
    public:

    QuatStateEstimator(BLA::Matrix<4> initialOrientation, float dt);

    BLA::Matrix<4> onLoop(SensorFrame sensorPacket);

    BLA::Matrix<4> getState() {
        return x;
    };
    
    private:
    constexpr static int initialLoopIters = 1000;

    constexpr static float gyroVariance = 0.000262;

    float dt = 0.0;

    BLA::Matrix<4,4> P = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };

    BLA::Matrix<4,4> P_min = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };

    const BLA::Matrix<3,3> R = {
        pow(gyroVariance, 2), 0.0, 0.0,
        0.0, pow(gyroVariance, 2), 0.0,
        0.0, 0.0, pow(gyroVariance, 2)
    };

    const BLA::Matrix<4,4> Q = {
        pow(0.0001, 2), 0.0, 0.0, 0.0,
        0.0,  pow(0.0001, 2), 0.0, 0.0,
        0.0, 0.0, pow(0.0001, 2), 0.0,
        0.0, 0.0, 0.0, pow(0.001, 2)
    };

    BLA::Matrix<4> measurementFunction(SensorFrame sensorPacket);
    BLA::Matrix<4,4> measurementJacobian(SensorFrame sensorPacket);

    BLA::Matrix<3> updateFunction(SensorFrame sensorPacket);
    BLA::Matrix<4,3> QuatStateEstimator::updateJacobian(SensorFrame sensorPacket);

    BLA::Matrix<4> x;
    BLA::Matrix<4> x_min;
};