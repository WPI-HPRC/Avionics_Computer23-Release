#include <EKF/StateEstimator.h>
#include "StateEstimator.h"

QuatStateEstimator::QuatStateEstimator(BLA::Matrix<4> initialOrientation, float dt) {
    this->x = initialOrientation;
    this->x_min = initialOrientation;
    this->dt = dt;
};

BLA::Matrix<4> QuatStateEstimator::onLoop(SensorFrame sensorPacket) {

    BLA::Matrix<4> dx = measurementFunction(sensorPacket);

    x_min = x_min + dx * dt;

    BLA::Matrix<4,4> A = measurementJacobian(sensorPacket);

    P_min = P_min + (A * P_min * BLA::MatrixTranspose<BLA::Matrix<4,4>>(A)) + (Q * dt);

    BLA::Matrix<3> h = updateFunction(sensorPacket);

    BLA::Matrix<3,4> C = updateJacobian(sensorPacket);

    // BLA::Matrix<3,3> k_pre = C*P_min*BLA::MatrixTranspose<BLA::Matrix<3,4>>(C) + R;

    // BLA::Matrix<4,3> K = P_min * BLA::MatrixTranspose<BLA::Matrix<3,4>>(C) * BLA::Inverse(k_pre);

    // BLA::Matrix<3> y = {sensorPacket.ac_x, sensorPacket.ac_y, sensorPacket.ac_z};

    // y = y * G;

    // x = x_min + K * (y - h);

    // P = (BLA::Eye<4,4>() - K * C) * P;

    // BLA::Matrix<3> euler = quatToEuler(x_min);
    // Serial.println("Roll: " + String(euler(0)));
    // Serial.println("Pitch: " + String(euler(1)));
    // Serial.println("Yaw: " + String(euler(2)));

    return this->x_min / BLA::Norm(x_min);
}

BLA::Matrix<4> QuatStateEstimator::measurementFunction(SensorFrame sensorPacket) {
    float p = sensorPacket.gy_x;
    float q = sensorPacket.gy_y;
    float r = sensorPacket.gy_z;

    BLA::Matrix<4,4> dx = {
        0, -p, -q, -r,
        p, 0, r, q,
        q, -r, 0, p,
        r, q, -p, 0
    };

    return dx*x;

};

BLA::Matrix<4, 4> QuatStateEstimator::measurementJacobian(SensorFrame sensorPacket)
{
    float p = sensorPacket.gy_x;
    float q = sensorPacket.gy_y;
    float r = sensorPacket.gy_z;

    BLA::Matrix<4,4> A = {
        0, -p, -q, -r,
        p, 0, r, q,
        q, -r, 0, p,
        r, q, -p, 0
    };

    return A;
}

BLA::Matrix<3> QuatStateEstimator::updateFunction(SensorFrame sensorPacket) {
    BLA::Matrix<3> y = {sensorPacket.ac_x, sensorPacket.ac_y, sensorPacket.ac_z};

    float w = x(0);
    float i = x(1);
    float j = x(2);
    float k = x(3);

    BLA::Matrix<3,3> h = {
        1-2*(pow(j,2)+pow(k,2)), 2*(i*j-w*k), 2*(i*k+w*k),
        2*(i*j+w*k), 1-2*(pow(i,2)+pow(k,2)), 2*(j*k-w*k),
        2*(i*k-w*j), 2*(j*k+w*i), 1-2*(pow(i,2)+pow(j,2))
    };

    return h*y;
};

BLA::Matrix<3,4> QuatStateEstimator::updateJacobian(SensorFrame sensorPacket) {
    float w = x(0);
    float i = x(1);
    float j = x(2);
    float k = x(3);

    float y1 = sensorPacket.ac_x;
    float y2 = sensorPacket.ac_y;
    float y3 = sensorPacket.ac_z;

    BLA::Matrix<3, 4> C = {
    -2 * k * (y3 - y2), 2 * (j * y3 + k * y1), 2 * (i * y2 - 2 * j * y1), y3 * (2 * i + 2 * w) - 4 * k * y1 - 2 * w * y2,
    2 * k * (y1 - y3), 2 * (j * y1 - 2 * i * y2), 2 * (i * y1 + k * y3), y3 * (2 * j - 2 * w) - 4 * k * y2 + 2 * w * y1,
    2 * (i * y2 - j * y1), 2 * (k * y1 - 2 * i * y3 + w * y2), 2 * (k * y2 - w * y1), 2 * (i * y1 + j * y2 - 2 * k * y1)
    };

    return C / BLA::Norm(x);
};