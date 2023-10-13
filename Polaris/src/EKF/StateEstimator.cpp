#include <EKF/StateEstimator.h>
#include "StateEstimator.h"

QuatStateEstimator::QuatStateEstimator(BLA::Matrix<4> initialOrientation, float dt) {
    this->x = initialOrientation;
    this->x_min = initialOrientation;
    this->dt = dt;
};

BLA::Matrix<4> QuatStateEstimator::onLoop(SensorFrame sensorPacket) {

    // Convert Accel values to m/s/s
    sensorPacket.ac_x = sensorPacket.ac_x * 9.81;
    sensorPacket.ac_y = sensorPacket.ac_y * 9.81;
    sensorPacket.ac_z = sensorPacket.ac_z * 9.81;
    // Convert gyro values from deg/s to rad/s
    sensorPacket.gy_x = sensorPacket.gy_x * (PI / 180);
    sensorPacket.gy_y = sensorPacket.gy_y * (PI / 180);
    sensorPacket.gy_z = sensorPacket.gy_z * (PI / 180);

    BLA::Matrix<4> u = {sensorPacket.gy_x, sensorPacket.gy_y, sensorPacket.gy_z};
    BLA::Matrix<3> y = {sensorPacket.ac_x, sensorPacket.ac_y, sensorPacket.ac_z};
    
    x_min = x + measurementFunction(sensorPacket) * dt;

    BLA::Matrix<4,4> A = measurementJacobian(sensorPacket);

    P_min = P_min + A * P * BLA::MatrixTranspose<BLA::Matrix<4,4>>(A) + Q*dt;

    BLA::Matrix<3> h = updateFunction(sensorPacket);
    
    BLA::Matrix<3,4> C = updateJacobian(sensorPacket);

    BLA::Matrix<3,3> K_1 = C*P*BLA::MatrixTranspose<BLA::Matrix<3,4>>(C) + R;

    BLA::Matrix<4,3> K = P * BLA::MatrixTranspose<BLA::Matrix<3,4>>(C) * BLA::Inverse(K_1);

    x = x_min + K * (y - h);

    x = x / BLA::Norm(x);

    BLA::Matrix<4,4> eye4 = {
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1
    };

    P = (eye4 - (K * C)) * P;
    
    // Convert values to m/s/s and rad/s
    // sensorPacket.ac_x = sensorPacket.ac_x;
    // sensorPacket.ac_y = sensorPacket.ac_y;
    // sensorPacket.ac_z = sensorPacket.ac_z;
    // sensorPacket.gy_x = sensorPacket.gy_x * (PI / 180);
    // sensorPacket.gy_y = sensorPacket.gy_y * (PI / 180);
    // sensorPacket.gy_z = sensorPacket.gy_z * (PI / 180);

    // BLA::Matrix<4> dx = measurementFunction(sensorPacket);

    // x_min = x_min + dx * dt;

    // BLA::Matrix<4,4> A = measurementJacobian(sensorPacket);

    // P_min = P_min + A * P * BLA::MatrixTranspose<BLA::Matrix<4,4>>(A) + Q * dt;

    // BLA::Matrix<3> h = updateFunction(sensorPacket);

    // BLA::Matrix<3,4> C = updateJacobian(sensorPacket);

    // BLA::Matrix<3,3> k_pre = C*P*BLA::MatrixTranspose<BLA::Matrix<3,4>>(C) + R;

    // BLA::Matrix<4,3> K = P * BLA::MatrixTranspose<BLA::Matrix<3,4>>(C) * BLA::Inverse(k_pre);

    // BLA::Matrix<3> y = {sensorPacket.ac_x, sensorPacket.ac_y, sensorPacket.ac_z};


    Serial.println("<----- A Matrix ----->");
    for (int i = 0; i < A.Rows; i++) {
        for (int j = 0; j < A.Cols; j++) {
            // std::cout << matrix(i, j) << "\t";
            Serial.print(String(A(i,j)) + "\t");
        }
        Serial.println("");
    }

    Serial.println("<----- C Matrix ----->");
    for (int i = 0; i < C.Rows; i++) {
        for (int j = 0; j < C.Cols; j++) {
            // std::cout << matrix(i, j) << "\t";
            Serial.print(String(C(i,j)) + "\t");
        }
        Serial.println("");
    }
    Serial.println("<----- H Matrix ----->");
    for (int i = 0; i < h.Rows; i++) {
        for (int j = 0; j < h.Cols; j++) {
            // std::cout << matrix(i, j) << "\t";
            Serial.print(String(h(i,j)) + "\t");
        }
        Serial.println("");
    }
    Serial.println("<----- P Matrix ----->");
    for (int i = 0; i < P.Rows; i++) {
        for (int j = 0; j < P.Cols; j++) {
            // std::cout << matrix(i, j) << "\t";
            Serial.print(String(P(i,j)) + "\t");
        }
        Serial.println("");
    }
    Serial.println("<----------------->");
    

    // x = x + K * (y - h);

    // Serial.println("W: " + String(x(0)));
    // Serial.println("I: " + String(x(1)));
    // Serial.println("J: " + String(x(2)));
    // Serial.println("K: " + String(x(3)));

    // BLA::Matrix<4,4> eye4 = {
    //     1, 0, 0, 0,
    //     0, 1, 0, 0,
    //     0, 0, 1, 0,
    //     0, 0, 0, 1
    // };


    // P = (eye4 - K * C) * P;

    // BLA::Matrix<3> euler = quatToEuler(x_min);
    // Serial.println("Roll: " + String(euler(0)));
    // Serial.println("Pitch: " + String(euler(1)));
    // Serial.println("Yaw: " + String(euler(2)));

    return this->x;
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

    return (dx*x / BLA::Norm(x));

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

    float val = 0.5;

    A = A * val;

    return A;
}

BLA::Matrix<3> QuatStateEstimator::updateFunction(SensorFrame sensorPacket) {
    BLA::Matrix<3> y = {sensorPacket.ac_x, sensorPacket.ac_y, sensorPacket.ac_z};

    x = x / BLA::Norm(x);

    float w = x(0);
    float i = x(1);
    float j = x(2);
    float k = x(3);

    BLA::Matrix<3,3> rot = {
        1-2*(pow(j,2)+pow(k,2)), 2*(i*j-w*k), 2*(i*k+w*k),
        2*(i*j+w*k), 1-2*(pow(i,2)+pow(k,2)), 2*(j*k-w*k),
        2*(i*k-w*j), 2*(j*k+w*i), 1-2*(pow(i,2)+pow(j,2))
    };

    return rot*y;
};

BLA::Matrix<3,4> QuatStateEstimator::updateJacobian(SensorFrame sensorPacket) {
    float w = x(0);
    float i = x(1);
    float j = x(2);
    float k = x(3);

    BLA::Matrix<4> q = {w,i,j,k};
    float q_norm = BLA::Norm(q);

    w = w / q_norm;
    i = i / q_norm;
    j = j / q_norm;
    k = k / q_norm;

    BLA::Matrix<3, 4> C;

    C(0, 0) = -4 * j * j - 4 * k * k;
    C(0, 1) = 2 * (i * j - w * k);
    C(0, 2) = 2 * (i * k + w * j);
    C(0, 3) = -2 * (2 * j * j + 2 * k * k);
    
    C(1, 0) = 2 * (i * j + w * k);
    C(1, 1) = -4 * i * i - 4 * k * k;
    C(1, 2) = 2 * (j * k - w * k);
    C(1, 3) = -2 * (2 * i * i + 2 * k * k);
    
    C(2, 0) = 2 * (i * k - w * j);
    C(2, 1) = 2 * (j * k + w * i);
    C(2, 2) = -4 * i * i - 4 * j * j;
    C(2, 3) = -2 * (2 * i * i + 2 * j * j);

    return C / q_norm;
};