#include <EKF/StateEstimator.h>
#include "StateEstimator.h"

/**
 * @brief Construct a new Quat State Estimator:: Quat State Estimator object
 * 
 * @param initialOrientation 
 * @param dt 
 */
QuatStateEstimator::QuatStateEstimator(BLA::Matrix<4> initialOrientation, float dt) {
    this->x = initialOrientation;
    this->x_min = initialOrientation;
    this->dt = dt;
};

/**
 * @brief Run every loop of the state machine to perform the predict and update step of the EKF
 * 
 * @param sensorPacket Sensor Frame
 * @return BLA::Matrix<4> State Vector
 */
BLA::Matrix<4> QuatStateEstimator::onLoop(SensorFrame sensorPacket) {

    /* Read Data from Sensors and Convert to SI Units */
    // Convert Accel values to m/s/s
    sensorPacket.ac_x = sensorPacket.ac_x * 9.81;
    sensorPacket.ac_y = sensorPacket.ac_y * 9.81;
    sensorPacket.ac_z = sensorPacket.ac_z * 9.81;
    // Convert gyro values from deg/s to rad/s
    sensorPacket.gy_x = sensorPacket.gy_x * (PI / 180);
    sensorPacket.gy_y = sensorPacket.gy_y * (PI / 180);
    sensorPacket.gy_z = sensorPacket.gy_z * (PI / 180);

    // Convert Gauss to Tesla
    sensorPacket.X_mag = sensorPacket.X_mag / 10000;
    sensorPacket.Y_mag = sensorPacket.Y_mag / 10000;
    sensorPacket.Z_mag = sensorPacket.Z_mag / 10000;

    /* PREDICTION STEP 
    x_min = f(x,dt)
    A = df(x,dt) / dx
    P_min = A*P*A' + sigma_gyr^2*W*W'
    
    */

    // Apply measurement function to predict priori state of the system
    x_min = measurementFunction(sensorPacket);

    // Take the jacobian to obtain the covariance of the prediction step
    BLA::Matrix<4,4> A = measurementJacobian(sensorPacket);

    // Update model covariance from previous state
    BLA::Matrix<4,3> W = updateModelCovariance(sensorPacket);

    // Apply updated model covariance to process noise covariance matrix
    Q = W * gyroVar * BLA::MatrixTranspose<BLA::Matrix<4,3>>(W);

    // Update Priori Error Covariance
    P_min = A*P*BLA::MatrixTranspose<BLA::Matrix<4,4>>(A) + Q * dt;

    // Apply Soft and Hard Iron Calibration to magnetometer data
    // **IMPORTANT** THIS MAY NEED TO BE RE-CALIBRATED UPON POSITION CHANGE
    BLA::Matrix<3> magVector = {
        sensorPacket.X_mag, sensorPacket.Y_mag, sensorPacket.Z_mag
    };

    magVector = softIronCal * (magVector - hardIronCal);

    // Calculate update function with magnetometer readings to correct orientation
    BLA::Matrix<3> z = magVector;

    BLA::Matrix<3> magNorm = z / BLA::Norm(z);

    BLA::Matrix<3> h = updateFunction(sensorPacket);

    // Take the jacobian to obtain the covariance of the correction function
    BLA::Matrix<3,4> H = updateJacobian(sensorPacket);

    // Compute the kalman gain from the magnetometer covariance readings
    BLA::Matrix<3> v = magNorm - h;
    BLA::Matrix<3,3> S = H * P_min * BLA::MatrixTranspose<BLA::Matrix<3,4>>(H) + R;
    BLA::Matrix<4,3> K = P_min * BLA::MatrixTranspose<BLA::Matrix<3,4>>(H) * BLA::Inverse(S);

    // Use our kalman gain and magnetometer readings to correct priori orientation
    x = x + K*v;

    // Update error covariance matrix
    P = (eye4 - K*H)*P_min;

    // Serial.println("<----- A Matrix ----->");
    // for (int i = 0; i < A.Rows; i++) {
    //     for (int j = 0; j < A.Cols; j++) {
    //         // std::cout << matrix(i, j) << "\t";
    //         Serial.print(String(A(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }


    // Serial.println(heading);

    // Serial.println("<----- A Matrix ----->");
    // for (int i = 0; i < A.Rows; i++) {
    //     for (int j = 0; j < A.Cols; j++) {
    //         // std::cout << matrix(i, j) << "\t";
    //         Serial.print(String(A(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    // Serial.println("<----- Priori State ----->");
    // for (int i = 0; i < x_min.Rows; i++) {
    //     for (int j = 0; j < x_min.Cols; j++) {
    //         // std::cout << matrix(i, j) << "\t";
    //         Serial.print(String(x_min(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    // Serial.println("<----- Update Function ----->");
    // for (int i = 0; i < h.Rows; i++) {
    //     for (int j = 0; j < h.Cols; j++) {
    //         // std::cout << matrix(i, j) << "\t";
    //         Serial.print(String(h(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    // Serial.println("<----- C Matrix ----->");
    // for (int i = 0; i < C.Rows; i++) {
    //     for (int j = 0; j < C.Cols; j++) {
    //         // std::cout << matrix(i, j) << "\t";
    //         Serial.print(String(C(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    // Serial.println("<----- Kalman Gain ----->");
    // for (int i = 0; i < K.Rows; i++) {
    //     for (int j = 0; j < K.Cols; j++) {
    //         // std::cout << matrix(i, j) << "\t";
    //         Serial.print(String(K(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }
    // Serial.println("<----- P Matrix ----->");
    // for (int i = 0; i < P.Rows; i++) {
    //     for (int j = 0; j < P.Cols; j++) {
    //         // std::cout << matrix(i, j) << "\t";
    //         Serial.print(String(P(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }
    // Serial.println("<----------------->");


    Serial.println("<----- Priori State ----->");
    Serial.println("W: " + String(x_min(0)));
    Serial.println("I: " + String(x_min(1)));
    Serial.println("J: " + String(x_min(2)));
    Serial.println("K: " + String(x_min(3)));

    Serial.println("<----- Posterior State ----->");
    Serial.println("W: " + String(x(0)));
    Serial.println("I: " + String(x(1)));
    Serial.println("J: " + String(x(2)));
    Serial.println("K: " + String(x(3)));

    // x = x / BLA::Norm(x);
    x = x / BLA::Norm(x);

    return this->x;
}

BLA::Matrix<4> QuatStateEstimator::measurementFunction(SensorFrame sensorPacket) {
    float p = sensorPacket.gy_x; float q = sensorPacket.gy_y; float r = sensorPacket.gy_z;

    BLA::Matrix<4> f_q = {
        x(0) - (dt/2)*p*x(1) - (dt/2)*q*x(2) - (dt/2)*r*x(3),
        x(1) + (dt/2)*p*x(0) - (dt/2)*q*x(3) + (dt/2)*r*x(2),
        x(2) + (dt/2)*p*x(3) + (dt/2)*q*x(0) - (dt/2)*r*x(1),
        x(3) - (dt/2)*p*x(2) + (dt/2)*q*x(1) + (dt/2)*r*x(0)
    };

    // Normalize Update Function
    f_q = f_q / BLA::Norm(f_q);

    BLA::Matrix<4> f = f_q;

    return f;
};

BLA::Matrix<4, 4> QuatStateEstimator::measurementJacobian(SensorFrame sensorPacket) {
    float p = sensorPacket.gy_x; float q = sensorPacket.gy_y; float r = sensorPacket.gy_z;

    BLA::Matrix<4,4> A = {
        1, -(dt/2)*p, -(dt/2)*q, -(dt/2)*r,
        (dt/2)*p, 1, (dt/2)*r, -(dt/2)*q,
        (dt/2)*q, -(dt/2)*r, 1, (dt/2)*p,
        (dt/2)*r, (dt/2)*q, -(dt/2)*p, 1
    };

    return A;
}

BLA::Matrix<3> QuatStateEstimator::updateFunction(SensorFrame sensorPacket) {
    BLA::Matrix<3> r = {
        cos(inclination), 0, sin(inclination)
    };
    r = r / BLA::Norm(r);

    float rx = r(0);
    float ry = r(1);
    float rz = r(2);
    float qw = x_min(0);
    float qx = x_min(1);
    float qy = x_min(2);
    float qz = x_min(3);

    BLA::Matrix<3> h = {
        rx*(0.5-pow(qy,2)-pow(qz,2))+ry*(qw*qz+qx*qy)+rz*(qx*qz-qw*qy),
        rx*(qx*qy-qw*qz)+ry*(0.5-pow(qx,2)-pow(qz,2))+rz*(qw*qx+qy*qz),
        rx*(qw*qy+qx*qz)+ry*(qy*qz-qw*qx)+rz*(0.5-pow(qx,2)-pow(qy,2))
    };

    return h * 2.0f;

};

BLA::Matrix<3,4> QuatStateEstimator::updateJacobian(SensorFrame sensorPacket) {
    BLA::Matrix<3> r = {
        cos(inclination), 0, sin(inclination)
    };
    r = r / BLA::Norm(r);

    float rx = r(0);
    float ry = r(1);
    float rz = r(2);
    float qw = x_min(0);
    float qx = x_min(1);
    float qy = x_min(2);
    float qz = x_min(3);

    BLA::Matrix<3,4> H = {
        rx*qw+ry*qz-rz*qy, rx*qy+ry*qy+rz*qz, -rx*qy+ry*qx-rz*qw, -rx*qz+ry*qw+rz*qx,
        -rx*qz+ry*qw+rz*qx, rx*qy-ry*qx+rz*qw, rx*qx+ry*qy+rz*qz, -rx*qw-ry*qz+rz*qy, 
        rx*qy-ry*qx+rz*qw, rx*qz-ry*qw-rz*qx, rx*qw+ry*qz-rz*qy, rx*qx+ry*qy+rz*qz
    };

    return H * 2.0f;

};

BLA::Matrix<4,3> QuatStateEstimator::updateModelCovariance(SensorFrame sensorPacket) {

    BLA::Matrix<4,3> W = {
        -x(1), -x(2), -x(3),
        x(0), -x(3), x(2),
        x(3), x(0), -x(1),
        -x(2), x(1), x(0)
    };

    return W * (dt/2);
};