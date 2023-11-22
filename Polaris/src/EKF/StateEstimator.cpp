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

    // Convert Gauss to microTesla
    sensorPacket.X_mag = sensorPacket.X_mag * 100;
    sensorPacket.Y_mag = sensorPacket.Y_mag * 100;
    sensorPacket.Z_mag = sensorPacket.Z_mag * 100;

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
    P_min = A*P*BLA::MatrixTranspose<BLA::Matrix<4,4>>(A) + Q;

    // Apply Soft and Hard Iron Calibration to magnetometer data
    // **IMPORTANT** THIS MAY NEED TO BE RE-CALIBRATED UPON POSITION CHANGE
    BLA::Matrix<3> magVector = {
        sensorPacket.X_mag, sensorPacket.Y_mag, sensorPacket.Z_mag
    };
    
    // Serial.println(String(magVector(0))+","+String(magVector(1))+","+String(magVector(2)));

    magVector = softIronCal * (magVector - hardIronCal);

    BLA::Matrix<3> accelVector = {sensorPacket.ac_x,sensorPacket.ac_y,sensorPacket.ac_z};
    // Normalize Accel and Mag for use in correction step
    magVector = magVector / BLA::Norm(magVector);
    accelVector = accelVector / BLA::Norm(accelVector);

    // Calculate update function with magnetometer readings to correct orientation
    BLA::Matrix<6> z = {
        accelVector(0), accelVector(1), accelVector(2), magVector(0), magVector(1), magVector(2)
    };

    BLA::Matrix<6> h = updateFunction(sensorPacket);

    // Take the jacobian to obtain the covariance of the correction function
    BLA::Matrix<6,4> H = updateJacobian(sensorPacket);

    // Compute the kalman gain from the magnetometer covariance readings
    BLA::Matrix<6> v = z - h;
    BLA::Matrix<6,6> S = H * P_min * BLA::MatrixTranspose<BLA::Matrix<6,4>>(H) + R;
    BLA::Matrix<4,6>  K = P_min * BLA::MatrixTranspose<BLA::Matrix<6,4>>(H) * BLA::Inverse(S);

    // Use our kalman gain and magnetometer readings to correct priori orientation
    x = x_min + K*v;
    // x = x_min;

    // Update error covariance matrix
    P = (eye4 - K*H)*P_min;
    BLA::Matrix<4> tmp = K*v;
    // P = P_min;

    // Serial.println("<----- K*v Matrix ----->");
    // for (int i = 0; i < tmp.Rows; i++) {
    //     for (int j = 0; j < tmp.Cols; j++) {
    //         Serial.print(String(h(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    // Serial.println("<----- P_MIN Matrix ----->");
    // for (int i = 0; i < P_min.Rows; i++) {
    //     for (int j = 0; j < P_min.Cols; j++) {
    //         Serial.print(String(h(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    // Serial.println("<----- P Matrix ----->");
    // for (int i = 0; i < P.Rows; i++) {
    //     for (int j = 0; j < P.Cols; j++) {
    //         Serial.print(String(h(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    // Serial.print(x(0)); Serial.print(",");
    // Serial.print(x(1)); Serial.print(",");
    // Serial.print(x(2)); Serial.print(",");
    // Serial.println(x(3));


    // Serial.println("<----- Priori State ----->");
    // Serial.println("W: " + String(x_min(0)));
    // Serial.println("I: " + String(x_min(1)));
    // Serial.println("J: " + String(x_min(2)));
    // Serial.println("K: " + String(x_min(3)));

    // Serial.println("<----- State ----->");
    // Serial.println("W: " + String(x(0)));
    // Serial.println("I: " + String(x(1)));
    // Serial.println("J: " + String(x(2)));
    // Serial.println("K: " + String(x(3)));

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

    // Normalize Quaternion Function
    f_q = f_q / BLA::Norm(f_q);

    BLA::Matrix<4> f = f_q;

    return f;
};

BLA::Matrix<4,4> QuatStateEstimator::measurementJacobian(SensorFrame sensorPacket) {
    float p = sensorPacket.gy_x; float q = sensorPacket.gy_y; float r = sensorPacket.gy_z;

    BLA::Matrix<4,4> A = {
        1, -(dt/2)*p, -(dt/2)*q, -(dt/2)*r,
        (dt/2)*p, 1, (dt/2)*r, -(dt/2)*q,
        (dt/2)*q, -(dt/2)*r, 1, (dt/2)*p,
        (dt/2)*r, (dt/2)*q, -(dt/2)*p, 1
    };

    return A;
}

BLA::Matrix<6> QuatStateEstimator::updateFunction(SensorFrame sensorPacket) {
    BLA::Matrix<3> r = {
        cos(inclination), 0, sin(inclination)
    }; // NED Mag vector
    // BLA::Matrix<3> r = {
    //     19911.6, -4905.1, 47269.5
    // };

    // Must normalize vector for corrections step
    r = r / BLA::Norm(r);

    BLA::Matrix<3> G = {
        0, 0, -9.81
    }; // NED Gravity Vector
    
    // Must normalize vector for correction step
    G = G / BLA::Norm(G);

    // Normalize quaternion prior to calculation
    BLA::Matrix<4> q = {x_min(0), x_min(1), x_min(2), x_min(3)};
    q = q / BLA::Norm(q);

    // float rx = r(0);
    // float ry = r(1);
    // float rz = r(2);
    // float gx = G(0);
    // float gy = G(1);
    // float gz = G(2);

    // float qw = q(0);
    // float qx = q(1);
    // float qy = q(2);
    // float qz = q(3);

    BLA::Matrix<3,3> quatRotm = quat2rotm(q);

    BLA::Matrix<3> a_hat = BLA::MatrixTranspose<BLA::Matrix<3,3>>(quatRotm) * G;
    BLA::Matrix<3> m_hat = BLA::MatrixTranspose<BLA::Matrix<3,3>>(quatRotm) * r;

    BLA::Matrix<6> h = {
        a_hat(0),
        a_hat(1),
        a_hat(2),
        m_hat(0),
        m_hat(1),
        m_hat(2)
    };

    return h;


    // BLA::Matrix<6> h = {
    //     gx*(0.5-pow(qy,2)-pow(qz,2))+gy*(qw*qz+qx*qy)+gz*(qx*qz-qw*qy),
    //     gx*(qx*qy-qw*qz)+gy*(0.5-pow(qx,2)-pow(qz,2))+gz*(qw*qx+qy*qz),
    //     gx*(qw*qy+qx*qz)+gy*(qy*qz-qw*qx)+gz*(0.5-pow(qx,2)-pow(qy,2)),
    //     rx*(0.5-pow(qy,2)-pow(qz,2))+ry*(qw*qz+qx*qy)+rz*(qx*qz-qw*qy),
    //     rx*(qx*qy-qw*qz)+ry*(0.5-pow(qx,2)-pow(qz,2))+rz*(qw*qx+qy*qz),
    //     rx*(qw*qy+qx*qz)+ry*(qy*qz-qw*qx)+rz*(0.5-pow(qx,2)-pow(qy,2))
    // };

    // return h * 2.0f;

};

BLA::Matrix<6,4> QuatStateEstimator::updateJacobian(SensorFrame sensorPacket) {
    BLA::Matrix<3> r = {
        cos(inclination), 0, sin(inclination)
    }; // NED Mag vector

    // Must normalize vector for corrections step
    r = r / BLA::Norm(r);

    BLA::Matrix<3> G = {
        0, 0, -9.81
    }; // NED Gravity Vector
    
    // Must normalize vector for correction step
    G = G / BLA::Norm(G);

    // Normalize quaternion prior to calculation
    BLA::Matrix<4> q = {x_min(0), x_min(1), x_min(2), x_min(3)};
    q = q / BLA::Norm(q);

    float rx = r(0);
    float ry = r(1);
    float rz = r(2);
    float gx = G(0);
    float gy = G(1);
    float gz = G(2);

    float qw = q(0);
    float qx = q(1);
    float qy = q(2);
    float qz = q(3);

    BLA::Matrix<6,4> H = {
        gx*qw+gy*qz-gz*qy, gx*qx+gy*qy+gz*qz, -gx*qy+gy*qx-gz*qw, -gx*qz+gy*qw+gz*qx,
        -gx*qz+gy*qw+gz*qx, gx*qy-gy*qx+gz*qw, gx*qx+gy*qy+gz*qz, -gx*qw-gy*qz+gz*qy, 
        gx*qy-gy*qx+gz*qw, gx*qz-gy*qw-gz*qx, gx*qw+gy*qz-gz*qy, gx*qx+gy*qy+gz*qz,
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

BLA::Matrix<3,3> QuatStateEstimator::quat2rotm(BLA::Matrix<4> q) {
    q = q / BLA::Norm(q);
    
    float qw = q(0);
    float qx = q(1);
    float qy = q(2);
    float qz = q(3);

    BLA::Matrix<3,3> rotm = {
        pow(qw,2)+pow(qx,2)-pow(qy,2)-pow(qz,2), 2*(qx*qy-qw*qz), 2*(qx*qx+qw*qy),
        2*(qx*qy+qw*qz), pow(qw,2)-pow(qx,2)+pow(qy,2)-pow(qz,2), 2*(qy*qz-qw*qx),
        2*(qx*qz-qw*qy), 2*(qw*qx+qy*qz), pow(qw,2)-pow(qx,2)-pow(qy,2)+pow(qz,2)
    };

    return rotm;
};
