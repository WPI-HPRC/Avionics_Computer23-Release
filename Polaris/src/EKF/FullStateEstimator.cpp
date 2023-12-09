#include <EKF/FullStateEstimator.h>


/**
 * @brief Construct a new Quat State Estimator:: Quat State Estimator object
 * 
 * @param initialOrientation 
 * @param dt 
 */
VehicleStateEstimator::VehicleStateEstimator(BLA::Matrix<10> initialState, float dt) {
    this->x = initialState;
    this->x_min = initialState;
    this->dt = dt;
};

/**
 * @brief Run every loop of the state machine to perform the predict and update step of the EKF
 * 
 * @param sensorPacket Sensor Frame
 * @return BLA::Matrix<4> State Vector
 */
BLA::Matrix<10> VehicleStateEstimator::onLoop(SensorFrame sensorPacket) {

    /* Read Data from Sensors and Convert to SI Units */
    // Convert Accel values to m/s/s
    sensorPacket.ac_x = sensorPacket.ac_x * g0;
    sensorPacket.ac_y = sensorPacket.ac_y * g0;
    sensorPacket.ac_z = sensorPacket.ac_z * g0;
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
    BLA::Matrix<10,10> A = measurementJacobian(sensorPacket);

    // Update model covariance from previous state
    BLA::Matrix<10,6> W = updateModelCovariance(sensorPacket);

    // Apply updated model covariance to process noise covariance matrix
    BLA::Matrix<10,10> Q = W * gyroAccelVar * BLA::MatrixTranspose<BLA::Matrix<10,6>>(W);

    // Update Priori Error Covariance
    P_min = A*P*BLA::MatrixTranspose<BLA::Matrix<10,10>>(A) + Q;

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
    BLA::Matrix<6,10> H = updateJacobian(sensorPacket);

    // Compute the kalman gain from the magnetometer covariance readings
    BLA::Matrix<6> v = z - h;
    BLA::Matrix<6,6> S = H * P_min * BLA::MatrixTranspose<BLA::Matrix<6,10>>(H) + R;
    BLA::Matrix<10,6>  K = P_min * BLA::MatrixTranspose<BLA::Matrix<6,10>>(H) * BLA::Inverse(S);

    // Use our kalman gain and magnetometer readings to correct priori orientation
    x = x_min + K*v;
    // x = x_min;

    // Update error covariance matrix
    P = (eye10 - K*H)*P_min;
    // P = P_min;

    // Serial.println("<----- State ----->");
    // for (int i = 0; i < x.Rows; i++) {
    //     for (int j = 0; j < x.Cols; j++) {
    //         Serial.print(String(x(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    Serial.print("QUAT|");
    Serial.print(x(0)); Serial.print(",");
    Serial.print(x(1)); Serial.print(",");
    Serial.print(x(2)); Serial.print(",");
    Serial.println(x(3));

    return this->x;
}

BLA::Matrix<10> VehicleStateEstimator::measurementFunction(SensorFrame sensorPacket) {
    float p = sensorPacket.gy_x; float q = sensorPacket.gy_y; float r = sensorPacket.gy_z;
    BLA::Matrix<3> bodyAccel = {sensorPacket.ac_x, sensorPacket.ac_y, sensorPacket.ac_y};

    BLA::Matrix<4> f_q = {
        x(0) - (dt/2)*p*x(1) - (dt/2)*q*x(2) - (dt/2)*r*x(3),
        x(1) + (dt/2)*p*x(0) - (dt/2)*q*x(3) + (dt/2)*r*x(2),
        x(2) + (dt/2)*p*x(3) + (dt/2)*q*x(0) - (dt/2)*r*x(1),
        x(3) - (dt/2)*p*x(2) + (dt/2)*q*x(1) + (dt/2)*r*x(0)
    };

    BLA::Matrix<3> gravNED = {0, 0,-g0};

    BLA::Matrix<4> quat = {x(0), x(1), x(2), x(3)};

    BLA::Matrix<3,3> rotm = quat2rotm(quat);

    BLA::Matrix<3,3> rotmTranspose = BLA::MatrixTranspose<BLA::Matrix<3,3>>(rotm);

    BLA::Matrix<3> bodyGrav = rotmTranspose * gravNED;

    BLA::Matrix<3> linearAccelBody = bodyAccel - bodyGrav;

    BLA::Matrix<3> accelNED = rotmTranspose * linearAccelBody;

    // Serial.println("<----- Accel NED ----->");
    // for (int i = 0; i < accelNED.Rows; i++) {
    //     for (int j = 0; j < accelNED.Cols; j++) {
    //         Serial.print(String(accelNED(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    BLA::Matrix<3> f_v = {
        x(7) + accelNED(0)*dt,
        x(8) + accelNED(1)*dt,
        x(9) + accelNED(2)*dt,
    };

    BLA::Matrix<3> f_p = {
        x(4) + f_v(0)*dt,
        x(5) + f_v(1)*dt,
        x(6) + f_v(2)*dt,
    };

    // Normalize Quaternion Function
    f_q = f_q / BLA::Norm(f_q);

    BLA::Matrix<10> f = {
        f_q(0),
        f_q(1),
        f_q(2),
        f_q(3),
        f_v(0),
        f_v(1),
        f_v(2),
        f_p(0),
        f_p(1),
        f_p(2)
    };

    return f;
};

BLA::Matrix<10,10> VehicleStateEstimator::measurementJacobian(SensorFrame sensorPacket) {
    float p = sensorPacket.gy_x; float q = sensorPacket.gy_y; float r = sensorPacket.gy_z;

    BLA::Matrix<10,10> A = {
        1, -0.5*dt*p, -0.5*dt*q, -0.5*dt*r, 0, 0, 0, 0, 0, 0,
        0.5*dt*p, 1, 0.5*dt*r, -0.5*dt*q, 0, 0, 0, 0, 0, 0,
        0.5*dt*q, -0.5*dt*r, 1, 0.5*dt*p, 0, 0, 0, 0, 0, 0,
        0.5*dt*r, 0.5*dt*q, -0.5*dt*p, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, dt, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, dt, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0, dt,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    };

    return A;
}
/*
BLA::Matrix<6> VehicleStateEstimator::updateFunction(SensorFrame sensorPacket) {
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

BLA::Matrix<6,10> VehicleStateEstimator::updateJacobian(SensorFrame sensorPacket) {
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

};*/


BLA::Matrix<6> VehicleStateEstimator::updateFunction(SensorFrame sensorPacket) {
    BLA::Matrix<3> r = {
        cos(inclination), 0, sin(inclination)
    };
    // Must normalize vector for corrections step
    r = r / BLA::Norm(r);

    BLA::Matrix<3> G = {
        0, 0, -g0
    }; // NED Gravity Vector
    
    // Must normalize vector for correction step
    G = G / BLA::Norm(G);

    // Normalize quaternion prior to calculation
    BLA::Matrix<4> q = {x_min(0), x_min(1), x_min(2), x_min(3)};
    q = q / BLA::Norm(q);

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

};

BLA::Matrix<6,10> VehicleStateEstimator::updateJacobian(SensorFrame sensorPacket) {
    BLA::Matrix<3> r = {
        cos(inclination), 0, sin(inclination)
    }; // NED Mag vector

    // Must normalize vector for corrections step
    r = r / BLA::Norm(r);

    BLA::Matrix<3> G = {
        0, 0, -g0
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

    BLA::Matrix<6,10> H = {
        gx*qw+gy*qz-gz*qy, gx*qx+gy*qy+gz*qz, -gx*qy+gy*qx-gz*qw, -gx*qz+gy*qw+gz*qx, 0, 0, 0, 0, 0, 0,
        -gx*qz+gy*qw+gz*qx, gx*qy-gy*qx+gz*qw, gx*qx+gy*qy+gz*qz, -gx*qw-gy*qz+gz*qy, 0, 0, 0, 0, 0, 0,
        gx*qy-gy*qx+gz*qw, gx*qz-gy*qw-gz*qx, gx*qw+gy*qz-gz*qy, gx*qx+gy*qy+gz*qz, 0, 0, 0, 0, 0, 0,
        rx*qw+ry*qz-rz*qy, rx*qy+ry*qy+rz*qz, -rx*qy+ry*qx-rz*qw, -rx*qz+ry*qw+rz*qx, 0, 0, 0, 0, 0, 0,
        -rx*qz+ry*qw+rz*qx, rx*qy-ry*qx+rz*qw, rx*qx+ry*qy+rz*qz, -rx*qw-ry*qz+rz*qy, 0, 0, 0, 0, 0, 0,
        rx*qy-ry*qx+rz*qw, rx*qz-ry*qw-rz*qx, rx*qw+ry*qz-rz*qy, rx*qx+ry*qy+rz*qz, 0, 0, 0, 0, 0, 0,
    };

    return H * 2.0f;

};

BLA::Matrix<10,6> VehicleStateEstimator::updateModelCovariance(SensorFrame sensorPacket) {

    BLA::Matrix<10,6> W = {
        -0.5*x(1), -0.5*x(2), -0.5*x(3), 0, 0, 0,
        0.5*x(0), -0.5*x(3), 0.5*x(2), 0, 0, 0,
        0.5*x(3), 0.5*x(0), -0.5*x(1), 0, 0, 0,
        -0.5*x(2), 0.5*x(1), 0.5*x(0), 0, 0, 0,
        0, 0, 0, x(4), 0, 0, 
        0, 0, 0, 0, x(5), 0, 
        0, 0, 0, 0, 0, x(6),
        0, 0, 0, x(4), 0, 0, 
        0, 0, 0, 0, x(5), 0,
        0, 0, 0, 0, 0, x(6)
    };

    return W;
};

BLA::Matrix<3,3> VehicleStateEstimator::quat2rotm(BLA::Matrix<4> q) {
    // q = q / BLA::Norm(q);
    
    float qw = q(0);
    float qx = q(1);
    float qy = q(2);
    float qz = q(3);

    BLA::Matrix<3,3> rotm = {
        qw*qw + qx*qx - qy*qy - qz*qz, 2 * (qx*qy - qw*qz), 2 * (qx*qz + qw*qy),
        2 * (qx*qy + qw*qz), qw*qw - qx*qx + qy*qy - qz*qz, 2 * (qy*qz - qw*qx),
        2 * (qx*qz - qw*qy), 2 * (qw*qx + qy*qz), qw*qw - qx*qx - qy*qy + qz*qz
    };


    return rotm;
};