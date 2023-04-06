#include "StateEstimator.h"

stateStruct StateEstimator::getState(float alt, float ac_X, float ac_Y, float ac_Z)
{
  
  
  state.vel_vert = (alt - prevAltitude) / timeStep;
  state.vel_total += sqrt((ac_X*ac_X + ac_Y*ac_Y + ac_Z*ac_Z)) * timeStep;
  state.vel_lat = sqrt(state.vel_total*state.vel_total - state.vel_vert*state.vel_vert);

  prevAltitude = alt;

  return state;
}







// =================================================================================================

// using namespace BLA;

// BLA::Matrix<4, 4> Q = {1, 0,0,0,
//                         0, 1,0,0,
//                         0, 0,1,0,
//                         0, 0,0,1
//                           };

// BLA::Matrix<1, 1> R_Baro = {10};

// BLA::Matrix<1, 1> R_GPS = {100};

// BLA::Matrix<4, 1> X = {
//     5,
//     0,
//     -9.81,
//     0
// };

// BLA::Matrix<4, 4> P = {10, 0,0,0,
//                        0, 1,0,0,
//                        0, 0,1,0,
//                        0, 0,0,100};

// BLA::Matrix<4, 4> I = {1, 0,0,0,
//                        0, 1,0,0,
//                        0, 0,1,0,
//                        0, 0,0,1};

// BLA::Matrix<1, 4> H_Baro = {1, 0, 0, 1};
// BLA::Matrix<1, 4> H_GPS = {1, 0, 0, 0};

// BLA::Matrix<4, 4> F;

// BLA::Matrix<1, 1> Z_Baro;
// BLA::Matrix<4, 1> K_Baro;

// BLA::Matrix<1, 1> Z_GPS;
// BLA::Matrix<4, 1> K_GPS;

// BLA::Matrix<2, 1> B;
// BLA::Matrix<1, 1> U;

// unsigned long currentTime = 0;
// unsigned long prevTime = 0;
// float delT = 0.0f;

// bool isFirstStep = true;

// void StateEstimator::predict(float accel, float loopTime)
// {
//   currentTime = micros();
//   delT = (currentTime - prevTime) / 1000000.0f;
//   loopTime = delT;
//   prevTime = currentTime;

//   if (!isFirstStep)
//   {
//     F = {
//         1, delT,-(delT*delT)/2.0, 0,
//         0, 1, -delT, 0,
//         0, 0, 1, 0,
//         0, 0, 0, 1};

//     B = {delT * delT / 2.0, 
//         delT,
//         0,
//         0};
    
//     U = {accel};

//     X = F * X + B * U;
//     P = F * P * ~F + Q;
//   }
//   isFirstStep = false;
// }

// void StateEstimator::updateBaro(float altitude)
// {
//   Z_Baro = {altitude};
//   K_Baro = P * ~H_Baro * (H_Baro * P * ~H_Baro + R_Baro).Inverse();

//   X = X + K_Baro * (Z_Baro - H_Baro * X);
//   P = (I - K_Baro * H_Baro) * P * (~(I - K_Baro * H_Baro)) + K_Baro * R_Baro * ~K_Baro;
// }

// void StateEstimator::updateGPS(float gpsAltitude)
// {
//   Z_GPS = {gpsAltitude};
//   K_GPS = P * ~H_GPS * (H_GPS * P * ~H_GPS + R_GPS).Inverse();

//   X = X + K_GPS * (Z_GPS - H_GPS * X);
//   P = (I - K_GPS * H_GPS) * P * (~(I - K_GPS * H_GPS)) + K_GPS * R_GPS * ~K_GPS;
// }

// float StateEstimator::getKalmanPosition() {
//   return X(0,0);
// }

// float StateEstimator::getKalmanVelocity() {
//   return X(1,0);
// }

// float StateEstimator::getKalmanGravity() {
//   return X(2,0);
// }

// float StateEstimator::getKalmanBias() {
//   return X(3,0);
// }

// using namespace Eigen;

