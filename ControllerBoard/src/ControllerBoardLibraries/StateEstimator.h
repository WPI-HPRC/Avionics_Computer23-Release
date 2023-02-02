#include <Arduino.h>
#include <eigen.h>
/*
    @brief StateEstimator class for the controller board
    @author Samay Govani
*/
enum EstimatorState{
    PRELAUNCH,
    POSTLAUNCH
};


using namespace Eigen;

class StateEstimator{
    private:
    public:
        /*
            @brief Current state of the rocket
            State:
            0: q0 (scalar component of quaternion)
            1: q1 (x component of quaternion)
            2: q2 (y component of quaternion)
            3: q3 (z component of quaternion)
            4: gyro bias x
            5: gyro bias y
            6: gyro bias z
            7: Altitude
            8: Velocity North
            9: Velocity East
            10: Velocity Down
        */
       Eigen::MatrixXf state = MatrixXf::Zero(11,1);
       
       /*
            @brief Current state of the estimator
            @details The state of the estimator is used to determine what set of sensors/equations to use
            During PRELAUNCH, the IMU+Magnetometer+Barometer are used to estimate the full state using the classic EKF
            During POSTLAUCH, the IMU+Magnetometer+Barometer are used to estimate the velocity, and orientation but the gyro bias is assumed to be constant throughout the flight
       */
       EstimatorState E_state = PRELAUNCH;

        /*
            @brief Constructor for the StateEstimator class
        */
        StateEstimator();

        /*
            @brief Updates the state of the rocket
        */
        void updateState(int16_t Accel_X, int16_t Accel_Y, int16_t Accel_Z, int16_t Gyro_X, int16_t Gyro_Y, int16_t Gyro_Z, int32_t Mag_X, int32_t Mag_Y, int32_t Mag_Z, int32_t Baro_D1, int32_t Baro_D2, int16_t dt_MS){
            if (E_state == PRELAUNCH){
                
            }
            else if (E_state == POSTLAUNCH){
                
            }
        }
        /*
            @brief Updates the state of the rocket when on the pad
            
        */
        void updateState_PreLaunch(float Accel_X, float Accel_Y, float Accel_Z, float Gyro_X, float Gyro_Y, float Gyro_Z, float Mag_X, float Mag_Y, float Mag_Z, float Baro_Pressure, float dt_MS);
        void updateState_PostLaunch(float Accel_X, float Accel_Y, float Accel_Z, float Gyro_X, float Gyro_Y, float Gyro_Z, float Mag_X, float Mag_Y, float Mag_Z, float Baro_Pressure, float dt_MS);
};