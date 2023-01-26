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
            7: x position
            8: y position
            9: z position
            10: x velocity
            11: y velocity
            12: z velocity
        */
       Eigen::MatrixXf state = MatrixXf::Zero(13,1);
       
       /*
            @brief Current state of the estimator
            @details The state of the estimator is used to determine what set of sensors/equations to use
            During PRELAUNCH, the IMU+Magnetometer+Barometer are used to estimate the full state using the classic EKF
            During POSTLAUCH, the IMU+Magnetometer+Barometer are used to estimate the position, velocity, and orientation but the gyro bias is assumed to be constant throughout the flight
       */
       EstimatorState E_state = PRELAUNCH;

        /*
            @brief Constructor for the StateEstimator class
        */
        StateEstimator();

        /*
            @brief Updates the state of the rocket
            @details This function is called every loop and updates the state of the rocket using the current sensor data
        */
        void updateState(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ, float magX, float magY, float magZ, float baroAlt, float dt){
            if (E_state == PRELAUNCH){
                
            }
            else if (E_state == POSTLAUNCH){
                
            }
        }
};