#include "Controller.h"
#include <Arduino.h>

Controller::Controller() {
    // constructor things


}

// takes in altitude, vertical and lateral velocity
// returns airbrake extension in a range from 0-100
uint8_t Controller::calcAbPct(float alt, int8_t velZ, int8_t velLat) {

    float a = -0.928228913;
    float b = 3.958695493;
    float c = -2.467757547;
    float d = 0.002306584;
    float e = -3.71e-9;
    float f = 0.001035235;
    // where TotalCD = rocketCD + a + b*airbrake_extension + c*airbrake_extension^2
    float dynamicPressure = 0; // write fcn for this
    float dragForce = 0; // write fcn for this

    // A = c;
    float B = b+f*dynamicPressure;
    float C = a+d*dynamicPressure+e*(dynamicPressure*dynamicPressure)-dragForce;

    float extension = (-B+sqrt((B*B)-4*c*C))/(2*c);

    // uint8_t extensionPct = extension * 100 / length;
    uint8_t extensionPct = 0;

    return extensionPct;
}

// takes in state, dt, targetApogee, m, A, currentExtension, currentCD, allowableError, P, altZero, timeZero
float Controller::targetDragCoefficient() {    
    // testCD = currentCD; // data type? lol 
    // this one here'll be a doozy, gotta find a library for that
    // apogee = rk4(state,dt,m,A,currentExtension,testCD,P,alt_0,T_0);

    // float error = abs(1-(apogee/10000));
    // bestCD = testCD;
    // lowestError = error;

    // while(error > allowableError)
    //     if(testCD ~= bestCD)
    //             testCD = testCD + 0.5*(testCD-bestCD);
    //     else
    //         if(apogee < targetApogee)
    //             testCD = 1.05*testCD;
    //         else
    //             testCD = 0.95*testCD;
    //     apogee = rk4(state,dt,m,A,currentExtension,testCD,P,alt_0,T_0);
    //     error = abs(1-(apogee/targetApogee));

    //     if(error < lowestError)
    //         bestCD = testCD;
    //         lowestError = error;
    // targetCd = testCD;
    return 0.0
}

float Controller:apogee() {
// function apogee = rk4(x,dt,m,A,ext,cd,P,alt_0,T_0)
    
//     xCurr = x;
//     while(xCurr(2) < 0)
//         k1 = dt*rocketDynamics(xCurr,m,A,ext,cd,P,alt_0,T_0);
//         k2 = dt*rocketDynamics(xCurr + (k1/2),m,A,ext,cd,P,alt_0,T_0);
//         k3 = dt*rocketDynamics(xCurr + (k2/2),m,A,ext,cd,P,alt_0,T_0);
//         k4 = dt*rocketDynamics(xCurr + k3,m,A,ext,cd,P,alt_0,T_0);
        
//         xCurr = xCurr + ((1/6)*(k1 + 2*k2 + 2*k3 + k4));
//     end
//     apogee = xCurr(1);
    return 0.0
}

// def not void and not a number but some other secret third thing (a fucking vector)
void rocketDynamics() {
// function xDot = rocketDynamics(x,m,A,ext,cd,P,alt_0,T_0) % add P, alt_0, T_0
//     alt = x(1); vx = x(2); vy = x(3);
//     g = -9.8; w = 0.4826; extMax = 0.037389; 

//     A = A + 4*ext*extMaxw; %ext is defined from 0-1
//     rho = density(alt,alt_0,P,T_0);
//     v = [vx,vy];

//     D = 0.5*cd*rho*A.*v.^2;
//     xAcc = -g-D(1)/m;
//     yAcc = -D(2)/m;

//     xVel = vx;
//     xDot = [xVel,xAcc,yAcc];
}

float density(float h, float hZero, float p, float timeZero) {
    float r = 8.314;
    float m = 0.02896;
    float temp = temperature_at_altitude(timeZero,h,hZero);
    float rho = (p*m)/(r*temp);
    return rho;
}

float temperature_at_altitude(float timeZero, float h, float hZero) {
    float l = -6.5;
    float temp = timeZero - (l*(h-hZero));
    return temp;
}