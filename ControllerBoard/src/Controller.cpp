#include "Controller.h"
#include <Arduino.h>

Controller::Controller() {
    // constructor things
}

// takes in altitude, vertical and lateral velocity
// returns airbrake extension in a range from 0-100
uint8_t Controller::calcAbPct(float alt, int8_t velLat, int8_t velVert) {
    
    // calculate dynamic pressure and drag force
    float dynamicPressure = pressure_at_altitude(pZero,alt,tZero);
    float dragForce = targetDragCoefficient(alt,velLat,velVert,dt,pZero,0,tZero);

    float a = 0.699638467114056;
    float b = -0.000844592841487885;
    float c = 0.283556333385388;
    float d = 6.71177099854179e-09;
    float e = -0.00506902401028055;
    float f = 0.000013347923590557;
    // where TotalCD = rocketCD + a + b*airbrake_extension + c*airbrake_extension^2

    float extension = a + b*dragForce + c*dynamicPressure + d*(dragForce*dragForce) + (e*dynamicPressure*dynamicPressure) + f*dragForce*dynamicPressure;
    ext = extension;
    cd = dragForce;
    uint8_t extensionPct = round(extension/length * 100);
    
    return extensionPct;
}

float Controller::targetDragCoefficient(float alt, int8_t velLat, int8_t velVert, int8_t dt, float pZero, float altZero, float tZero) {    
    float testCD = cd; // data type? lol 
    // calculate what our predicted apogee will be rn
    float apogee = rk4(alt,velLat,velVert,dt,testCD,pZero,altZero,tZero);

    float error = abs(1-((targetApogee-alt)/(apogee-alt)));
    float delta = 0.00001; 
    bool oppSide = false;
    float lastCD = testCD;
    float lastApogee = apogee;
    float lowestError = error;

    int8_t i = 1;
    int8_t maxIterations = 4;
    float apogeeDelta;
    
    while(error > allowableError && i < maxIterations) {
        apogeeDelta = rk4(alt,velLat,velVert,dt,testCD+delta,pZero,altZero,tZero);
        testCD = testCD - (((targetApogee-apogee)*delta)/(apogee-apogeeDelta)); 

        if(testCD < 0) {
            testCD = 0;
            break;
        }

        apogee = rk4(alt,velLat,velVert,dt,testCD,pZero,altZero,tZero);
        error = abs(1-((targetApogee-alt)/(apogee-alt))); 
        i = i+1;
    }
    return testCD;
}

float Controller::rk4(float alt, int8_t velLat, int8_t velVert, int8_t dt, float currCD, float pZero, float altZero, float tZero) {    
    // init k1,k2,k3,k4 as arrays
    float xCurr[3] = {alt,velLat,velVert}; // probably not right
    float w = 0.4826; 
    float extMax = 0.037389; 
    float k1[3], k2[3], k3[3], k4[3];
    float A = area + 4*ext*extMax*w; // assume this is *
    float P = pressure_at_altitude(pZero,alt,tZero);
    float rho = density(alt,tZero,P);
    float v = sqrt((velLat*velLat)+(velVert+velVert));

    while(xCurr[1] < 0) {
        k1[0] = (float) velLat * dt;
        k1[1] = dt*calcLatAcc(velLat, v, currCD, rho, A);
        k1[2] = dt*calcVertAcc(velVert, v, currCD, rho, A);

        velLat = velLat + (k1[1]/2);
        v = sqrt((velLat*velLat)+(velVert+velVert));
        k2[0] = (float) velLat * dt + (k1[1]/2);
        k2[1] = dt*calcLatAcc(velLat, v, currCD, rho, A);
        k2[2] = dt*calcVertAcc(velVert + (k1[2]/2), v, currCD, rho, A);

        velLat = velLat + (k2[1]/2);
        v = sqrt((velLat*velLat)+(velVert+velVert));
        k3[0] = (float) velLat * dt + (k2[1]/2);
        k3[1] = dt*calcLatAcc(velLat, v, currCD, rho, A);
        k3[2] = dt*calcVertAcc(velVert + (k2[2]/2), v, currCD, rho, A);

        velLat = velLat + k3[1];
        v = sqrt((velLat*velLat)+(velVert+velVert));
        k4[0] = (float) velLat * dt + k3[1];
        k4[1] = dt*calcLatAcc(velLat, v, currCD, rho, A);
        k4[2] = dt*calcVertAcc(velVert + k3[2], v, currCD, rho, A);
        
        xCurr[0] = xCurr[0] + ((1/6)*(k1[0] + 2*k2[0] + 2*k3[0] + k4[0]));
        xCurr[1] = xCurr[1] + ((1/6)*(k1[1] + 2*k2[1] + 2*k3[1] + k4[1]));
        xCurr[2] = xCurr[2] + ((1/6)*(k1[2] + 2*k2[2] + 2*k3[2] + k4[2]));
    }
    float apogee = xCurr[0];
    return apogee;
}

float Controller::calcLatAcc(float velLat, float v, float currCD, float rho, float A) {
    float Dx = 0.5*currCD*rho*A*v*v;
    float xAcc = -g-(Dx*velLat/v)/m;
    return xAcc;
}

float Controller::calcVertAcc(float velVert, float v, float currCD, float rho, float A) {
    float Dy = 0.5*currCD*rho*A*v*v;
    float yAcc = -(Dy*velVert/v)/m;
    return yAcc;
}

// calculates density based off temperature
float Controller::density(float alt, float tZero, float p) {
    float temp = temperature_at_altitude(tZero,alt);
    float rho = (p*M)/(R*temp);
    return rho;
}

// calculates temperature, takes in original temperature, current height, and initial height
float Controller::temperature_at_altitude(float tZero, float alt) {
    float temp = tZero - l*alt;
    return temp;
}

// calculates dynamic pressure
// takes in P, height, initial temperature
float Controller::pressure_at_altitude(float P, float h, float T) {
    P = P*exp((-g*M*h)/(R*T));
}

// sets initial pressure and temperature
float Controller::setInitPressureTemp(float initP, float initT) {
    pZero = initP;
    tZero = initT;
}

/*
coef_drag_or = RAW_DATA_FULL{DATA_START:DATA_END, COL_VERT_ACCEL}; % nice, same as the velocity vector

launch_alt = 0;
rocketMass = 24.829;
r_specific = 287.05;
cel_to_kelvin = 273.15;

accel_drag = accel + grav;
temperature = temperature + cel_to_kelvin;
pressure = pressure .* 100;

%%

rho = pressure ./ (r_specific .* temperature); %3.31e-9 * altitude.^2 - 1.14e-4.*altitude + 1.22;

coef_drag = (2 * rocketMass .* accel_drag) ./ (rho * crossSectionalArea .* velocity.^2)


*/