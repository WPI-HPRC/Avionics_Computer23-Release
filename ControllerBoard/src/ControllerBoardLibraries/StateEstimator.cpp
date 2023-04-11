#include "StateEstimator.h"

StateStruct StateEstimator::getState(float alt, float ac_X, float ac_Y, float ac_Z)
{
  
  state.vel_vert = (alt - prevAltitude) / timeStep;
  state.vel_total += sqrt((ac_X*ac_X + ac_Y*ac_Y + ac_Z*ac_Z)) * timeStep;
  state.vel_lat = sqrt(state.vel_total*state.vel_total - state.vel_vert*state.vel_vert);

  prevAltitude = alt;

  return state;
}
