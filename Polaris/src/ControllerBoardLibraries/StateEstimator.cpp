#include "StateEstimator.h"

StateStruct StateEstimator::getState(float alt, float ac_total)
{
  
  state.vel_vert = (alt - prevAltitude) / timeStep;
  state.vel_total += ac_total * timeStep;
  state.vel_lat = sqrt(state.vel_total*state.vel_total - state.vel_vert*state.vel_vert);

  prevAltitude = alt;

  return state;
}
