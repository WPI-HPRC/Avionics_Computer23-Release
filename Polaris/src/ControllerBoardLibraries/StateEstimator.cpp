#include "StateEstimator.h"

StateStruct StateEstimator::getState(float alt, float ac_x, float ac_y, float ac_z)
{
  // add new altitude value to buffer
  altitudeBuffer[altBuffInd] = alt;
  // take running average value
  float sum = 0.0;
  for (int i = 0; i < 10; i++)
  {
      sum += altitudeBuffer[i];
  }
  float avg_alt = sum / 10.0;

  altBuffInd = (altBuffInd + 1) % 10;

  // Calculate vertical velocity by differentiating altitude over time
  state.vel_vert = (avg_alt - prev_avg_alt) / timeStep;

  // Calculate velocity in each axis by integrating acceleration over time
  vel_x += ac_x * timeStep;
  vel_y += ac_y * timeStep;
  vel_z += ac_z * timeStep;

  // Calculate magnitude of total velocity
  avgBuffer[avgBuffInd] = sqrt((vel_x * vel_x + vel_y *vel_y + vel_z * vel_z));
  state.vel_total = avgBuffer[avgBuffInd] - avgBuffer[(avgBuffInd + 1) % 5];
  altBuffInd = (altBuffInd + 1) % 5; 

  // Calculate lateral velocity from Pythagorean Theorm
  if (state.vel_total > abs(state.vel_vert)) {
    state.vel_lat = sqrt(state.vel_total*state.vel_total - abs(state.vel_vert*state.vel_vert));
  } else {
    state.vel_lat = 0;
  }

  // Store altitude for next estimate
  prev_avg_alt = avg_alt;

  // Return StateStruct with vertical, lateral, and total velocities
  return state;
}
