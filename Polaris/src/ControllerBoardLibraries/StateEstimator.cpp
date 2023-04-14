#include "StateEstimator.h"

StateStruct StateEstimator::getState(float alt, float ac_x, float ac_y, float ac_z)
{
  // add new altitude value to buffer
  altitudeBuffer[BuffInd] = alt;
  // take running average value
  float sum = 0.0;
  for (int i = 0; i < 10; i++)
  {
      sum += altitudeBuffer[i];
  }
  float avg_alt = sum / 10.0;

  BuffInd = (BuffInd + 1) % 10;

  // Calculate vertical velocity by differentiating altitude over time
  state.vel_vert = (avg_alt - prev_avg_alt) / timeStep;

  // Calculate velocity in each axis by integrating acceleration over time
  vel_x += ac_x * timeStep;
  vel_y += ac_y * timeStep;
  vel_z += ac_z * timeStep;

  // Calculate magnitude of total velocity
  state.vel_total = sqrt((vel_x * vel_x + vel_y *vel_y + vel_z * vel_z));

  // Calculate lateral velocity from Pythagorean Theorm
  state.vel_lat = sqrt(state.vel_total*state.vel_total - state.vel_vert*state.vel_vert);

  // Store altitude for next estimate
  prev_avg_alt = avg_alt;

  // Return StateStruct with vertical, lateral, and total velocities
  return state;
}
