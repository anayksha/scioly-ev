/*
will have to check what type variables have to be for performance
bc using doubles for time and vel might be unnecessary

The setup of the calculations can be found here: https://www.overleaf.com/read/zccntryzrtrq#407659
I am NOT planning to type a derivation into overleaf

TODO: rn distances is precalculated for calculations, consider doing that for
time instead of having time intervals
*/

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Arduino.h>
#include <math.h>

extern const double accel;

class Trajectory {

  double targetD;
  double targetT;

  double t_a;      // time acceleration/deceleration takes
  double t_c;      // time cruising at constant speed takes 
  double v_cruise; // cruise velocity

  // distance traveled between first 1 phases of the trajectory, pre calc for performance
  double phase1Dist; // acceleration phase
  double phase2Dist; // cruise phase

  public:

    // computes all of the instance variables
    Trajectory(double targetD, double targetT);

    // Returns target velocity in ______/s when given time in microseconds
    double getTargetVel(double t);

    // Returns target position in ______/s when given time in microseconds
    double getTargetPos(double t);
};

#endif