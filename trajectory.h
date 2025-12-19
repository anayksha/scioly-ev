/*
will have to check what type variables have to be for performance
bc using doubles for time and vel might be unnecessary

The setup of the calculations can be found here: https://www.overleaf.com/read/zccntryzrtrq#407659
And the desmos graph to visualze and  verify the solution can be found here: https://www.desmos.com/calculator/avxj6yh4fy
I am NOT planning to type the derivation itself into overleaf

TODO: rn distances is precalculated for calculations, consider doing that for
time instead of having time intervals
*/

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Arduino.h>
#include <math.h>

extern const double accel;

class Trajectory {

  double targetD; // in counts
  double targetT; // in seconds

  double t_a;      // time acceleration/deceleration takes
  double t_c;      // time cruising at constant speed takes 
  double v_cruise; // cruise velocity

  // distance traveled between first 1 phases of the trajectory, pre calc for performance
  double phase1Dist; // acceleration phase
  double phase2Dist; // cruise phase

  public:

    // computes all of the instance variables given targetD and targetT in counts and seconds
    Trajectory(long targetD, long targetT);

    // Returns target velocity in counts/s when given time in seconds
    double getTargetVel(double t);

    // Returns target position in counts when given time in seconds
    long getTargetPos(double t);
};

#endif