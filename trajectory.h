/*
This is a file for the trajectory class, basically a trajectory object can be created from target time and 
target distance and this object has methods that give the ideal position or ideal velocity the ev should be at
given a certain amount of time

The setup of the calculations can be found here: https://www.overleaf.com/read/zccntryzrtrq#407659
And the desmos graph to visualze and  verify the solution can be found here: https://www.desmos.com/calculator/avxj6yh4fy
I am NOT planning to type the derivation itself into overleaf

TODO: rn some distances is precalculated for calculations, consider doing that for
time instead of having time intervals for performance
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