#include "trajectory.h"

Trajectory::Trajectory(double targetD, double targetT) {
  this->targetD = targetD;
  this->targetT = targetT; 

  t_a = (accel * targetT - sqrt(pow(accel * targetT, 2) - (4 * accel * targetD))) / (2*accel);
  t_c = targetT - 2*t_a;
  v_cruise = accel * t_a;

  phase1Dist = getTargetPos(t_a);
  phase2Dist = v_cruise * t_c + phase1Dist;
}

double Trajectory::getTargetVel(double t) {
  t /= 1000000;

  if(t < t_a) {
    return accel * t;
  }
  if(t < t_a + t_c) {
    return v_cruise;
  }
  if(t < 2*t_a + t_c) {
    return -accel * (t - t_a - t_c) + v_cruise;
  }
  return 0;
}

double Trajectory::getTargetPos(double t) {
  t /= 1000000;

  if(t < t_a) {
    return 0.5 * accel * pow(t, 2);
  }
  if(t < t_a + t_c) {
    return v_cruise * (t - t_a) + phase1Dist;
  }
  if(t < 2*t_a + t_c) {
    return -0.5 * accel * pow(t - t_a - t_c, 2) + v_cruise * (t - t_a - t_c) + phase2Dist;
  }
  return targetD; 
}