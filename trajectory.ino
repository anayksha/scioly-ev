/*
will have to check what type variables have to be for performance
bc using doubles for time and vel might be unnecessary

The setup of the calculations can be found here: https://www.overleaf.com/read/zccntryzrtrq#407659
I am NOT planning to type a derivation into overleaf

TODO: rn distances is precalculated for calculations, consider doing that for
time instead of having time intervals
*/

double Trajectory::t_a;      // time acceleration/deceleration takes
double t_c;      // time cruising at constant speed takes 
double v_cruise; // cruise velocity

// distance traveled between first 1 phases of the trajectory, pre calc for performance
double phase1Dist; // acceleration phase
double phase2Dist; // cruise phase

public:
  Trajectory() {
    t_a = (accel * targetT - sqrt(pow(accel * targetT, 2) - (4 * accel * targetD))) / (2*accel);
    t_c = targetT - 2*t_a;
    v_cruise = accel * t_a;

    phase1Dist = getTargetPos(t_a);
    phase2Dist = v_cruise * t_c + phase1Dist;
  }

  double getTargetVel(double t) {
    // give time in microseconds
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

  double getTargetPos(double t) {
    // give time in microseconds
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
};