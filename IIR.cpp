#include "IIR.h"

IIR::IIR(double samplePd, double timeConst)
  : samplePd(samplePd), timeConst(timeConst)
{
  // hehe nothing to do here
}

double IIR::getFilteredVal(double newMeasurement) {
  prevFilteredVal = alpha * newMeasurement + prevFilteredWeightage * prevFilteredVal;
  return prevFilteredVal;
}