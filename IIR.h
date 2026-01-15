#ifndef IIR_H
#define IIR_H

class IIR {
  const double samplePd;
  const double timeConst;
  const double alpha = samplePd / (timeConst + samplePd);
  const double prevFilteredWeightage = 1.0 - alpha;

  double prevFilteredVal = 0;

  public:
    IIR(double samplePd, double timeConst);

    double getFilteredVal(double newMeasurement);
};

#endif