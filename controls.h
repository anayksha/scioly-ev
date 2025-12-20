#ifndef CONTROLS_H
#define CONTROLS_H

#include <Arduino.h>
#include <Encoder.h>

// edits target values in place with an encoder w/ button
class Controls {
  // control mode vars
  static constexpr uint8_t DIST_SET = 0;
  static constexpr uint8_t TIME_SET = 1;

  uint8_t mode = 0;

  // encoder vars
  const uint8_t encA;
  const uint8_t encB;

  // cant use second enc object bc it tweaks the shit out when theres 1 enc
  // with interrupts and one without
  static constexpr int8_t encArr[] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
  };

  uint8_t prevEncState;

  // hehe things we modify
  double &targetD;
  double &targetT;

  public:
    // initializes instance vars and Encoder obj to lock in
    Controls(double &targetD, double &targetT, uint8_t encA, uint8_t encB, uint8_t btn);
    
    // reads enc for value 0-3
    uint8_t readEnc();

    // returns 1 or -1 based on change in encoder rotation
    int8_t encStep();

    // updates targetD and targetT based on encoder and button input and returns true/false
    // depending on if something was changed
    bool update();
};

#endif