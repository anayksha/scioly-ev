/*
basically an object that helps read a rotary encoder w/ button so target time
and target distance can be set while the vehicle is active instead of preprogrammed
*/
#ifndef CONTROLS_H
#define CONTROLS_H

#include <Arduino.h>
#include <ESP32Encoder.h>

// edits target values in place with an encoder w/ button
class Controls {
  // control mode vars
  static constexpr uint8_t DIST_SET = 0;
  static constexpr uint8_t TIME_SET = 1;
  static constexpr uint8_t READY_TO_RUN = 2;

  uint8_t mode;
  uint8_t decimal;

  // encoder vars
  ESP32Encoder ctrlEnc;

  // button vars
  const uint8_t btnPin;
  bool btnPressed;

  // hehe things we modify
  double &targetD;
  double &targetT;

  public:
    // initializes instance vars to lock in
    Controls(double &targetD, double &targetT, uint8_t encA, uint8_t encB, uint8_t btn);

    // true if still setting stuff, false if ready to run
    bool ctrlsActive();

    // resets controls so u can go for another run
    void reset();

    // updates targetD and targetT based on encoder and button input and returns true/false
    // depending on if something was changed
    bool update();
};

#endif