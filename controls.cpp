#include "controls.h"

Controls::Controls(double &targetD, double &targetT, uint8_t encA, uint8_t encB, uint8_t btn)
  : targetD(targetD), targetT(targetT), encA(encA), encB(encB)
{
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(btn, INPUT_PULLUP);

  prevEncState = readEnc();
}

uint8_t Controls::readEnc() {
  return (digitalRead(encA) << 1) | digitalRead(encB);
}

int8_t Controls::encStep() {
  uint8_t currEncState = readEnc();
  uint8_t index = (prevEncState << 2) | currEncState; 

  prevEncState = currEncState;

  return encArr[index];
}

bool Controls::update() {
  bool updated = false;

  uint8_t currEncState = readEnc();

  // if encoder position changed change that
  if(currEncState != prevEncState) {
    if(mode == DIST_SET) {
      targetD += encStep();
    } else {
      targetT += encStep();
    }
    updated = true;
  }

  // if encoder button pressed do the digit change/mode change
}