#include "controls.h"

Controls::Controls(double &targetD, double &targetT, uint8_t encA, uint8_t encB, uint8_t btn)
  : targetD(targetD), targetT(targetT), encA(encA), encB(encB), btnPin(btn)
{
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(btn, INPUT_PULLUP);

  mode = DIST_SET;
  decimal = 0;

  prevEncState = readEnc();
  btnPressed = false;
}

bool Controls::ctrlsActive() {
  return mode != READY_TO_RUN;
}

void Controls::reset() {
  mode = DIST_SET;
  decimal = 0;

  prevEncState = readEnc();
  btnPressed = false;
}

uint8_t Controls::readEnc() {
  return (digitalRead(encA) << 1) | digitalRead(encB);
}

int8_t Controls::encStep(uint8_t currEncState) {
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
      targetD += encStep(currEncState) * pow(0.1, decimal);
    } else {
      targetT += encStep(currEncState) * pow(0.1, decimal);
    }
    updated = true;
  }

  // if encoder button pressed do the digit change/mode change
  bool currBtnState = !digitalRead(btnPin); // bc its input pullup to gnd, low is true and high is false

  if(currBtnState != btnPressed) {
    if(currBtnState) {
      decimal++;

      if(decimal == 4) {
        mode++;
        decimal = 0;
      }

      updated = true;
    }
    btnPressed = currBtnState;
  }

  return updated;
}