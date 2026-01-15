#include "controls.h"

Controls::Controls(double &targetD, double &targetT, uint8_t encA, uint8_t encB, uint8_t btn)
  : targetD(targetD), targetT(targetT), btnPin(btn)
{
  ctrlEnc.attachFullQuad(encA, encB);

  pinMode(btn, INPUT_PULLUP);
  
  reset();
}

bool Controls::ctrlsActive() {
  return mode != READY_TO_RUN;
}

void Controls::reset() {
  mode = DIST_SET;
  decimal = 0;

  ctrlEnc.clearCount();

  btnPressed = false;
}

bool Controls::update() {
  bool updated = false;

  long encVal = ctrlEnc.getCount();

  // if encoder position changed change that
  if(abs(encVal) >= 4) {
    // ts is kinda broken at the moment bc it doesn't loop through digits,
    // it only adds and subtracts, so it can change the upper digits
    if(mode == DIST_SET) {
      targetD += (encVal / 4) * pow(0.1, decimal);
    } else if(mode == TIME_SET) { // unnecessary "if" but for if i accidentally call update() when mode is READY_TO_RUN
      targetT += (encVal / 4) * pow(0.1, decimal);
    }
    
    ctrlEnc.setCount(encVal % 4);
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
  if(updated) {
    Serial.println("enc val: " + String(encVal));
    Serial.println("btn state: " + String(currBtnState));
  }
  return updated;
}