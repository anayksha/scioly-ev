#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <BTS7960.h>
#include <PID_v1.h>

#include "trajectory.h"

// rotary encoder pins
#define ENC_A 2
#define ENC_B 3

// motor controller pins TODO: fix numebers
#define R_EN -1
#define L_EN -1
#define R_PWM -1
#define L_PWM -1

// button pins
#define START_BTN_PIN -1

// TODO: add LCD and rotary encoder and stuff

// movement vars?
const int CPR = 480;

const double wheelDia = 6.0325; // in cm
const double wheelCircuference = wheelDia * 3.14159;

const double targetD = 10.0;
const double targetT = 12.0;

const double accel = 2;

// PID pain
const double K_p_pos = 0;
const double K_i_pos = 0;
const double K_d_pos = 0;
const long posPIDInterval = 15000; // in microseconds

const double K_p_vel = 0;
const double K_i_vel = 0;
const double K_d_vel = 0;
const long velPIDInterval = 2000; // in microseconds

// random shi
unsigned long lastPosPIDTime = 0; // in microseconds
unsigned long lastVelPIDTime = 0; // in microseconds

bool running; // run the PID part of the loop and dont check for other input

Encoder motorEnc(ENC_A, ENC_B);

BTS7960 motor(L_EN, R_EN, L_PWM, R_PWM);

// class Trajectory;
Trajectory* traj = nullptr; // this is bad and needs to be changed

void setup() {
  Serial.begin(9600);

  // reset encoder value
  motorEnc.write(0);

  // configure motor
  motor.Enable();

  // configure start button
  pinMode(START_BTN_PIN, INPUT_PULLUP);

  running = false;
}

void loop() {
  if(!running){
    delay(10);

    if(digitalRead(START_BTN_PIN) == LOW) {
      running = true;
      motorEnc.write(0);
      motor.Enable();

      lastPosPIDTime = 0;
      lastVelPIDTime = 0;

      delete traj;
      traj = new Trajectory();
    }
  } else {
    // might wanna move this to some separate function
    unsigned long time = micros();

    if(time - lastPosPIDTime >= posPIDInterval) {
      // pos loop

      lastPosPIDTime = time;
    }

    if(time - lastVelPIDTime >= velPIDInterval) {
      // vel loop

      lastVelPIDTime = time;
    }
  }
}