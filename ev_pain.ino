#include <Encoder.h>
// #include <RobojaxBTS7960.h>
#include <BTS7960.h>
#include <PID_v1.h>

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

const double targetDist = 10.0;
const double targetTime = 12.0;

// PID pain
const double K_p_pos = 0;
const double K_i_pos = 0;
const double K_d_pos = 0;

const double K_p_vel = 0;
const double K_i_vel = 0;
const double K_d_vel = 0;

BTS7960 motor(L_EN, R_EN, L_PWM, R_PWM);

void setup() {
  Serial.begin(9600);

  motor.Enable();
}

void loop() {

}
