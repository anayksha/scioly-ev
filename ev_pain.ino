/*
hehe I love PID trust

also once the arduino is on for like 35 min the ev will prob start tweaking the shit out bc
i think micros() overflows and will probably have to restart arduino
*/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <BTS7960.h>
#include <PID_v1.h>

#include "trajectory.h"
#include "controls.h"

// rotary encoder pins
#define MTR_ENC_A 2
#define MTR_ENC_B 3

#define CTRL_ENC_A 4
#define CTRL_ENC_B 5
#define CTRL_ENC_BTN 6

// motor controller pins
#define R_EN 11
#define L_EN 8
#define R_PWM 10
#define L_PWM 9

// button pins
#define START_BTN_PIN 7

// screen pins
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C // TODO USE I2C SCANNER TO FIGURE OUT WHAT IT ACC IS!!

// movement vars?
constexpr unsigned int CPR = 480;

constexpr double wheelDia = 6.0325;  // in cm
constexpr double wheelCircumference = wheelDia * 3.14159;

double targetDInM = 10.0;
unsigned long targetD; // target dist in counts, set right after onboard setting is done
double targetT = 12.0;

constexpr double accelInM = 1.2; // m/s^2
constexpr double accel = accelInM * 100 / wheelCircumference * CPR;  // in counts per s squared
constexpr double maxSpdInM = 1.5; // m/s
constexpr double maxSpd = maxSpdInM * 100 / wheelCircumference * CPR;

// PID pain
constexpr double Kp_pos = 0;
constexpr double Ki_pos = 0;
constexpr double Kd_pos = 0;
constexpr unsigned int posPIDInterval = 15000;  // in microseconds

constexpr double Kp_vel = 0;
constexpr double Ki_vel = 0;
constexpr double Kd_vel = 0;
constexpr unsigned int velPIDInterval = 2000;  // in microseconds

double posPIDIn, posPIDOut, posPIDSetpt;
double velPIDIn, velPIDOut, velPIDSetpt;

// random timing/constrol shi
// I set the last times to negative numbers to make sure the first PID loop that runs triggers
// both pos and vel loops, and to make sure it runs right after start button pressed 
long lastPosPIDTime = -2 * posPIDInterval;  // in microseconds
long lastVelPIDTime = -2 * velPIDInterval;  // in microseconds
unsigned long startTime;

long lastEncVal = 0;

bool running = false;  // run the PID part of the loop and dont check for other input

// objects for literally everything
Encoder motorEnc(MTR_ENC_A, MTR_ENC_B);

BTS7960 motor(L_EN, R_EN, L_PWM, R_PWM);

Controls controls(targetDInM, targetT, CTRL_ENC_A, CTRL_ENC_B, CTRL_ENC_BTN);

Trajectory* traj = nullptr;  // idk what pointers are so idk if i should be using them bc memory leaks or some shi

PID posPID(&posPIDIn, &posPIDOut, &posPIDSetpt, Kp_pos, Ki_pos, Kd_pos, DIRECT);
PID velPID(&velPIDIn, &velPIDOut, &velPIDSetpt, Kp_vel, Ki_vel, Kd_vel, DIRECT);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// drive motor left or right given a value from -255 to 255
void driveMotor(int pwm) {
  if(pwm >= 0) {
    motor.TurnRight(pwm);
  } else {
    motor.TurnLeft(pwm);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("------------Beginning Setup------------"));

  // reset encoder value
  motorEnc.write(0);
  Serial.println(F("encoder configured"));

  // configure motor
  motor.Enable();
  Serial.println(F("motor configured"));

  // configure start button
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  Serial.println(F("start button configured"));

  // configure PID
  posPID.SetOutputLimits(-maxSpd, maxSpd);
  velPID.SetOutputLimits(-255, 255);
  posPID.SetMode(AUTOMATIC);
  velPID.SetMode(AUTOMATIC);
  Serial.println(F("PID configured"));

  // configure display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("yeah its over ts screen didnt work"));
    for(;;); // loop forever ig
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.setCursor(0, 0);
  display.println(F("display configured"));
  display.display();

  // reset vars
  running = false;
  lastPosPIDTime = -2 * posPIDInterval;
  lastVelPIDTime = -2 * velPIDInterval;
  lastEncVal = 0;
  Serial.println(F("variables reset"));

  Serial.println(F("------------Setup Complete------------"));
}

void loop() {
  if(running) {
    // might wanna move all this to some separate function
    unsigned long runTime = micros() - startTime;

    // pos loop
    // might want to check if lastPos(& Vel)PIDTime is 0
    if(runTime - lastPosPIDTime >= posPIDInterval) {
      lastPosPIDTime = runTime;

      posPIDSetpt = traj->getTargetPos(runTime / 1000000.0);
      posPIDIn = motorEnc.read();

      posPID.Compute(); // sets posPIDOut in-place btw
    }

    // vel loop
    if(runTime - lastVelPIDTime >= velPIDInterval) {
      long currEncVal = motorEnc.read();
      velPIDIn = (currEncVal - lastEncVal) / (runTime - lastVelPIDTime); // calc curr velocity as fast as possible
      lastEncVal = currEncVal;

      lastVelPIDTime = runTime; // reset time early so intervals are accurate

      velPIDSetpt = traj->getTargetVel(runTime / 1000000.0) + posPIDOut;

      posPID.Compute(); // sets velPIDOut in-place btw

      driveMotor(velPIDOut);
    }

    // TODO: prob need a better termination condition
    if(motorEnc.read() >= targetD) {
      // disable motor
      motor.Stop();
      motor.Disable();

      // print results
      String timeTraveled = String((micros() - startTime) / 1000000.0, 3);
      String distTraveled = String(motorEnc.read() / (double) CPR * wheelCircumference, 1);
      Serial.println("Acc dist traveled: " + distTraveled + "cm\tAcc time traveled: " + timeTraveled + "s");

      // reset vars
      running = false;
      controls.reset();
      motorEnc.write(0);
      lastEncVal = 0;
      lastPosPIDTime = -2 * posPIDInterval;
      lastVelPIDTime = -2 * velPIDInterval;
      delete traj;
      traj = nullptr;
    }
  } else if(controls.ctrlsActive()) {
    delay(10);
    
    if(controls.update()) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Target Dist: " + String(targetDInM, 3));
      display.println("Target Time: " + String(targetT, 3));
      display.display();
    }

    // check if ready to run after ts updated
    if(!controls.ctrlsActive()) {
      // if ctrls disabled bc ready to run,
      targetD = round(targetDInM * 100 / wheelCircumference * CPR);

      delete traj;
      traj = new Trajectory(targetD, targetT);

      lastEncVal = 0;
      lastPosPIDTime = -2 * posPIDInterval;
      lastVelPIDTime = -2 * velPIDInterval;
    }
  } else {
    delay(10);

    if(digitalRead(START_BTN_PIN) == LOW) {
      running = true;
      motorEnc.write(0);
      motor.Enable();

      startTime = micros();
    }
  }
}